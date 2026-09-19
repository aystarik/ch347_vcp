# CH347 driver review — memory leaks, use-after-free and deadlocks

Scope: `mfd-ch347.c`, `spi-ch347.c`, `i2c-ch347.c`, `gpio-ch347.c`, `ch347.h`.
Method: full read of all four drivers, plus `git blame`/`git show` on the history to
separate long-standing code from recent regressions, plus verification of the relevant
kernel contracts (`spi_unregister_controller()`, USB anchors, `usb_set_intfdata()`).
Fixes: applied on branch `dev` — see *Status of fixes (applied)*.

## Verdict

Yes — there are real defects. Two of them are **regressions introduced by the two most
recent commits** (`20ea23a`, `259096f`), and one is a long-standing bug (`e718c6b`).
The most serious is a **reference-count leak in the MFD transfer pool**: every transmit is
released twice, which corrupts the TX pool accounting, can hand a still-in-flight DMA
buffer and URB to the next transfer, and aliases one buffer between concurrent transfers.
The second is a **double free / use-after-free of the SPI controller on a probe error
path**. On deadlocks: I found **no unconditional deadlock** — the lock ordering, callback
context and reset pairing are all sound (see *Deadlock assessment*). The only constructible
hang is a corrupted USB anchor list caused by the buffer-aliasing race in finding 1.

| # | Severity | Where | Kind |
|---|----------|-------|------|
| 1 | Critical | `mfd-ch347.c` (20ea23a) | TX buffer double-release → pool counter/bitmap corruption, in-flight buffer + URB reuse, spurious `-ENOMEM` |
| 2 | Critical | `spi-ch347.c:513-519` (259096f) | Controller reference double-put → UAF / double free on probe error |
| 3 | High | `mfd-ch347.c:195-196` + `:692-694` | Double free + UAF read on `ch347_init_buffers()` failure |
| 4 | Medium | `mfd-ch347.c:165` | TX semaphore initialised to the *RX* buffer count (4 instead of 8) |
| 5 | Low | `mfd-ch347.c:663, 692` | `usb_set_intfdata()` never cleared → dangling interface data |
| 6 | Low | `mfd-ch347.c:289/317`, `:524` | `ch347->interface` dereferenced after it is set to `NULL` (dormant under dynamic debug) |
| 7 | Low | `mfd-ch347.c:712-729` | `pre_reset`/`post_reset` do not NULL-check `ch347`; `post_reset` writes `errors` without `err_lock` |
| 8 | Low | `i2c-ch347.c:85` + `:177` | Read path reports `len+1` in `msgs[i].len` → 1-byte out-of-bounds heap read to userspace |
| 9 | Low (latent) | `mfd-ch347.c:271-282` | `ch347_free()` tears down URBs after dropping `io_mutex` |

---

## Status of fixes (applied)

All nine findings are fixed on branch `dev`. The four most recent upstream patches were
reworked in place and seven fix commits were added on top of `a81c3ed`. A clean
`make` now builds all four modules with no warnings against kernel 7.0.0 headers.

| Finding | Commit |
|---------|--------|
| 1 — TX buffer double-release | `6f3293b` (reworked patch) |
| 2 — SPI controller double-put | `1fa7787` (reworked patch) |
| 3 — `ch347_free_buffers()` double free | `7915577` |
| 4 — TX semaphore count | `7915577` |
| 5 — `usb_set_intfdata()` not cleared | `eae2d61` |
| 6 — logging through nullable `interface` | `eae2d61` |
| 7 — reset handlers | `eae2d61` |
| 8 — I2C read length | `571cc8b` |
| 9 — `ch347_free()` teardown window | `71e92f4` |

The reworked patches `a2c42ed` (MFD children removal) and `de008a7` (GPIO locking) needed
no correction.

The items from *Other bugs noticed in passing* are applied as well, together with kernel
7.0 compatibility (see below): `e384490` (GPIO value setters, `direction_output()` value,
`dbg_show()` locking), `54cacde` (SPI chip-select, errno, length truncation) and `6977171`
(I2C per-device speed cache).

### Kernel 7.0 compatibility

`gpio_chip.set()` and `set_multiple()` returned `void` up to 6.15 and return `int` from
6.16 on, so the driver did not compile against 7.0 ("assignment to `int (*)(...)` from
incompatible pointer type"). `gpio-ch347.c` now selects the signature with
`LINUX_VERSION_CODE`, following the upstream conversion pattern:

```c
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0))
#define CH347_GPIO_SET_FN(fn)		int fn
#define CH347_GPIO_SET_DONE(rc)		return (rc)
#else
#define CH347_GPIO_SET_FN(fn)		void fn
#define CH347_GPIO_SET_DONE(rc)		do { (void)(rc); return; } while (0)
#endif
```

On 6.16+ the setters report `-EIO` when the transfer fails; before 6.16 the status is
discarded, as that API cannot carry it. Both branches were syntax-checked, and the module
builds cleanly against the running 7.0.0 headers.

---

## 1. CRITICAL — TX URB buffer is released twice, while the write may still be in flight

**Location:** `mfd-ch347.c:309` (callback) and `mfd-ch347.c:494-498` (transfer success path).

`ch347_write_bulk_callback()` already returns the buffer to the pool:

```c
static void ch347_write_bulk_callback(struct urb *urb)
{
	struct ch347_tx_buffer *txb = urb->context;
	struct ch347_dev *ch347 = txb->ch347;
	...
	ch347_put_tx_buffer(ch347, txb);      /* line 309 */
}
```

Commit `20ea23a` added a *second* release on the synchronous path of `ch347_data_xfer()`:

```c
	/* Return the submitted TX buffer once the transfer is fully done */
	if (obuf && txb) {
		ch347_put_tx_buffer(ch347, txb);  /* line 496 — duplicate */
		txb = NULL;
	}
```

Before that commit the only success-path release was the callback (verified with
`git show 20ea23a^:mfd-ch347.c`), so this is a regression.

`ch347_put_tx_buffer()` clears a bit **and** does `up(&ch347->tx_limit_sem)`. Every transmit
therefore has a `down()`/`up()` imbalance — one `down`, two `up`s — which is an
unconditional resource-count leak, and the two releases are not even idempotent with
respect to the bitmap once another transfer has re-reserved the slot.
Consequences, in increasing order of severity:

**(a) The same DMA buffer is handed out while it is still in flight.**
`ch347_data_xfer()` only waits for the *RX* URB, never for the write to complete. On the
`obuf`-only paths — `ch347_set_cs()` (`spi-ch347.c:143`, `ch347_xfer(..., NULL, 0)`) and
`ch347_i2c_set_speed()` (`i2c-ch347.c:149`) — it submits and returns immediately. The early
`put` clears `txb_bitmap[index]`, and `find_first_zero_bit()` returns the *lowest* free
index, i.e. the same one, so the next call re-`memcpy()`s new data into
`txb->urb->buf_dma` and re-submits the same URB while the HCD may still be transferring
from that buffer. This corrupts outgoing SPI/I2C traffic (it defeats the whole purpose of
the 8-buffer pool); re-submitting the same URB then fails with `-EBUSY` and the core emits
`WARN_ONCE("URB %pK submitted while active")` (`drivers/usb/core/urb.c`). For
`obuf`+`ibuf` transfers the RX wait usually implies the write has completed, so the reuse
hazard there is a race rather than a certainty — but the counter imbalance below applies to
every transfer.

**(b) Two transfers can own the same buffer.** If transfer B grabs index 0 between A's
synchronous `put` and A's callback `put`, A's callback then executes
`clear_bit(0, txb_bitmap)` on B's reservation. A third transfer can then take index 0 as
well. The error path of `ch347_data_xfer()` (`mfd-ch347.c:504-508`) calls
`usb_kill_urb()` on that shared URB, killing an unrelated in-flight transfer.

**(c) The TX semaphore stops bounding the pool (permit leak).**

```c
static int __get_free_buf_index(struct semaphore *limit_sem, spinlock_t *lock,
				unsigned long *bitmap, unsigned count)
{
	if (down_interruptible(limit_sem))
		return count;                 /* down failed: nothing acquired */

	spin_lock_irqsave(lock, flags);
	index = find_first_zero_bit(bitmap, count);
	if (index < count)
		set_bit(index, bitmap);
	spin_unlock_irqrestore(lock, flags);

	return index;                     /* may return count == "no buffer" */
}
```

The function assumes *semaphore count == number of free bitmap bits*. The double `up()`
falsifies that: `tx_limit_sem` gains a permit on every transmit (`down` once, `up` twice),
so its value drifts upward without bound and no longer limits how many callers may hold
buffers. Once its value exceeds the 8-slot pool and enough callers are in flight to set all
8 bitmap bits, `find_first_zero_bit()` returns `count` **after** the permit was
successfully taken; `ch347_get_tx_buffer()` then returns `NULL` and `ch347_data_xfer()`
reports `-ENOMEM` while the permit is never returned (`txb == NULL`, so the `error:` path
does not `up()`):

```c
	txb = ch347_get_tx_buffer(ch347);
	if (!txb) {
		retval = -ENOMEM;   /* permit already consumed, never released */
		goto error;
	}
```

To be precise about the failure mode: this is a **leak, not a guaranteed hang**. Each
successful transfer still *adds* a permit, so the counter does not monotonically drain to
zero and the TX path cannot be starved indefinitely by this mechanism alone; the observable
symptom is intermittent spurious `-ENOMEM` I/O failures plus permanent loss of permits
under concurrency. The buffer aliasing in (a)/(b) is the more damaging part.

**Anchor corruption — the one constructible hang.** `usb_anchor_urb()` does no
already-anchored check; it unconditionally does `usb_get_urb()` + `list_add_tail()` on
`urb->anchor_list` (`drivers/usb/core/urb.c`). If transfer B picks up the same `txb` while
A's URB is still pending — exactly what the premature `put` allows — B re-adds an
already-linked list node, corrupting `ch347->submitted`, and its `usb_submit_urb()` then
returns `-EBUSY`. A corrupted anchor list can in turn make the
`do { ... } while (!surely_empty)` loop in `usb_kill_anchored_urbs()` (called from
`ch347_draw_down()`) fail to observe an empty list and spin indefinitely. That is the only
path to a real hang that I could construct, and it is a consequence of the same
double-release.

**Trigger:** the permit drift needs only traffic. The in-flight reuse (a) needs
back-to-back `obuf`-only transfers; the aliasing (b) and the anchor corruption need two or
more concurrent callers (e.g. several I2C clients, or `spidev` traffic alongside
`i2c-tools`), which is a normal way to use this adapter.

**Fix:** release the TX buffer in exactly one place. The correct place is the completion
callback (adding a wait for TX completion before release would defeat the async design),
so delete the synchronous `put`:

```diff
-	/* Return the submitted TX buffer once the transfer is fully done */
-	if (obuf && txb) {
-		ch347_put_tx_buffer(ch347, txb);
-		txb = NULL;
-	}
-
 	mutex_unlock(&ch347->io_mutex);
```

The `error:` path (`mfd-ch347.c:504-512`) must keep its `ch347_put_tx_buffer()`, since a
URB that was never successfully submitted never gets a callback. Optionally add a comment
in `ch347_write_bulk_callback()` stating that it owns the release.

---

## 2. CRITICAL — double put of the SPI controller reference on probe error

**Location:** `spi-ch347.c:495-519` (commit `259096f`).

```c
	rv = device_create_file(&controller->dev, &dev_attr_delete_device);
	if (rv) {
		dev_err(dev, "%s: Can not create 'delete_device' file: %d", __func__, rv);
		goto out_remove_new;
	}
	return 0;

out_remove_new:
	device_remove_file(&controller->dev, &dev_attr_new_device);
out_unregister:
	spi_unregister_controller(controller);
out_free:
	spi_controller_put(controller);
	return rv;
```

`spi_register_controller()` does **not** consume the reference taken by `spi_alloc_host()`,
but `spi_unregister_controller()` does: it performs the final `put_device()` which runs
`spi_controller_release()` → `kfree(ctlr)`. This is the documented asymmetry ("probing
comprises two steps, removal one") and the exact cause of the well-known
`bcm2835_spi_remove()` use-after-free discussed on LKML:

> Apparently the problem is that `spi_unregister_controller()` drops the last ref on the
> controller, causing it to be freed [...] `spi_unregister_controller()` → `put_device()` →
> `spi_controller_release()` → `kfree(ctlr)`
> — [Vladimir Oltean, LKML](https://lkml.org/lkml/2020/10/14/898)

So on either `device_create_file()` failure the code frees the controller inside
`spi_unregister_controller()` and then falls through to `spi_controller_put()`, which
decrements the kref of freed memory and can free the object a second time. Before
`259096f` this path used `devm_spi_register_controller()` and could not happen.

**Trigger:** either `device_create_file(&controller->dev, &dev_attr_new_device)` or
`..._delete_device` fails, i.e. low memory / sysfs failure during probe. Rare, but it is a
use-after-free with a kernel object, not a benign error return.

**Fix:** make the two cleanup levels mutually exclusive — an explicit
`spi_unregister_controller()` already releases the allocation reference, so only the
not-yet-registered path may call `spi_controller_put()`:

```c
out_remove_new:
	device_remove_file(&controller->dev, &dev_attr_new_device);
out_unregister:
	spi_unregister_controller(controller);
	return rv;                 /* final reference already dropped */
out_free:
	spi_controller_put(controller);
	return rv;
```

(`spi_register_controller()` failure still goes to `out_free`, which is correct; on failure
the controller was never registered.)

---

## 3. HIGH — double free + use-after-free when `ch347_init_buffers()` fails

**Location:** `mfd-ch347.c:142-157`, `:195-196`, `:669-671`, `:692-694`. Long-standing
(introduced in `e718c6b`, not by the recent commits).

`ch347_free_buffers()` frees the URBs but leaves the now-dangling pointers in
`ch347->rxb[i].urb` / `ch347->txb[i].urb`:

```c
	for (i = 0; i < CH347_RX_BUFFERS; ++i) {
		if (ch347->rxb[i].urb) {
			ch347_urb_free(ch347, ch347->rxb[i].urb);
		}                                 /* pointer not set to NULL */
	}
```

`ch347_init_buffers()` calls it on its own failure path and returns an error:

```c
out_free:
	ch347_free_buffers(ch347);
	return retval;
```

and `ch347_probe()` then calls it again through `ch347_free()`:

```c
	ret = ch347_init_buffers(ch347);
	if (ret)
		goto out_free;
	...
out_free:
	ch347_free(ch347);      /* -> ch347_draw_down() + ch347_free_buffers() again */
```

`ch347_free()` first calls `ch347_draw_down()`, which walks the dangling pointers
(`usb_kill_urb(ch347->rxb[i].urb->urb)` — read of freed memory), and then
`ch347_free_buffers()` frees each already-freed `struct ch347_urb` and coherent DMA buffer
a second time.

**Trigger:** any one allocation failure inside `ch347_init_buffers()` — `usb_alloc_urb()`
or `usb_alloc_coherent()` returning `NULL` under memory pressure for one of the 4 RX or
8 TX buffers. It is a probe-failure path only, but it turns a clean `-ENOMEM` into memory
corruption / a crash.

**Fix:** make the teardown idempotent (this also protects any future second caller):

```c
	for (i = 0; i < CH347_RX_BUFFERS; ++i) {
		if (ch347->rxb[i].urb) {
			ch347_urb_free(ch347, ch347->rxb[i].urb);
			ch347->rxb[i].urb = NULL;
		}
	}
	/* same for txb[] */
```

Alternatively, drop `ch347_free_buffers()` from `ch347_init_buffers()` entirely and let the
single `ch347_free()` in the probe error path do the cleanup — but only if the
`probe()` error path is guaranteed to run, so NULLing the slots is the safer fix.

---

## 4. MEDIUM — TX semaphore initialised with the RX buffer count

**Location:** `mfd-ch347.c:165` (long-standing).

```c
	sema_init(&ch347->rx_limit_sem, CH347_RX_BUFFERS);
	sema_init(&ch347->tx_limit_sem, CH347_RX_BUFFERS);   /* should be CH347_TX_BUFFERS */
```

There are `CH347_RX_BUFFERS == 4` RX buffers and `CH347_TX_BUFFERS == 8` TX buffers.
`ch347_probe()` already sets the correct value (`mfd-ch347.c:659`), but
`ch347_init_buffers()` immediately overwrites it with 4. Effect: at most half of the TX
pool can ever be in flight, and the `__get_free_buf_index()` invariant
("semaphore count == free bitmap entries") is false from the start, which is what lets
finding #1(c) reach the `index == count` state once the count drifts upward. This is a
latent bug on its own and a contributing factor to the TX permit leak.

**Fix:** `sema_init(&ch347->tx_limit_sem, CH347_TX_BUFFERS);` (and drop the duplicate
initialisation in `ch347_probe()`).

---

## 5. LOW — `usb_set_intfdata()` is never cleared

`ch347_probe()` installs the context (`mfd-ch347.c:663`) but neither the probe error path
(`:692-694`) nor `ch347_disconnect()` (`:612-625`) calls
`usb_set_intfdata(interface, NULL)`. After `ch347_free()` the interface still points at
freed memory. In practice the USB core does not call `suspend`/`pre_reset`/`disconnect` for
a failed probe, and a re-probe overwrites the pointer, so this is latent rather than
exploitable — but `ch347_suspend()`/`ch347_pre_reset()` unconditionally trust
`usb_get_intfdata()` (see #7), so the dangling pointer is only one core-behaviour change
away from a NULL/garbage dereference.

**Fix:** call `usb_set_intfdata(interface, NULL)` in `ch347_disconnect()` and on the
`ch347_probe()` error path.

---

## 6. LOW — `ch347->interface` dereferenced after being set to `NULL`

`ch347_disconnect()` and `ch347_free()` null the pointer *before* the URB teardown:

```c
	mutex_lock(&ch347->io_mutex);
	ch347->interface = NULL;
	mutex_unlock(&ch347->io_mutex);
	ch347_draw_down(ch347);
```

`ch347_draw_down()` kills the RX URBs, so their completion callbacks run with
`ch347->interface == NULL`, and both callbacks log through it:

```c
	dev_dbg(&ch347->interface->dev, ...);   /* mfd-ch347.c:289 and :317 */
```

This is dormant by default because `dev_dbg()` is compiled to a dynamic-debug jump label
and its arguments are not evaluated while the site is disabled — but enabling dynamic
debug for this driver turns disconnect into a NULL-pointer dereference. The same pattern
exists in `ch347_xfer()` (`:524`), which can be entered with `interface == NULL`.

**Fix:** keep a never-cleared `struct device *dev` in `struct ch347_dev` and log through
that (it is what `usb_get_dev()`/the interface device is for), instead of through the
nullable `interface`.

---

## 7. LOW — reset handlers lack the NULL guard and error-lock

```c
static int ch347_pre_reset(struct usb_interface *intf)
{
	struct ch347_dev *ch347 = usb_get_intfdata(intf);
	mutex_lock(&ch347->io_mutex);        /* no NULL check, unlike ch347_suspend() */
	ch347_draw_down(ch347);
	return 0;
}

static int ch347_post_reset(struct usb_interface *intf)
{
	struct ch347_dev *ch347 = usb_get_intfdata(intf);
	ch347->errors = -EPIPE;              /* written without err_lock */
	mutex_unlock(&ch347->io_mutex);
	return 0;
}
```

`ch347_suspend()` returns early when `usb_get_intfdata()` is `NULL`; the reset pair does
not. Every other access to `ch347->errors` uses `spin_lock_irq(&ch347->err_lock)`
(`mfd-ch347.c:380-386`, `:585-587`, `:303-305`), so `post_reset` should too.

The lock/unlock pairing across `pre_reset`/`post_reset` is otherwise correct: the driver's
`pre_reset` always returns 0 and the USB core always calls `post_reset` to undo it.
Likewise there is no lock-order inversion anywhere — `gpio.lock`, the SPI mutex and the
I2C mutex are all acquired *outside* `ch347->io_mutex` and nothing in the MFD layer calls
back into them — and no callback sleeps or takes `io_mutex`, so `usb_kill_urb()` under
`io_mutex` in `ch347_pre_reset()` cannot deadlock.

---

## 8. LOW (memory safety) — I2C read reports one byte too many

**Location:** `i2c-ch347.c:69-85`, used at `:177`. Long-standing (`2092c84`).

`ch347_i2c_read()` returns the *device byte count including the status byte*
(`bytestoread + 1`) rather than the number of bytes copied into `msg->buf`:

```c
		ret = ch347_xfer(..., ch347->ibuf, bytestoread + 1);
		if (ret > 0) {
			...
			memcpy(&msg->buf[byteoffset], &ch347->ibuf[1], bytestoread);
			byteoffset += bytestoread;
		}
	...
	return ret;                   /* == bytestoread + 1 */
```

and the caller assigns it straight to the message length:

```c
			ret = ch347_i2c_read(ch347, &msgs[i]);
			...
			msgs[i].len = ret;     /* i2c-ch347.c:177 */
```

For an N-byte read the core is told `len == N + 1`. `i2c-dev` allocates `msgs[i].buf` for
the user-requested length and copies `msgs[i].len` bytes back with `copy_to_user()`, so one
byte past the heap allocation is disclosed to userspace on every I2C read. The function
should return the accumulated `byteoffset` (or `0` on success), not the raw transfer
result.

---

## 9. LOW (latent) — `ch347_free()` tears down the URBs after dropping the lock

```c
static void ch347_free(struct ch347_dev *ch347)
{
	mutex_lock(&ch347->io_mutex);
	ch347->interface = NULL;
	mutex_unlock(&ch347->io_mutex);       /* lock dropped here ... */

	ch347_draw_down(ch347);               /* ... URBs are freed out here */
	ch347_free_buffers(ch347);
	...
}
```

A `ch347_data_xfer()` that acquires `io_mutex` immediately after the unlock observes
`interface == NULL`, bails out with `-ENODEV`, and then touches `txb`/`rxb` in its
`error:` block (`usb_kill_urb()`, `ch347_put_*_buffer()`) while `ch347_free()` may already
be freeing those same URBs — a use-after-free window rather than a deadlock. It is **not
reachable today**: `ch347_disconnect()` removes the MFD children first
(`mfd_remove_devices()`, which synchronises the I2C/SPI/GPIO removal paths), and the probe
failure path runs before any child exists. It becomes live if a child can ever call
`ch347_xfer()` after removal, so it is worth closing by holding `io_mutex` across
`ch347_draw_down()` and `ch347_free_buffers()`.

---

## Deadlock assessment

Explicit answer to the "deadlocks" part of the request: I found **no unconditional
deadlock**. What was checked:

* **Lock ordering is acyclic.** Every path takes a child lock (`gpio.lock`, the SPI
  `io_mutex`, the I2C `io_mutex`) *before* the MFD `ch347->io_mutex`, and the MFD layer
  never calls back into a child driver, so no ABBA cycle exists.
* **No sleeping under a spinlock.** The `err_lock`, `rxb_lock` and `txb_lock` sections only
  touch a bitmap/flag and call `complete()`, `up()` or `usb_submit_urb(GFP_ATOMIC)`.
* **No lock is held across a wait that needs the same lock.** The only wait under
  `io_mutex` is `wait_for_completion_timeout()`, and neither URB completion callback takes
  `io_mutex`, so `usb_kill_urb()`/`usb_kill_anchored_urbs()` under that mutex
  (`ch347_pre_reset()`, `ch347_free()`) cannot block against its own completion path.
* **Every acquisition is released on every path.** `ch347_data_xfer()` unlocks before each
  `goto error` that leaves the locked region and the `error:`/`exit:` labels are only
  reached unlocked; all early returns in `gpio-ch347.c` unlock; and
  `ch347_transfer_one_message()` unlocks on both the success and `msg_done` paths.
* **The `pre_reset`/`post_reset` mutex pair is balanced.** `ch347_pre_reset()` always
  returns 0 and the USB core calls `post_reset()` to undo a successful `pre_reset()`;
  `pre_reset()` is only called for a bound interface, so a failed probe cannot leave the
  mutex armed.
* **The long waits are bounded.** `wait_for_completion_timeout()` (1000 ms) and
  `usb_wait_anchor_empty_timeout()` (1000 ms) are the only potentially long waits, so
  `ch347_free()`/`ch347_disconnect()` cannot block indefinitely on `io_mutex`.
* **The RX semaphore cannot drain.** The RX pool has exactly one `down` and one `up` per
  buffer on every path, so `down_interruptible()` there cannot block forever.
* **The TX semaphore inflates rather than drains** (finding 1(c)), so it also cannot cause
  an indefinite block.

The only hang I could construct is the corrupted-anchor loop described in finding 1
(`usb_kill_anchored_urbs()` never observing an empty list after `usb_anchor_urb()`
double-links a still-pending URB). It is a consequence of the double-release, not an
independent lock problem.

---

## Verified correct (no action needed)

These were specifically checked and are sound:

* **RX pool get/put pairing** — `ch347_put_rx_buffer()` is called exactly once on every
  path of `ch347_data_xfer()` (success, timeout, submit failure, `-ENODEV`, parameter
  errors) and never from `ch347_read_bulk_callback()`, so there is no RX double-release.
* **RX completion/UAF on timeout** — `usb_kill_urb()` in the timeout branch returns only
  after the completion callback has finished, and `ch347_put_rx_buffer()` re-inits the
  completion afterwards, so no stale `complete()` can be delivered to the next owner of
  that slot.
* **URB lifetime** — `ch347_urb_alloc()`/`ch347_urb_free()` pair `usb_alloc_urb()` +
  `usb_alloc_coherent()` with `usb_kill_urb()` + `usb_free_coherent()` +
  `usb_free_urb()` correctly on all allocation-failure orders.
* **USB anchors (given a correctly released buffer)** — TX URBs are anchored on every
  submit and never explicitly unanchored, which is safe because the kernel disassociates a
  URB from its anchor automatically on completion
  ([USB Anchors](https://www.kernel.org/doc/Documentation/driver-api/usb/anchors.rst):
  "The association is maintained until an URB is finished by (successful) completion.
  Thus disassociation is automatic."). A buffer that is released only by its completion
  callback therefore never leaves a stale anchor entry. The anchor corruption in finding 1
  occurs only because the premature release lets a second transfer call `usb_anchor_urb()`
  on a URB that is still anchored (that function performs no already-anchored check).
* **`ch347->submitted` teardown** — `usb_wait_anchor_empty_timeout()` followed by
  `usb_kill_anchored_urbs()` is the correct idiom and is bounded by the 1 s timeout.
* **No sleeping in atomic context** — the two URB completion callbacks run in softirq
  context and only use `spin_lock_irqsave`, `complete()`, `up()` and
  `usb_submit_urb(GFP_ATOMIC)`; all mutexes are taken only from process context.
* **SPI `spi_finalize_current_message()`** is called after `mutex_unlock()`, so the
  controller callback cannot recurse into the driver while its mutex is held.
* **`ch347_i2c_xfer()` mutex balance** — every `goto exit` unlocks; `ch347_gpio_*()`
  lock/unlock balance was checked on all early-return paths.
* **`new_device_store()`** — `kstrdup()` is freed on every branch.

## Other bugs noticed in passing (now also fixed)

* `spi-ch347.c` — `set_cs()` returned early when `SPI_NO_CS` was **not** set
  (`if (!(spi->mode & SPI_NO_CS)) return;`), i.e. software chip-select was toggled for the
  wrong class of devices. Fixed in `54cacde`; the toggle is now also skipped when the
  controller is configured for hardware NSS.
* `gpio-ch347.c` — `ch347_gpio_direction_output()` ignored its `value` argument and copied
  the pin's *current* state instead. Fixed in `e384490`.
* `spi-ch347.c` — `ch347_spi_write(..., u16 data_len)` truncated `xfer->len` above 64 KiB
  (callers pass `u32`). Fixed in `54cacde`.
* `spi-ch347.c` — `ch347_transfer_setup()` returned bare `-1` instead of an errno. Fixed in
  `54cacde`.
* `i2c-ch347.c` — `static int speed_last` in `ch347_i2c_set_speed()` was shared across all
  adapters, so a second CH347 may never have had its bus speed programmed. Fixed in
  `6977171`.
* `gpio-ch347.c` — `ch347_gpio_dbg_show()` read `ch347->ibuf` outside `ch347->lock`. Fixed
  in `e384490` with a non-sleeping snapshot taken under `mutex_trylock()`.

## Order in which the fixes were applied

All of the following are applied on `dev`:

1. Removed the duplicate `ch347_put_tx_buffer()` in `ch347_data_xfer()` (#1) — restores the
   TX pool invariant, stops the in-flight buffer/URB aliasing and removes the only
   constructible hang.
2. Fixed the SPI error-path double put (#2) — a use-after-free of a kernel object.
3. Made `ch347_free_buffers()` idempotent by clearing each slot as it is released (#3).
4. Corrected the TX semaphore initial value and dropped its duplicate initialisation (#4).
5. Held `io_mutex` across the `ch347_free()` teardown (#9).
6. Cleared the interface data, moved logging to `usb_dev`, and hardened the reset handlers
   (#5, #6, #7).
7. Returned the real read length from `ch347_i2c_read()` (#8).
8. Applied the remaining items above plus kernel 7.0 compatibility (`e384490`, `54cacde`,
   `6977171`).

`dev` now builds all four modules cleanly against kernel 7.0.0 headers:

```
CC [M]  mfd-ch347.o
CC [M]  i2c-ch347.o
CC [M]  gpio-ch347.o
CC [M]  spi-ch347.o
LD [M]  mfd-ch347.ko i2c-ch347.ko gpio-ch347.ko spi-ch347.ko
```

The four reworked patches were rewritten in place, so `dev` no longer contains the original
`20ea23a`/`259096f` commits; `backup/pre-rework-259096f` preserves the old tip and
`origin/dev` still points at it, so publishing the rework needs
`git push --force-with-lease origin dev`.
