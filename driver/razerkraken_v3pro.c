// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Razer Kraken V3 Pro: the HyperSpeed dongle's CDC control channel. See razerkraken_v3pro.h for
 * the protocol.
 *
 * Findings this code is built on (each was tested on the hardware before it was written here):
 * - The dongle pushes notifications (battery, HyperSense, link) only while DTR is raised, and
 *   nothing is queued while it isn't, so the channel stays open for as long as we are bound.
 * - Line coding matters: with 115200 8N1 and RTS set, every frame after the first was silently
 *   dropped. 9600 8N1 with DTR|RTS, as cdc_acm leaves it, works.
 * - Acks and read replies come from the headset, 90-225 ms after the write, and never while the
 *   headset link is down: a frame sent then is lost. The link query is answered by the dongle.
 * - The dongle loses frames sent in bursts; writes are paced 250 ms apart.
 * - Lighting doesn't survive a headset power-off, and writes in the first ~15 s after power-on
 *   show but get overridden, so lighting is re-applied after every link-up until acked.
 * - Likewise a static colour that changes the mode (from the onboard effect, or from off) is
 *   never acked, and is sometimes overridden by the onboard effect a moment later. So any
 *   lighting change that isn't fully acked is retried the same way; static to static is acked.
 *   HyperSense, sidetone and power saving are kept by the headset.
 * - The dongle can stop relaying: the link reads up and writes complete, but the headset never
 *   answers. Only a replug fixes it, so it is detected and reported, not worked around.
 * - A bulk OUT that doesn't complete means the dongle stopped taking data; also replug-only.
 * The dongle is never reset from here.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ctype.h>
#include <linux/version.h>

#include "razerkraken_driver.h"
#include "razerkraken_v3pro.h"
#include "razercommon.h"

#define KV3P_EP_NOTIFY 0x85
#define KV3P_EP_IN 0x86
#define KV3P_EP_OUT 0x06

#define KV3P_FRAME_LEN 28
#define KV3P_BODY_LEN 20 /* bytes 7..26; byte 27 is the checksum */
#define KV3P_RX_URBS 2
#define KV3P_RX_LEN 128
#define KV3P_NOTIFY_LEN 16
#define KV3P_H4_MAX 260 /* 3-byte event header + 255 */
#define KV3P_REPLY_MAX 32

#define KV3P_WRITE_TIMEOUT_MS 1000
#define KV3P_REPLY_TIMEOUT_MS 1000
#define KV3P_LINK_TIMEOUT_MS 200
#define KV3P_WRITE_GAP_MS 250
#define KV3P_SETTLE_MS 1500
#define KV3P_RETRY_MS 5000
#define KV3P_TRIES 6
#define KV3P_JAM_THRESHOLD 3
#define KV3P_INIT_WAIT_MS 1500
#define KV3P_MAX_RX_ERRORS 20

#define KV3P_OPCODE_RAZER 0xFFF3
#define KV3P_OPCODE_LINK 0xFC9A

#define KV3P_CTRL_DTR 0x01
#define KV3P_CTRL_RTS 0x02

#define KV3P_CMD_COLOR 0x8C
#define KV3P_CMD_BRIGHTNESS 0x8E
#define KV3P_CMD_HYPERSENSE 0x8A
#define KV3P_NOTIFY_HYPERSENSE 0x90

#define KV3P_FALLBACK_SERIAL "XX000000052C"

/* system_wq is deprecated from 6.17 on */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
#define KV3P_WQ system_dfl_wq
#else
#define KV3P_WQ system_wq
#endif

static const u8 kv3p_prefix_set[3] = { 0x02, 0x21, 0x40 };
static const u8 kv3p_prefix_sidetone[3] = { 0x02, 0x21, 0x25 };
static const u8 kv3p_prefix_power[3] = { 0x02, 0x21, 0x26 };
static const u8 kv3p_prefix_setting[3] = { 0x02, 0x20, 0x40 };
static const u8 kv3p_prefix_firmware[3] = { 0x02, 0x20, 0x20 };
static const u8 kv3p_prefix_serial[3] = { 0x02, 0x20, 0x21 };
static const u8 kv3p_prefix_battery[3] = { 0x02, 0x20, 0x24 };
static const u8 kv3p_prefix_power_get[3] = { 0x02, 0x20, 0x26 };
static const u8 kv3p_link_query[4] = { 0x01, 0x9a, 0xfc, 0x00 };

enum kv3p_wait {
    KV3P_WAIT_NONE,
    KV3P_WAIT_ACK,      /* 02 21 40 CMD 01, from the headset */
    KV3P_WAIT_PREFIX,   /* an echo (02 21 25/26) or a read reply (02 20 xx) */
    KV3P_WAIT_LINK,     /* Command Complete for 0xFC9A, from the dongle */
};

struct kv3p_expect {
    enum kv3p_wait kind;
    u8 prefix[3];
    u8 cmd;
};

struct kv3p {
    struct kref ref;
    struct list_head node;
    struct usb_device *udev;

    /* Serialises frames on the bulk OUT pipe, the transport pointers and the desired state. */
    struct mutex io_lock;
    struct usb_interface *comm; /* NULL while the CDC side isn't bound */
    struct usb_interface *data;
    bool gone;
    bool suspended;
    bool wedged;
    bool has_tx;
    unsigned long last_tx;
    unsigned int unanswered;
    bool jammed;

    struct urb *rx_urb[KV3P_RX_URBS];
    struct urb *notify_urb;
    unsigned int rx_errors;

    /* Protects everything below: filled from URB completions. */
    spinlock_t rx_lock;
    u8 h4[KV3P_H4_MAX];
    unsigned int h4_len;
    struct kv3p_expect expect;
    struct completion reply;
    int link; /* -1 unknown, 0 down, 1 up */
    bool linkup_pending;
    bool batt_valid;
    u8 batt_state;
    u8 batt_pct;
    u16 batt_mv;
    bool fw_valid;
    u8 fw[3];
    bool serial_valid;
    char serial[16];
    bool hs_valid;
    u8 hs_on;
    u8 hs_level;
    bool ps_valid;
    u8 ps_on;
    u16 ps_seconds;
    bool st_valid;
    u8 st_on;
    u8 st_level;

    struct completion init_done;
    struct delayed_work refresh_work;
    unsigned int refresh_try;

    /* Desired lighting (io_lock). Re-applied after every link-up once something was set. */
    bool lighting_set;
    u8 effect;
    u8 rgb[3];
    u8 brightness;
};

static LIST_HEAD(kv3p_list);
static DEFINE_MUTEX(kv3p_list_lock);
static struct usb_driver kv3p_cdc_driver;

/* ------------------------------------------------------------------------- */
/* Context                                                                   */
/* ------------------------------------------------------------------------- */

static void kv3p_refresh_work(struct work_struct *work);

struct kv3p *kv3p_get(struct usb_device *udev)
{
    struct kv3p *ctx;

    mutex_lock(&kv3p_list_lock);
    list_for_each_entry(ctx, &kv3p_list, node) {
        if (ctx->udev == udev) {
            kref_get(&ctx->ref);
            goto out;
        }
    }

    ctx = kzalloc_obj(*ctx);
    if (!ctx)
        goto out;

    kref_init(&ctx->ref);
    ctx->udev = usb_get_dev(udev);
    mutex_init(&ctx->io_lock);
    spin_lock_init(&ctx->rx_lock);
    init_completion(&ctx->reply);
    init_completion(&ctx->init_done);
    INIT_DELAYED_WORK(&ctx->refresh_work, kv3p_refresh_work);
    ctx->link = -1;
    /* The headset boots into its onboard effect. */
    ctx->effect = KV3P_EFFECT_SPECTRUM;
    ctx->brightness = 0xFF;
    list_add(&ctx->node, &kv3p_list);
out:
    mutex_unlock(&kv3p_list_lock);
    return ctx;
}

static void kv3p_release(struct kref *ref)
{
    struct kv3p *ctx = container_of(ref, struct kv3p, ref);

    /* Called with kv3p_list_lock held (kref_put_mutex). */
    list_del(&ctx->node);
    mutex_unlock(&kv3p_list_lock);
    usb_put_dev(ctx->udev);
    kfree(ctx);
}

void kv3p_put(struct kv3p *ctx)
{
    if (ctx)
        kref_put_mutex(&ctx->ref, kv3p_release, &kv3p_list_lock);
}

/* ------------------------------------------------------------------------- */
/* Frames                                                                    */
/* ------------------------------------------------------------------------- */

static void kv3p_build(u8 *frame, const u8 *prefix, const u8 *body, size_t body_len)
{
    unsigned int i;
    u8 sum = 0;

    memset(frame, 0, KV3P_FRAME_LEN);
    frame[0] = 0x01; /* H4 command */
    frame[1] = KV3P_OPCODE_RAZER & 0xFF;
    frame[2] = KV3P_OPCODE_RAZER >> 8;
    frame[3] = KV3P_FRAME_LEN - 4;
    memcpy(&frame[4], prefix, 3);
    memcpy(&frame[7], body, min_t(size_t, body_len, KV3P_BODY_LEN));
    for (i = 7; i < KV3P_FRAME_LEN - 1; i++)
        sum += frame[i];
    frame[KV3P_FRAME_LEN - 1] = (u8)-sum;
}

static void kv3p_frame_color(u8 *frame, const u8 *rgb)
{
    const u8 body[] = { KV3P_CMD_COLOR, 0x00, 0x00, 0x08, 0x00, rgb[0], rgb[1], rgb[2] };

    kv3p_build(frame, kv3p_prefix_set, body, sizeof(body));
}

static void kv3p_frame_release(u8 *frame)
{
    const u8 body[] = { KV3P_CMD_COLOR, 0x00, 0x00, 0x03 };

    kv3p_build(frame, kv3p_prefix_set, body, sizeof(body));
}

static void kv3p_frame_brightness(u8 *frame, u8 level)
{
    const u8 body[] = { KV3P_CMD_BRIGHTNESS, 0x00, 0x00, level };

    kv3p_build(frame, kv3p_prefix_set, body, sizeof(body));
}

static void kv3p_frame_hypersense(u8 *frame, u8 on, u8 level)
{
    const u8 body[] = { KV3P_CMD_HYPERSENSE, 0x00, on, level };

    kv3p_build(frame, kv3p_prefix_set, body, sizeof(body));
}

static void kv3p_frame_sidetone(u8 *frame, u8 on, u8 level)
{
    const u8 body[] = { on, level };

    kv3p_build(frame, kv3p_prefix_sidetone, body, sizeof(body));
}

static void kv3p_frame_power(u8 *frame, u8 on, u16 seconds)
{
    const u8 body[] = { on, seconds & 0xFF, seconds >> 8 };

    kv3p_build(frame, kv3p_prefix_power, body, sizeof(body));
}

static void kv3p_frame_get(u8 *frame, const u8 *prefix)
{
    kv3p_build(frame, prefix, NULL, 0);
}

/* ------------------------------------------------------------------------- */
/* Receive (URB completion context)                                          */
/* ------------------------------------------------------------------------- */

static bool kv3p_body_ok(const u8 *body, unsigned int len)
{
    u8 sum = 0;

    while (len--)
        sum += *body++;
    return sum == 0;
}

/* rx_lock held */
static void kv3p_set_link(struct kv3p *ctx, int up)
{
    int was = ctx->link;

    ctx->link = up;
    /* A reconnect: re-read and re-apply once the headset has settled. Unknown -> up is the
     * probe/resume refresh's own link query, and that refresh is already doing the work. */
    if (up == 1 && was == 0) {
        ctx->linkup_pending = true;
        mod_delayed_work(KV3P_WQ, &ctx->refresh_work, msecs_to_jiffies(KV3P_SETTLE_MS));
    } else if (up == 0 && was != 0) {
        /* Frames sent now are dropped; re-apply waits for the next link-up. */
        cancel_delayed_work(&ctx->refresh_work);
    }
}

/* rx_lock held */
static void kv3p_match(struct kv3p *ctx, enum kv3p_wait kind, const u8 *prefix, u8 cmd)
{
    const struct kv3p_expect *e = &ctx->expect;

    if (e->kind != kind)
        return;
    if (kind != KV3P_WAIT_LINK && memcmp(e->prefix, prefix, 3))
        return;
    if (kind == KV3P_WAIT_ACK && e->cmd != cmd)
        return;
    complete(&ctx->reply);
}

/* rx_lock held. `pl` is the Command Complete's return parameters after the status byte. */
static void kv3p_rx_razer(struct kv3p *ctx, const u8 *pl, unsigned int n)
{
    bool cs_ok = kv3p_body_ok(&pl[3], n - 3);

    if (!memcmp(pl, kv3p_prefix_set, 3)) {
        if (cs_ok)
            kv3p_match(ctx, KV3P_WAIT_ACK, pl, pl[3]);
        return;
    }

    if (!memcmp(pl, kv3p_prefix_battery, 3) && n >= 7) {
        u16 mv = (pl[4] << 8) | pl[5];

        ctx->batt_state = pl[3];
        /* The first report after a link-up carries 0 mV and no usable level. */
        if (mv) {
            ctx->batt_mv = mv;
            ctx->batt_pct = min_t(u8, pl[6], 100);
            ctx->batt_valid = true;
        }
    } else if (!memcmp(pl, kv3p_prefix_setting, 3) && pl[3] == KV3P_NOTIFY_HYPERSENSE && cs_ok && n >= 8) {
        ctx->hs_on = pl[6];
        if (pl[7])
            ctx->hs_level = pl[7];
        ctx->hs_valid = true;
    } else if (!memcmp(pl, kv3p_prefix_firmware, 3) && n >= 6) {
        memcpy(ctx->fw, &pl[3], 3);
        ctx->fw_valid = true;
    } else if (!memcmp(pl, kv3p_prefix_serial, 3) && n > 3) {
        unsigned int i;

        for (i = 0; i < sizeof(ctx->serial) - 1 && 3 + i < n && pl[3 + i]; i++)
            ctx->serial[i] = isalnum(pl[3 + i]) ? pl[3 + i] : '_';
        ctx->serial[i] = '\0';
        ctx->serial_valid = i > 0;
    } else if ((!memcmp(pl, kv3p_prefix_power_get, 3) || (!memcmp(pl, kv3p_prefix_power, 3) && cs_ok)) && n >= 6) {
        ctx->ps_on = pl[3];
        ctx->ps_seconds = pl[4] | (pl[5] << 8);
        ctx->ps_valid = true;
    } else if (!memcmp(pl, kv3p_prefix_sidetone, 3) && cs_ok && n >= 5) {
        ctx->st_on = pl[3];
        ctx->st_level = pl[4];
        ctx->st_valid = true;
    } else if (pl[0] == 0x02 && pl[1] == 0x21 && !cs_ok) {
        return; /* a mangled echo is not an answer */
    }

    kv3p_match(ctx, KV3P_WAIT_PREFIX, pl, 0);
}

/* rx_lock held; `p` is one complete H4 event */
static void kv3p_rx_event(struct kv3p *ctx, const u8 *p, unsigned int len)
{
    unsigned int n;
    u16 opcode;

    dev_dbg(&ctx->udev->dev, "kraken v3 pro: rx %*phN\n", min_t(int, len, 64), p);

    if (p[1] == 0xFF && len >= 4) {
        /* Vendor event: e8 headset connected, e9 disconnected */
        if (p[3] == 0xE8)
            kv3p_set_link(ctx, 1);
        else if (p[3] == 0xE9)
            kv3p_set_link(ctx, 0);
        return;
    }
    if (p[1] != 0x0E || len < 7)
        return;

    opcode = p[4] | (p[5] << 8);
    n = len - 7;
    if (opcode == KV3P_OPCODE_LINK) {
        /* 00 down, 01 up; 02 was seen while busy and says nothing */
        if (n >= 1 && p[7] <= 1)
            kv3p_set_link(ctx, p[7]);
        kv3p_match(ctx, KV3P_WAIT_LINK, NULL, 0);
        return;
    }
    if (opcode != KV3P_OPCODE_RAZER) {
        if (p[6])
            dev_dbg(&ctx->udev->dev, "kraken v3 pro: dongle rejected opcode 0x%04x (status %u)\n", opcode, p[6]);
        return;
    }
    if (p[6] || n < 5)
        return;

    /* The headset answered through the dongle, so the relay works and the link is up. */
    WRITE_ONCE(ctx->unanswered, 0);
    if (READ_ONCE(ctx->jammed)) {
        WRITE_ONCE(ctx->jammed, false);
        dev_info(&ctx->udev->dev, "kraken v3 pro: the dongle is relaying to the headset again\n");
    }
    if (ctx->link != 1)
        kv3p_set_link(ctx, 1);

    kv3p_rx_razer(ctx, &p[7], n);
}

static void kv3p_rx_feed(struct kv3p *ctx, const u8 *data, unsigned int len)
{
    unsigned long flags;
    unsigned int need, skip;

    spin_lock_irqsave(&ctx->rx_lock, flags);
    while (len) {
        unsigned int take = min_t(unsigned int, len, sizeof(ctx->h4) - ctx->h4_len);

        memcpy(&ctx->h4[ctx->h4_len], data, take);
        ctx->h4_len += take;
        data += take;
        len -= take;

        for (;;) {
            if (!ctx->h4_len)
                break;
            if (ctx->h4[0] != 0x04) {
                /* Not an H4 event: drop bytes up to the next plausible packet start. */
                for (skip = 1; skip < ctx->h4_len && ctx->h4[skip] != 0x04; skip++)
                    ;
                memmove(ctx->h4, &ctx->h4[skip], ctx->h4_len - skip);
                ctx->h4_len -= skip;
                continue;
            }
            if (ctx->h4_len < 3)
                break;
            need = 3 + ctx->h4[2];
            if (ctx->h4_len < need)
                break;
            kv3p_rx_event(ctx, ctx->h4, need);
            memmove(ctx->h4, &ctx->h4[need], ctx->h4_len - need);
            ctx->h4_len -= need;
        }
    }
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
}

static void kv3p_resubmit(struct kv3p *ctx, struct urb *urb)
{
    int ret;

    if (READ_ONCE(ctx->gone) || READ_ONCE(ctx->suspended))
        return;
    ret = usb_submit_urb(urb, GFP_ATOMIC);
    if (ret && ret != -EPERM && ret != -ENODEV)
        dev_err(&ctx->udev->dev, "kraken v3 pro: can't resubmit a read: %d\n", ret);
}

static bool kv3p_urb_status_ok(struct kv3p *ctx, struct urb *urb)
{
    switch (urb->status) {
    case 0:
        ctx->rx_errors = 0;
        return true;
    case -ENOENT:
    case -ECONNRESET:
    case -ESHUTDOWN:
        return false; /* killed or gone */
    default:
        if (++ctx->rx_errors > KV3P_MAX_RX_ERRORS) {
            dev_err(&ctx->udev->dev, "kraken v3 pro: reads keep failing (%d); stopped\n", urb->status);
            return false;
        }
        return true;
    }
}

static void kv3p_rx_complete(struct urb *urb)
{
    struct kv3p *ctx = urb->context;

    if (!kv3p_urb_status_ok(ctx, urb))
        return;
    if (!urb->status)
        kv3p_rx_feed(ctx, urb->transfer_buffer, urb->actual_length);
    kv3p_resubmit(ctx, urb);
}

static void kv3p_notify_complete(struct urb *urb)
{
    struct kv3p *ctx = urb->context;

    /* CDC SERIAL_STATE, sent at every DTR change. Nothing to act on, but like cdc_acm we keep
     * the endpoint polled. */
    if (!kv3p_urb_status_ok(ctx, urb))
        return;
    kv3p_resubmit(ctx, urb);
}

/* ------------------------------------------------------------------------- */
/* Transmit (process context)                                                */
/* ------------------------------------------------------------------------- */

/*
 * Write one frame and wait for its response. io_lock held.
 * Returns 0 when the response arrived, -ETIMEDOUT when it didn't, -ENODEV when the CDC side is
 * unbound, -EIO when the dongle is wedged or the write failed.
 */
static int kv3p_xfer(struct kv3p *ctx, const u8 *frame, int len, const struct kv3p_expect *expect,
                     unsigned int timeout_ms)
{
    unsigned long flags, due;
    int actual = 0, ret;
    u8 *buf;

    lockdep_assert_held(&ctx->io_lock);
    if (!ctx->data || ctx->gone || ctx->suspended)
        return -ENODEV;
    if (ctx->wedged)
        return -EIO;

    if (ctx->has_tx) {
        due = ctx->last_tx + msecs_to_jiffies(KV3P_WRITE_GAP_MS);
        if (time_before(jiffies, due))
            msleep(jiffies_to_msecs(due - jiffies));
    }

    buf = kmemdup(frame, len, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    spin_lock_irqsave(&ctx->rx_lock, flags);
    ctx->expect = *expect;
    reinit_completion(&ctx->reply);
    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    dev_dbg(&ctx->udev->dev, "kraken v3 pro: tx %*phN\n", len, frame);
    ret = usb_bulk_msg(ctx->udev, usb_sndbulkpipe(ctx->udev, KV3P_EP_OUT), buf, len, &actual,
                       KV3P_WRITE_TIMEOUT_MS);
    kfree(buf);
    ctx->last_tx = jiffies;
    ctx->has_tx = true;

    if (ret == -ETIMEDOUT) {
        ctx->wedged = true;
        dev_err(&ctx->data->dev,
                "kraken v3 pro: a write didn't complete within %d ms: the dongle has stopped taking data. Replug it; nothing more is sent until then\n",
                KV3P_WRITE_TIMEOUT_MS);
        ret = -EIO;
    } else if (ret || actual != len) {
        dev_warn(&ctx->data->dev, "kraken v3 pro: write failed: %d (%d of %d bytes)\n", ret, actual, len);
        ret = ret ? ret : -EIO;
    } else {
        ret = wait_for_completion_timeout(&ctx->reply, msecs_to_jiffies(timeout_ms)) ? 0 : -ETIMEDOUT;
        if (READ_ONCE(ctx->gone))
            ret = -ENODEV;
        dev_dbg(&ctx->udev->dev, "kraken v3 pro: %s after %u ms\n", ret ? "no response" : "answered",
                jiffies_to_msecs(jiffies - ctx->last_tx));
    }

    spin_lock_irqsave(&ctx->rx_lock, flags);
    ctx->expect.kind = KV3P_WAIT_NONE;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    return ret;
}

/* A frame the headset must answer. io_lock held. -EAGAIN while the link is known to be down. */
static int kv3p_relay(struct kv3p *ctx, const u8 *frame, enum kv3p_wait kind, u8 cmd)
{
    struct kv3p_expect expect = { .kind = kind, .cmd = cmd };
    int ret;

    if (READ_ONCE(ctx->link) == 0)
        return -EAGAIN;
    memcpy(expect.prefix, &frame[4], 3);

    ret = kv3p_xfer(ctx, frame, KV3P_FRAME_LEN, &expect, KV3P_REPLY_TIMEOUT_MS);
    if (ret == -ETIMEDOUT && READ_ONCE(ctx->link) == 1) {
        ctx->unanswered++;
        if (ctx->unanswered == KV3P_JAM_THRESHOLD) {
            WRITE_ONCE(ctx->jammed, true);
            dev_warn(&ctx->udev->dev,
                     "kraken v3 pro: the headset link reads up but %u frames in a row went unanswered: the dongle has stopped relaying (audio is unaffected). Replug the dongle\n",
                     KV3P_JAM_THRESHOLD);
        }
    }
    return ret;
}

static int kv3p_read(struct kv3p *ctx, const u8 *prefix)
{
    u8 frame[KV3P_FRAME_LEN];

    kv3p_frame_get(frame, prefix);
    return kv3p_relay(ctx, frame, KV3P_WAIT_PREFIX, 0);
}

static int kv3p_query_link(struct kv3p *ctx)
{
    const struct kv3p_expect expect = { .kind = KV3P_WAIT_LINK };

    return kv3p_xfer(ctx, kv3p_link_query, sizeof(kv3p_link_query), &expect, KV3P_LINK_TIMEOUT_MS);
}

/* Send the desired lighting: brightness first, since after some power-ons colour is ignored until
 * brightness arrives. io_lock held. True when every frame was acked. */
static bool kv3p_apply_lighting(struct kv3p *ctx)
{
    u8 frame[KV3P_FRAME_LEN];
    int ret;

    kv3p_frame_brightness(frame, ctx->effect == KV3P_EFFECT_NONE ? 0 : ctx->brightness);
    ret = kv3p_relay(ctx, frame, KV3P_WAIT_ACK, KV3P_CMD_BRIGHTNESS);
    if (ctx->effect == KV3P_EFFECT_NONE || ret == -ENODEV || ret == -EIO || ret == -EAGAIN)
        return ret == 0;

    if (ctx->effect == KV3P_EFFECT_STATIC)
        kv3p_frame_color(frame, ctx->rgb);
    else
        kv3p_frame_release(frame);
    return kv3p_relay(ctx, frame, KV3P_WAIT_ACK, KV3P_CMD_COLOR) == 0 && ret == 0;
}

static bool kv3p_link_up(struct kv3p *ctx)
{
    return READ_ONCE(ctx->link) == 1;
}

/*
 * After the CDC side binds, and 1.5 s after every link-up: learn the link state, read what the
 * headset reports, then re-apply the lighting until it's acked (every 5 s, at most 6 times).
 */
static void kv3p_refresh_work(struct work_struct *work)
{
    struct kv3p *ctx = container_of(to_delayed_work(work), struct kv3p, refresh_work);
    unsigned long flags;
    bool linkup;

    mutex_lock(&ctx->io_lock);
    if (!ctx->data || ctx->gone || ctx->suspended)
        goto out;

    spin_lock_irqsave(&ctx->rx_lock, flags);
    linkup = ctx->linkup_pending;
    ctx->linkup_pending = false;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    if (linkup)
        ctx->refresh_try = 0;

    if (!kv3p_link_up(ctx))
        kv3p_query_link(ctx);

    if (kv3p_link_up(ctx) && !READ_ONCE(ctx->jammed)) {
        if (!ctx->fw_valid)
            kv3p_read(ctx, kv3p_prefix_firmware);
        if (!ctx->serial_valid)
            kv3p_read(ctx, kv3p_prefix_serial);
        kv3p_read(ctx, kv3p_prefix_battery);
        if (!ctx->ps_valid)
            kv3p_read(ctx, kv3p_prefix_power_get);
    }
    complete_all(&ctx->init_done);

    if (ctx->lighting_set && kv3p_link_up(ctx) && !READ_ONCE(ctx->jammed) &&
        !kv3p_apply_lighting(ctx) && ++ctx->refresh_try < KV3P_TRIES && kv3p_link_up(ctx))
        queue_delayed_work(KV3P_WQ, &ctx->refresh_work, msecs_to_jiffies(KV3P_RETRY_MS));
out:
    mutex_unlock(&ctx->io_lock);
}

/* ------------------------------------------------------------------------- */
/* CDC side: the usb_driver that owns interfaces 4 and 5                     */
/* ------------------------------------------------------------------------- */

static int kv3p_start_io(struct kv3p *ctx)
{
    struct usb_cdc_line_coding coding = {
        .dwDTERate = cpu_to_le32(9600),
        .bCharFormat = USB_CDC_1_STOP_BITS,
        .bParityType = USB_CDC_NO_PARITY,
        .bDataBits = 8,
    };
    unsigned int i;
    int ret;

    WRITE_ONCE(ctx->suspended, false);
    ctx->rx_errors = 0;
    ctx->h4_len = 0;

    ret = usb_submit_urb(ctx->notify_urb, GFP_KERNEL);
    for (i = 0; !ret && i < KV3P_RX_URBS; i++)
        ret = usb_submit_urb(ctx->rx_urb[i], GFP_KERNEL);
    if (ret)
        return ret;

    ret = usb_control_msg_send(ctx->udev, 0, USB_CDC_REQ_SET_LINE_CODING,
                               USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT, 0, KV3P_IF_COMM,
                               &coding, sizeof(coding), USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
    if (ret)
        return ret;

    /* DTR is what makes the dongle send notifications at all. */
    return usb_control_msg_send(ctx->udev, 0, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
                                USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
                                KV3P_CTRL_DTR | KV3P_CTRL_RTS, KV3P_IF_COMM, NULL, 0,
                                USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
}

static void kv3p_stop_io(struct kv3p *ctx)
{
    unsigned int i;

    WRITE_ONCE(ctx->suspended, true);
    usb_kill_urb(ctx->notify_urb);
    for (i = 0; i < KV3P_RX_URBS; i++)
        usb_kill_urb(ctx->rx_urb[i]);
}

static void kv3p_free_urbs(struct kv3p *ctx)
{
    unsigned int i;

    if (ctx->notify_urb) {
        kfree(ctx->notify_urb->transfer_buffer);
        usb_free_urb(ctx->notify_urb);
        ctx->notify_urb = NULL;
    }
    for (i = 0; i < KV3P_RX_URBS; i++) {
        if (ctx->rx_urb[i]) {
            kfree(ctx->rx_urb[i]->transfer_buffer);
            usb_free_urb(ctx->rx_urb[i]);
            ctx->rx_urb[i] = NULL;
        }
    }
}

static int kv3p_alloc_urbs(struct kv3p *ctx, const struct usb_endpoint_descriptor *notify)
{
    unsigned int i;
    void *buf;

    ctx->notify_urb = usb_alloc_urb(0, GFP_KERNEL);
    buf = kmalloc(KV3P_NOTIFY_LEN, GFP_KERNEL);
    if (!ctx->notify_urb || !buf) {
        kfree(buf);
        return -ENOMEM;
    }
    usb_fill_int_urb(ctx->notify_urb, ctx->udev, usb_rcvintpipe(ctx->udev, KV3P_EP_NOTIFY), buf,
                     KV3P_NOTIFY_LEN, kv3p_notify_complete, ctx, notify->bInterval);

    for (i = 0; i < KV3P_RX_URBS; i++) {
        ctx->rx_urb[i] = usb_alloc_urb(0, GFP_KERNEL);
        buf = kmalloc(KV3P_RX_LEN, GFP_KERNEL);
        if (!ctx->rx_urb[i] || !buf) {
            kfree(buf);
            return -ENOMEM;
        }
        usb_fill_bulk_urb(ctx->rx_urb[i], ctx->udev, usb_rcvbulkpipe(ctx->udev, KV3P_EP_IN), buf,
                          KV3P_RX_LEN, kv3p_rx_complete, ctx);
    }
    return 0;
}

static int kv3p_cdc_probe(struct usb_interface *comm, const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(comm);
    struct usb_endpoint_descriptor *notify, *bulk_in, *bulk_out;
    struct usb_interface *data;
    struct kv3p *ctx;
    int ret;

    data = usb_ifnum_to_if(udev, KV3P_IF_DATA);
    if (!data)
        return -ENODEV;
    if (usb_find_int_in_endpoint(comm->cur_altsetting, &notify) ||
        usb_find_common_endpoints(data->cur_altsetting, &bulk_in, &bulk_out, NULL, NULL) ||
        notify->bEndpointAddress != KV3P_EP_NOTIFY || bulk_in->bEndpointAddress != KV3P_EP_IN ||
        bulk_out->bEndpointAddress != KV3P_EP_OUT) {
        dev_err(&comm->dev, "kraken v3 pro: unexpected CDC endpoints\n");
        return -ENODEV;
    }

    ctx = kv3p_get(udev);
    if (!ctx)
        return -ENOMEM;

    ret = usb_driver_claim_interface(&kv3p_cdc_driver, data, ctx);
    if (ret) {
        dev_err(&comm->dev, "kraken v3 pro: interface %d is busy: %d\n", KV3P_IF_DATA, ret);
        goto err_put;
    }
    usb_set_intfdata(comm, ctx);

    ret = kv3p_alloc_urbs(ctx, notify);
    if (ret)
        goto err_release;

    mutex_lock(&ctx->io_lock);
    ctx->comm = comm;
    ctx->data = data;
    ctx->gone = false;
    ctx->wedged = false;
    ctx->jammed = false;
    ctx->unanswered = 0;
    ctx->has_tx = false;
    ctx->link = -1;
    reinit_completion(&ctx->init_done);
    ret = kv3p_start_io(ctx);
    mutex_unlock(&ctx->io_lock);
    if (ret) {
        dev_err(&comm->dev, "kraken v3 pro: can't open the control channel: %d\n", ret);
        goto err_stop;
    }

    /* Now, even if a link reply already queued it for later. */
    mod_delayed_work(KV3P_WQ, &ctx->refresh_work, 0);
    dev_info(&comm->dev, "kraken v3 pro: control channel open\n");
    return 0;

err_stop:
    kv3p_stop_io(ctx);
    cancel_delayed_work_sync(&ctx->refresh_work); /* a link reply may have scheduled it */
    mutex_lock(&ctx->io_lock);
    ctx->comm = NULL;
    ctx->data = NULL;
    mutex_unlock(&ctx->io_lock);
err_release:
    kv3p_free_urbs(ctx);
    usb_set_intfdata(comm, NULL);
    usb_set_intfdata(data, NULL);
    usb_driver_release_interface(&kv3p_cdc_driver, data);
err_put:
    kv3p_put(ctx);
    return ret;
}

static void kv3p_cdc_disconnect(struct usb_interface *intf)
{
    struct kv3p *ctx = usb_get_intfdata(intf);
    struct usb_interface *comm, *data;
    unsigned long flags;

    /* Both interfaces are ours; the first one to go tears down, then releases the other. */
    if (!ctx)
        return;
    comm = ctx->comm;
    data = ctx->data;
    usb_set_intfdata(comm, NULL);
    usb_set_intfdata(data, NULL);

    WRITE_ONCE(ctx->gone, true);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    complete_all(&ctx->reply); /* wake a waiting sender; it sees `gone` */
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    kv3p_stop_io(ctx);
    cancel_delayed_work_sync(&ctx->refresh_work);

    mutex_lock(&ctx->io_lock);
    /* Lower DTR, as cdc_acm does on close. Fails harmlessly if the dongle was unplugged. */
    usb_control_msg_send(ctx->udev, 0, USB_CDC_REQ_SET_CONTROL_LINE_STATE,
                         USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT, 0, KV3P_IF_COMM, NULL, 0,
                         USB_CTRL_SET_TIMEOUT, GFP_KERNEL);
    ctx->comm = NULL;
    ctx->data = NULL;
    ctx->link = -1;
    complete_all(&ctx->init_done);
    mutex_unlock(&ctx->io_lock);

    kv3p_free_urbs(ctx);
    usb_driver_release_interface(&kv3p_cdc_driver, intf == comm ? data : comm);
    dev_info(&intf->dev, "kraken v3 pro: control channel closed\n");
    kv3p_put(ctx);
}

static int kv3p_cdc_suspend(struct usb_interface *intf, pm_message_t message)
{
    struct kv3p *ctx = usb_get_intfdata(intf);

    if (!ctx || intf != ctx->comm)
        return 0;
    mutex_lock(&ctx->io_lock); /* let a frame in flight finish */
    kv3p_stop_io(ctx);
    mutex_unlock(&ctx->io_lock);
    cancel_delayed_work_sync(&ctx->refresh_work);
    return 0;
}

static int kv3p_cdc_resume(struct usb_interface *intf)
{
    struct kv3p *ctx = usb_get_intfdata(intf);
    unsigned long flags;
    int ret;

    if (!ctx || intf != ctx->comm)
        return 0;
    mutex_lock(&ctx->io_lock);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    ctx->link = -1;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    ctx->refresh_try = 0;
    ret = kv3p_start_io(ctx);
    mutex_unlock(&ctx->io_lock);
    if (ret)
        dev_err(&intf->dev, "kraken v3 pro: can't reopen the control channel after resume: %d\n", ret);
    else
        mod_delayed_work(KV3P_WQ, &ctx->refresh_work, 0);
    return 0;
}

static int kv3p_cdc_pre_reset(struct usb_interface *intf)
{
    return kv3p_cdc_suspend(intf, PMSG_SUSPEND);
}

static int kv3p_cdc_post_reset(struct usb_interface *intf)
{
    return kv3p_cdc_resume(intf);
}

static const struct usb_device_id kv3p_cdc_ids[] = {
    { USB_DEVICE_INTERFACE_NUMBER(USB_VENDOR_ID_RAZER, USB_DEVICE_ID_RAZER_KRAKEN_V3_PRO, KV3P_IF_COMM) },
    { }
};

MODULE_DEVICE_TABLE(usb, kv3p_cdc_ids);

static struct usb_driver kv3p_cdc_driver = {
    .name = "razerkraken_cdc",
    .id_table = kv3p_cdc_ids,
    .probe = kv3p_cdc_probe,
    .disconnect = kv3p_cdc_disconnect,
    .suspend = kv3p_cdc_suspend,
    .resume = kv3p_cdc_resume,
    .reset_resume = kv3p_cdc_resume,
    .pre_reset = kv3p_cdc_pre_reset,
    .post_reset = kv3p_cdc_post_reset,
    .supports_autosuspend = 0,
};

int kv3p_register(void)
{
    return usb_register(&kv3p_cdc_driver);
}

void kv3p_unregister(void)
{
    usb_deregister(&kv3p_cdc_driver);
}

/* ------------------------------------------------------------------------- */
/* Sysfs, on the HID device                                                  */
/* ------------------------------------------------------------------------- */

/* Wait (bounded) for the first reads after the CDC side bound. */
static void kv3p_wait_init(struct kv3p *ctx)
{
    if (READ_ONCE(ctx->data))
        wait_for_completion_interruptible_timeout(&ctx->init_done, msecs_to_jiffies(KV3P_INIT_WAIT_MS));
}

ssize_t kv3p_show_serial(struct kv3p *ctx, char *buf)
{
    char serial[sizeof(ctx->serial)];
    unsigned long flags;
    bool valid;

    kv3p_wait_init(ctx);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    valid = ctx->serial_valid;
    memcpy(serial, ctx->serial, sizeof(serial));
    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    /* With the headset off nothing answers the read. Never random: the daemon keys on it. */
    return sysfs_emit(buf, "%s\n", valid ? serial : KV3P_FALLBACK_SERIAL);
}

ssize_t kv3p_show_firmware_version(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    u8 fw[3] = { 0, 0, 0 };

    kv3p_wait_init(ctx);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    if (ctx->fw_valid)
        memcpy(fw, ctx->fw, sizeof(fw));
    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    return sysfs_emit(buf, "v%u.%u.%u\n", fw[0], fw[1], fw[2]);
}

/* Send the desired lighting now; if it isn't fully acked, retry it from the refresh work every
 * 5 s until it is (at most 6 times). io_lock held. */
static void kv3p_push_lighting(struct kv3p *ctx)
{
    ctx->lighting_set = true;
    ctx->refresh_try = 0;
    if (!ctx->data || ctx->wedged || READ_ONCE(ctx->jammed))
        return;
    if (!kv3p_apply_lighting(ctx) && kv3p_link_up(ctx))
        mod_delayed_work(KV3P_WQ, &ctx->refresh_work, msecs_to_jiffies(KV3P_RETRY_MS));
}

/* A lighting change: record it, then send it if the channel is up. Never fails for a missing
 * ack: the headset may be off or just powering on, and the retry or next link-up re-applies it. */
static ssize_t kv3p_store_lighting(struct kv3p *ctx, u8 effect, const u8 *rgb, size_t count)
{

    mutex_lock(&ctx->io_lock);
    ctx->effect = effect;
    if (rgb)
        memcpy(ctx->rgb, rgb, sizeof(ctx->rgb));
    kv3p_push_lighting(ctx);
    mutex_unlock(&ctx->io_lock);
    return count;
}

/* matrix_effect_none, _spectrum and _static. Static takes RGB, or RGB and brightness. */
ssize_t kv3p_store_effect(struct kv3p *ctx, u8 effect, const char *buf, size_t count)
{
    if (effect != KV3P_EFFECT_STATIC)
        return kv3p_store_lighting(ctx, effect, NULL, count);

    if (count != 3 && count != 4) {
        dev_warn(&ctx->udev->dev, "razerkraken: Static mode only accepts RGB (3byte) or RGB with intensity (4byte)\n");
        return -EINVAL;
    }
    if (count == 4) {
        mutex_lock(&ctx->io_lock);
        ctx->brightness = buf[3];
        mutex_unlock(&ctx->io_lock);
    }
    return kv3p_store_lighting(ctx, KV3P_EFFECT_STATIC, (const u8 *)buf, count);
}

ssize_t kv3p_show_effect_static(struct kv3p *ctx, char *buf)
{

    mutex_lock(&ctx->io_lock);
    memcpy(buf, ctx->rgb, 3);
    buf[3] = ctx->brightness;
    mutex_unlock(&ctx->io_lock);
    return 4;
}

ssize_t kv3p_show_current_effect(struct kv3p *ctx, char *buf)
{
    u8 effect;

    mutex_lock(&ctx->io_lock);
    effect = ctx->effect;
    mutex_unlock(&ctx->io_lock);
    return sysfs_emit(buf, "%02x\n", effect);
}

ssize_t kv3p_store_brightness(struct kv3p *ctx, const char *buf, size_t count)
{
    u8 frame[KV3P_FRAME_LEN];
    u8 level;
    int ret;

    ret = kstrtou8(buf, 10, &level);
    if (ret)
        return ret;

    mutex_lock(&ctx->io_lock);
    ctx->brightness = level;
    /* With the LEDs off ("none"), a brightness change is only remembered. */
    if (ctx->effect != KV3P_EFFECT_NONE) {
        ctx->lighting_set = true;
        if (ctx->data && !ctx->wedged && !READ_ONCE(ctx->jammed)) {
            kv3p_frame_brightness(frame, level);
            if (kv3p_relay(ctx, frame, KV3P_WAIT_ACK, KV3P_CMD_BRIGHTNESS) && kv3p_link_up(ctx))
                mod_delayed_work(KV3P_WQ, &ctx->refresh_work, msecs_to_jiffies(KV3P_RETRY_MS));
        }
    }
    mutex_unlock(&ctx->io_lock);
    return count;
}

ssize_t kv3p_show_brightness(struct kv3p *ctx, char *buf)
{
    u8 level;

    mutex_lock(&ctx->io_lock);
    level = ctx->brightness;
    mutex_unlock(&ctx->io_lock);
    return sysfs_emit(buf, "%u\n", level);
}

/* Read a value from the headset if it isn't known yet. */
static void kv3p_read_if_unknown(struct kv3p *ctx, const u8 *prefix, bool *valid)
{
    unsigned long flags;
    bool known;

    spin_lock_irqsave(&ctx->rx_lock, flags);
    known = *valid;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    if (known)
        return;

    mutex_lock(&ctx->io_lock);
    if (ctx->data && !ctx->wedged)
        kv3p_read(ctx, prefix);
    mutex_unlock(&ctx->io_lock);
}

ssize_t kv3p_show_charge_level(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    int level = -1;

    kv3p_read_if_unknown(ctx, kv3p_prefix_battery, &ctx->batt_valid);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    if (ctx->batt_valid)
        level = DIV_ROUND_CLOSEST(ctx->batt_pct * 255, 100);
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    return sysfs_emit(buf, "%d\n", level);
}

ssize_t kv3p_show_charge_status(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    int charging;

    spin_lock_irqsave(&ctx->rx_lock, flags);
    /* Bit 2 of the battery state: 05 on the charge cable, 01 on battery, 00 just after power-on. */
    charging = ctx->batt_valid && (ctx->batt_state & 0x04);
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    return sysfs_emit(buf, "%d\n", charging);
}

/* Map a store's result to an error for the caller: the settings below live in the headset, so
 * unlike lighting they can't be kept for later. */
static ssize_t kv3p_setting_result(int ret, size_t count)
{
    switch (ret) {
    case 0:
        return count;
    case -EAGAIN:
    case -ENODEV:
        return -ENODEV; /* headset off, or the channel isn't open */
    default:
        return -EIO;
    }
}

ssize_t kv3p_store_idle_time(struct kv3p *ctx, const char *buf, size_t count)
{
    u8 frame[KV3P_FRAME_LEN];
    unsigned long flags;
    unsigned int seconds;
    u8 on = 1;
    int ret;

    ret = kstrtouint(buf, 10, &seconds);
    if (ret)
        return ret;

    if (seconds == 0) {
        /* Off keeps the time, as Synapse does. */
        on = 0;
        spin_lock_irqsave(&ctx->rx_lock, flags);
        seconds = ctx->ps_valid ? ctx->ps_seconds : 900;
        spin_unlock_irqrestore(&ctx->rx_lock, flags);
    }
    /* The headset takes 15-60 min in 1 min steps. */
    seconds = clamp_t(unsigned int, DIV_ROUND_CLOSEST(seconds, 60) * 60, 900, 3600);

    kv3p_frame_power(frame, on, seconds);
    mutex_lock(&ctx->io_lock);
    ret = kv3p_relay(ctx, frame, KV3P_WAIT_PREFIX, 0);
    mutex_unlock(&ctx->io_lock);
    return kv3p_setting_result(ret, count);
}

ssize_t kv3p_show_idle_time(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    int seconds = -ENODATA;

    kv3p_read_if_unknown(ctx, kv3p_prefix_power_get, &ctx->ps_valid);
    spin_lock_irqsave(&ctx->rx_lock, flags);
    if (ctx->ps_valid)
        seconds = ctx->ps_on ? ctx->ps_seconds : 0;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    if (seconds < 0)
        return seconds;
    return sysfs_emit(buf, "%u\n", seconds);
}

ssize_t kv3p_store_haptic_intensity(struct kv3p *ctx, const char *buf, size_t count)
{
    u8 frame[KV3P_FRAME_LEN];
    unsigned long flags;
    u8 value, level;
    int ret;

    ret = kstrtou8(buf, 10, &value);
    if (ret)
        return ret;
    if (value > 3)
        return -EINVAL;

    /* 0 is off, keeping the level (as Synapse does); 1-3 are low, medium, high. */
    spin_lock_irqsave(&ctx->rx_lock, flags);
    level = value ? value : (ctx->hs_valid && ctx->hs_level ? ctx->hs_level : 1);
    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    kv3p_frame_hypersense(frame, value ? 1 : 0, level);
    mutex_lock(&ctx->io_lock);
    ret = kv3p_relay(ctx, frame, KV3P_WAIT_ACK, KV3P_CMD_HYPERSENSE);
    mutex_unlock(&ctx->io_lock);

    if (!ret) {
        /* The ack isn't an echo, so record what was set. */
        spin_lock_irqsave(&ctx->rx_lock, flags);
        ctx->hs_on = value ? 1 : 0;
        ctx->hs_level = level;
        ctx->hs_valid = true;
        spin_unlock_irqrestore(&ctx->rx_lock, flags);
    }
    return kv3p_setting_result(ret, count);
}

ssize_t kv3p_show_haptic_intensity(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    int value = -1;

    /* No read exists; known once set here or changed with the headset's button. */
    spin_lock_irqsave(&ctx->rx_lock, flags);
    if (ctx->hs_valid)
        value = ctx->hs_on ? ctx->hs_level : 0;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    return sysfs_emit(buf, "%d\n", value);
}

ssize_t kv3p_store_sidetone(struct kv3p *ctx, const char *buf, size_t count)
{
    u8 frame[KV3P_FRAME_LEN];
    unsigned long flags;
    u8 value, level;
    int ret;

    ret = kstrtou8(buf, 10, &value);
    if (ret)
        return ret;
    if (value > 5)
        return -EINVAL;

    /* 0 is off, keeping the level; 1-5 are on at the headset's levels 0-4. */
    spin_lock_irqsave(&ctx->rx_lock, flags);
    level = value ? value - 1 : (ctx->st_valid ? ctx->st_level : 2);
    spin_unlock_irqrestore(&ctx->rx_lock, flags);

    kv3p_frame_sidetone(frame, value ? 1 : 0, level);
    mutex_lock(&ctx->io_lock);
    ret = kv3p_relay(ctx, frame, KV3P_WAIT_PREFIX, 0);
    mutex_unlock(&ctx->io_lock);
    return kv3p_setting_result(ret, count);
}

ssize_t kv3p_show_sidetone(struct kv3p *ctx, char *buf)
{
    unsigned long flags;
    int value = -1;

    /* No read exists; known once set here. */
    spin_lock_irqsave(&ctx->rx_lock, flags);
    if (ctx->st_valid)
        value = ctx->st_on ? ctx->st_level + 1 : 0;
    spin_unlock_irqrestore(&ctx->rx_lock, flags);
    return sysfs_emit(buf, "%d\n", value);
}
