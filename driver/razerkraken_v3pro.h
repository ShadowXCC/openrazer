/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Razer Kraken V3 Pro (HyperSpeed dongle 1532:052c)
 *
 * The dongle's HID interface (3) only carries media keys; every setting travels over its CDC-ACM
 * interfaces (4: comm, 5: data) as Bluetooth HCI H4 packets with Razer payloads in vendor command
 * 0xFFF3. This part of razerkraken owns interfaces 4/5 through its own usb_driver (udev hands them
 * over from cdc_acm) and keeps the channel open, which the dongle requires for its notifications.
 * The HID side (razerkraken_driver.c) exposes the sysfs attributes and calls in here.
 *
 * Protocol, with thanks to paladin-devops (openrazer PR #2884) for the first lighting frames:
 *
 *   host -> dongle, 28 bytes, bulk OUT 0x06:
 *     01 f3 ff 18 | P0 P1 P2 | b7 | b8..b26 | CS     CS makes sum(b7..CS) == 0 mod 256
 *     02 21 40 8c 00 00 08 00 R G B   static colour        02 21 40 8c 00 00 03   onboard effect
 *     02 21 40 8e 00 00 LL            brightness           02 21 40 8a 00 ON LL   HyperSense
 *     02 21 25 ON LL                  sidetone             02 21 26 ON S0 S1      power saving (s, LE)
 *     02 20 20/21/24/26               read firmware, serial, battery, power saving
 *   host -> dongle, 4 bytes: 01 9a fc 00   link query, answered by the dongle itself
 *
 *   dongle -> host, bulk IN 0x86: HCI events. Command Complete for 0xFFF3 carries acks
 *   (02 21 40 CMD 01), echoes of 02 21 25/26, read replies, and unsolicited battery and HyperSense
 *   reports; Command Complete for 0xFC9A and vendor events e8/e9 carry the headset link state.
 */

#ifndef __HID_RAZER_KRAKEN_V3PRO_H
#define __HID_RAZER_KRAKEN_V3PRO_H

#include <linux/device.h>
#include <linux/usb.h>

#define KV3P_IF_COMM 4
#define KV3P_IF_DATA 5

struct kv3p;

int kv3p_register(void);
void kv3p_unregister(void);

/* Per-dongle context shared by the HID side and the CDC side; refcounted. */
struct kv3p *kv3p_get(struct usb_device *udev);
void kv3p_put(struct kv3p *ctx);

/* matrix_current_effect values, as razerkraken's effect byte: bit 0 on, bit 2 spectrum */
#define KV3P_EFFECT_NONE 0x00
#define KV3P_EFFECT_STATIC 0x01
#define KV3P_EFFECT_SPECTRUM 0x05 /* the headset's onboard rainbow */

/* Sysfs show/store bodies for the HID side's attributes (razerkraken_driver.c) */
ssize_t kv3p_show_serial(struct kv3p *ctx, char *buf);
ssize_t kv3p_show_firmware_version(struct kv3p *ctx, char *buf);
ssize_t kv3p_store_effect(struct kv3p *ctx, u8 effect, const char *buf, size_t count);
ssize_t kv3p_show_effect_static(struct kv3p *ctx, char *buf);
ssize_t kv3p_show_current_effect(struct kv3p *ctx, char *buf);
ssize_t kv3p_store_brightness(struct kv3p *ctx, const char *buf, size_t count);
ssize_t kv3p_show_brightness(struct kv3p *ctx, char *buf);
ssize_t kv3p_show_charge_level(struct kv3p *ctx, char *buf);
ssize_t kv3p_show_charge_status(struct kv3p *ctx, char *buf);
ssize_t kv3p_store_idle_time(struct kv3p *ctx, const char *buf, size_t count);
ssize_t kv3p_show_idle_time(struct kv3p *ctx, char *buf);
ssize_t kv3p_store_haptic_intensity(struct kv3p *ctx, const char *buf, size_t count);
ssize_t kv3p_show_haptic_intensity(struct kv3p *ctx, char *buf);
ssize_t kv3p_store_sidetone(struct kv3p *ctx, const char *buf, size_t count);
ssize_t kv3p_show_sidetone(struct kv3p *ctx, char *buf);

#endif
