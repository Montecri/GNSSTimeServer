/* mac_hostname.h - deterministic, fully reversible MAC -> hostname.
 *
 * Public API for turning a device's 6-byte MAC address into a human-
 * readable hostname such as "noble-abbot-marc-hark", and back again.
 * This is handy on networks where you'd otherwise have to recognize
 * devices by raw MAC (e.g. in a DHCP client list or router admin
 * page) - the generated name is easier to read off a small display
 * or say out loud, and still uniquely identifies the device.
 *
 * v3 scheme: 48 bits = PREFIX 8 | TITLE 7 | GIVEN 17 | SURNAME 16.
 * Bijective over the entire 2^48 MAC space: every MAC gets a distinct
 * hostname (prefix-title-first-last) and every generated hostname
 * decodes back to exactly its MAC. "Bijective" here means it's a
 * one-to-one, two-way mapping - no two MACs share a hostname, and no
 * information is lost, so mac_hostname_to_mac() can always recover
 * the original address. Dashed form is 15..31 chars.
 *
 * Regenerating tables: see tools/ (name_components.py + emit_c_data.py).
 * Host-side self-test:  tools/host_selftest.c (build line in that file).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>

#define MAC_HOSTNAME_MAX_LEN     31   /* longest dashed hostname          */
#define MAC_HOSTNAME_BUF_LEN     32   /* MAX_LEN + terminating NUL        */
#define MAC_HOSTNAME_N_TOKENS    4    /* prefix, title, first, last       */

/* Output style for the generated hostname. Currently only one style
 * exists (all lower-case words joined with dashes, e.g.
 * "noble-abbot-marc-hark") - the enum exists so a second style could
 * be added later without breaking the function signatures below. */
typedef enum {
    MAC_HOSTNAME_DASHED = 0,          /* lower-case, dash separated       */
} mac_hostname_format_t;

/* Encode a 6-byte MAC address into a hostname string.
 * mac: the 6 raw MAC bytes (e.g. from an Ethernet/Wi-Fi interface).
 * out/out_len: caller-supplied buffer and its size in bytes.
 * Returns the number of characters written (always >= 15) on success,
 * -1 on bad arguments or if out_len is too small, -2 if the internal
 * word tables are corrupt (should not happen in normal use). */
int mac_hostname_from_mac(const uint8_t mac[6], char *out, size_t out_len,
                          mac_hostname_format_t format);

/* Decode a hostname string back into its original 6-byte MAC address.
 * Case-insensitive, so "Noble-Abbot-Marc-Hark" and
 * "noble-abbot-marc-hark" decode the same way.
 * Returns 0 on success, -1 if the string isn't a hostname this
 * scheme could have generated (wrong shape, unknown words, etc.). */
int mac_hostname_to_mac(const char *hostname, uint8_t mac[6]);

/* Split a dashed hostname like "noble-abbot-marc-hark" into its 4
 * word tokens (prefix, title, given, surname). This does not copy
 * the text - tokens[i] just points into the input string, so the
 * input must stay valid while you use the results.
 * Returns MAC_HOSTNAME_N_TOKENS (4) on success, -1 if the string
 * doesn't split cleanly into exactly 4 dash-separated tokens. */
int mac_hostname_split(const char *hostname,
                       const char *tokens[MAC_HOSTNAME_N_TOKENS],
                       size_t token_lens[MAC_HOSTNAME_N_TOKENS]);

/* Format a hostname as two shorter lines, useful for small displays
 * (e.g. a 2-line OLED status screen) that can't fit the full dashed
 * string on one line:
 *   line1 = "prefix-title" (<= 13 chars)
 *   line2 = "first-last"   (<= 17 chars)
 * Buffers must be large enough for the text plus the NUL terminator:
 * line1_len >= 14, line2_len >= 18. Returns 0 on success, -1 if the
 * hostname is invalid or a buffer is too small. */
int mac_hostname_display_lines(const char *hostname,
                               char *line1, size_t line1_len,
                               char *line2, size_t line2_len);

#ifdef ESP_PLATFORM
/* The functions below only exist when building for an ESP32 (ESP-IDF)
 * target - they're convenience wrappers that hook this codec up to
 * the device's real network interface. On other platforms, callers
 * are expected to get MAC bytes themselves and use the functions
 * above directly. */
#include "esp_err.h"
#include "esp_netif.h"

/* Convenience wrapper: reads the device's Ethernet MAC address
 * (burned into eFuse at the factory - the same address seen on the
 * wire by other devices on the network) and encodes it as a
 * hostname in one call. */
esp_err_t mac_hostname_get(char *out, size_t out_len,
                           mac_hostname_format_t format);

/* Applies the generated hostname to a network interface so it shows
 * up in DHCP and mDNS as this device's name. Call this after
 * esp_eth_start() so the interface exists. Also logs success, or a
 * hint about CONFIG_LWIP_MAX_HOSTNAME_LEN if the name doesn't fit. */
void mac_hostname_apply(esp_netif_t *netif, const char *hostname);
#endif /* ESP_PLATFORM */
