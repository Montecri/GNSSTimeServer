/* mac_hostname_esp.c - ESP-IDF glue for the mac_hostname component.
 *
 * mac_hostname.c (and name_data.c) implement the actual MAC <->
 * hostname conversion and don't know anything about ESP32 hardware
 * or the ESP-IDF networking stack - they just work on plain byte
 * arrays and strings, so they can be tested on a regular PC (see
 * tools/host_selftest.c). This file is the "glue" layer that
 * connects that generic codec to a real ESP32 device: it reads the
 * device's actual MAC address out of hardware and pushes the
 * resulting hostname into the network stack so other devices see it.
 *
 * Implements the two helpers main.c uses:
 *   mac_hostname_get()   - derive the hostname from the Ethernet MAC
 *   mac_hostname_apply() - install it on a netif (after esp_eth_start)
 *
 * Both functions are compiled only when building for actual ESP-IDF
 * hardware (guarded by `#ifdef ESP_PLATFORM` below) - on a host build
 * (e.g. running the self-test on your PC) this file compiles to
 * nothing.
 */
#include "mac_hostname.h"

#ifdef ESP_PLATFORM
#include "esp_mac.h"   /* esp_read_mac(): reads MAC addresses out of hardware */
#include "esp_log.h"   /* ESP_LOGI/ESP_LOGE: ESP-IDF's tagged logging macros  */

/* Tag prefixed to every log line this file emits (shows up in the
 * serial console as e.g. "I (1234) mac_hostname: ..."), so log
 * output from this component is easy to filter/find. */
static const char *TAG = "mac_hostname";

/* Reads the device's real Ethernet MAC address from hardware and
 * turns it into a hostname string, in one call. `esp_err_t` is
 * ESP-IDF's standard error-code type (ESP_OK on success, or a
 * specific *_ERR_* / *_FAIL code otherwise) used across the whole
 * framework, not just here. */
esp_err_t mac_hostname_get(char *out, size_t out_len,
                           mac_hostname_format_t format)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';

    /* ESP_MAC_ETH is the eFuse-derived address the EMAC actually uses
     * on the wire, so the hostname matches what a sniffer / the PTP
     * clock identity shows.  Use ESP_MAC_EFUSE_FACTORY here instead if
     * you prefer the raw base MAC. */
    /* "eFuse" = one-time-programmable memory burned in at the factory
     * - this is where the chip's permanent, unique addresses live.
     * "EMAC" is the Ethernet MAC hardware block inside the chip that
     * actually sends/receives frames on the wire using this address. */
    uint8_t mac[6];
    esp_err_t err = esp_read_mac(mac, ESP_MAC_ETH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_read_mac failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Hand the raw MAC bytes to the hardware-independent codec in
     * mac_hostname.c to do the actual encoding. */
    int written = mac_hostname_from_mac(mac, out, out_len, format);
    if (written < 0) {
        ESP_LOGE(TAG, "hostname encode failed (%d), buffer %u bytes",
                 written, (unsigned)out_len);
        return ESP_FAIL;
    }
    /* MACSTR/MAC2STR are ESP-IDF helper macros that format a MAC byte
     * array as "aa:bb:cc:dd:ee:ff" in a printf-style call, so we can
     * log the source MAC alongside the hostname it produced. */
    ESP_LOGI(TAG, "derived hostname %s from " MACSTR, out, MAC2STR(mac));
    return ESP_OK;
}

/* Installs `hostname` on a network interface (`netif`) so it's what
 * DHCP/mDNS advertise to the rest of the network - e.g. what shows
 * up in your router's connected-devices list, or what you can `ping`
 * by name on the local network. Must be called after the interface
 * exists and Ethernet has started (esp_eth_start()), since there's
 * no netif to configure before that. This function only logs on
 * failure/success; it doesn't return an error code because the
 * caller (main.c) treats a failed hostname as non-fatal - the device
 * still works, it just falls back to its default name. */
void mac_hostname_apply(esp_netif_t *netif, const char *hostname)
{
    if (!netif || !hostname || hostname[0] == '\0') {
        ESP_LOGE(TAG, "apply: invalid netif or hostname");
        return;
    }
    esp_err_t err = esp_netif_set_hostname(netif, hostname);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "hostname applied: %s", hostname);
    } else {
        ESP_LOGE(TAG, "esp_netif_set_hostname failed (%s) - check that "
                 "CONFIG_LWIP_MAX_HOSTNAME_LEN >= %d",
                 esp_err_to_name(err), MAC_HOSTNAME_BUF_LEN);
    }
}
#endif /* ESP_PLATFORM */
