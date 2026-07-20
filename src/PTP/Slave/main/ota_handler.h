/* ============================================================================
 *  ota_handler.h  --  Browser-based OTA firmware upload for ESP32-P4
 *
 *  Adds two HTTP endpoints to the existing web_server.h:
 *
 *    POST /api/ota          -> receive a raw .bin firmware image and flash it.
 *                              Content-Type: application/octet-stream
 *                              Body: the .bin file produced by `idf.py build`
 *                              On success: returns 200 + "OK", then the device
 *                              reboots into the new image after ~1 second.
 *
 *    GET  /api/ota/info     -> JSON describing the running partition, the
 *                              candidate update partition, sizes, and whether
 *                              OTA is currently in progress.
 *
 *  ─── REQUIRED: OTA-CAPABLE PARTITION TABLE ──────────────────────────────────
 *
 *  Your current partitions table has a single 1 MB `factory` slot. OTA needs
 *  two `ota_*` slots plus an `otadata` slot. Replace your partitions CSV with
 *  the one shipped alongside this header (partitions_ota.csv) and configure
 *  it in menuconfig:
 *
 *      idf.py menuconfig
 *        Partition Table -> Custom partition table CSV
 *        Custom partition CSV file = partitions_ota.csv
 *        (Also bump flash size to 16 MB if not already.)
 *
 *  partitions_ota.csv (ALSO PROVIDED AS A SEPARATE FILE):
 *
 *      # Name,   Type, SubType, Offset,   Size,    Flags
 *      nvs,      data, nvs,     0x9000,   0x6000,
 *      phy_init, data, phy,     0xf000,   0x1000,
 *      otadata,  data, ota,     0x10000,  0x2000,
 *      ota_0,    app,  ota_0,   0x20000,  0x300000,
 *      ota_1,    app,  ota_1,   0x320000, 0x300000,
 *
 *  After flashing this new table once over UART (idf.py flash), all future
 *  updates can be performed over the network via this page. The first build
 *  flashed with the new table goes to ota_0; subsequent OTA uploads alternate
 *  between ota_0 and ota_1.
 *
 *  ─── INTEGRATION ────────────────────────────────────────────────────────────
 *
 *  ota_handler.h is included BY web_server.h automatically (see the include
 *  in that file). You only need to make sure both files sit next to main.c
 *  and that "app_update" is listed in your REQUIRES.
 *
 *  Project: https://github.com/Montecri/GNSSTimeServer
 * ============================================================================ */

#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <stdatomic.h>
#include "ota_credentials.h"

#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_app_format.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *OTA_TAG = "OTA";

/* OTA upload buffer. 4 KB is the sweet spot: large enough to keep flash
 * throughput high, small enough to never push the httpd task close to its
 * stack limit. The buffer is heap-allocated so it never lives on the stack. */
#define OTA_RECV_BUF_SIZE 4096

/* State flag so /api/ota/info can report "in progress". Atomic because the
 * upload handler uses atomic_compare_exchange_strong to claim it — a plain
 * volatile bool has a TOCTOU window where two concurrent (post-auth)
 * uploads can both pass an "if (busy) return 409" check before either
 * stores true. Atomic CAS closes that window. */
static _Atomic bool g_ota_in_progress = false;

/* Deferred reboot: scheduled by a tiny task so the HTTP response gets
 * flushed back to the browser before the reset. */
static void ota_reboot_task(void *arg)
{
    (void)arg;
    ESP_LOGW(OTA_TAG, "Rebooting into new firmware in 1 second...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/* ----------------------------------------------------------------------------
 *  Basic Auth helpers
 *
 *  Constant-time-on-equal-length compare. We deliberately do NOT try to
 *  mask the length difference — the Authorization header length is
 *  observable on the wire anyway, so hiding it inside the compare buys
 *  nothing. What matters is that, for two equal-length candidates, the
 *  loop visits every byte regardless of where the first mismatch occurs.
 * --------------------------------------------------------------------------- */
static bool ota_ct_streq(const char *a, const char *b)
{
    size_t la = strlen(a);
    size_t lb = strlen(b);
    if (la != lb)
        return false;
    int diff = 0;
    for (size_t i = 0; i < la; i++)
    {
        diff |= (unsigned char)a[i] ^ (unsigned char)b[i];
    }
    return diff == 0;
}

/* Verify HTTP Basic Auth against OTA_BASIC_AUTH_B64. On success returns
 * ESP_OK and leaves the response untouched (caller proceeds). On failure
 * sends a 401 (or 503 if the credential isn't configured) and returns
 * ESP_FAIL — caller MUST propagate ESP_FAIL without sending its own
 * response. */
static esp_err_t ota_check_basic_auth(httpd_req_t *req)
{
    /* Fail closed if the credential macro is still the sentinel. We don't
     * want a half-configured build to silently allow anyone in. */
    if (ota_ct_streq(OTA_BASIC_AUTH_B64, "REPLACE_ME"))
    {
        ESP_LOGE(OTA_TAG, "OTA credential not configured "
                          "(see ota_credentials.h) — rejecting upload");
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req,
                           "OTA is disabled on this device: no credential is configured "
                           "in ota_credentials.h. Set OTA_BASIC_AUTH_B64, rebuild, and "
                           "flash once over UART to enable network OTA.\n");
        return ESP_FAIL;
    }

    size_t hdr_len = httpd_req_get_hdr_value_len(req, "Authorization");

    /* No header, or absurdly long header (defend against a 64 KB
     * Authorization line burning httpd stack/heap). 256 chars comfortably
     * covers any sane Base64-of-user:password. */
    if (hdr_len == 0 || hdr_len > 256)
    {
        goto challenge;
    }

    char auth[260] = {0};
    if (httpd_req_get_hdr_value_str(req, "Authorization",
                                    auth, sizeof(auth)) != ESP_OK)
    {
        goto challenge;
    }

    /* RFC 7617: scheme is literally "Basic", case-insensitive. Browsers and
     * curl both emit exactly "Basic ". Accept the canonical form only. */
    static const char prefix[] = "Basic ";
    const size_t plen = sizeof(prefix) - 1;
    if (strncmp(auth, prefix, plen) != 0)
    {
        goto challenge;
    }

    if (!ota_ct_streq(auth + plen, OTA_BASIC_AUTH_B64))
    {
        goto challenge;
    }

    return ESP_OK;

challenge:
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate",
                       "Basic realm=\"PTP Slave OTA\"");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Authentication required.\n");
    return ESP_FAIL;
}

/* ----------------------------------------------------------------------------
 *  POST /api/ota
 *
 *  The browser uploads the .bin via XHR/fetch as a raw octet-stream body.
 *  We stream the body straight into the inactive OTA partition without ever
 *  buffering the full image (the P4 only has 768 KB of RAM but firmware can
 *  be up to 3 MB).
 * --------------------------------------------------------------------------- */
static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    /* ─── Gate 1: authentication ──────────────────────────────────────────
     * Verify HTTP Basic credentials before touching anything else. On
     * failure the helper has already written the 401/503 response. Doing
     * auth first means an unauthenticated attacker cannot probe partition
     * state or busy state via response timing or error codes. */
    if (ota_check_basic_auth(req) != ESP_OK) {
        return ESP_FAIL;
    }

    /* ─── Gate 2: Content-Type enforcement (residual-CSRF mitigation) ─────
     * application/octet-stream is NOT a CORS-safelisted Content-Type, so a
     * cross-origin fetch with this Content-Type requires a preflight OPTIONS
     * request. We don't register OPTIONS, so the preflight fails and the
     * actual POST is never issued by the browser.
     *
     * This closes the residual CSRF window left open by Basic Auth alone:
     * an attacker page can't use credentials:'include' to silently replay a
     * cached Basic Auth credential if the browser refuses to send the
     * request in the first place.
     *
     * curl uploads (explicit user gesture, not a browser) and the on-device
     * /ota page (same-origin, no CORS check) both send this Content-Type
     * naturally, so legitimate flows are unaffected. */
    char ct[64] = {0};
    if (httpd_req_get_hdr_value_str(req, "Content-Type",
                                    ct, sizeof(ct)) != ESP_OK ||
        strncasecmp(ct, "application/octet-stream", 24) != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Content-Type must be application/octet-stream");
        return ESP_FAIL;
    }

    /* ─── Gate 3: body-length sanity ──────────────────────────────────────
     * Reject empty or unsized requests before doing any allocation. */
    esp_err_t err;
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update = NULL;
    char *buf = NULL;
    int received_total = 0;
    int content_len = req->content_len;
    esp_err_t resp_err = ESP_OK;

    if (content_len <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Missing Content-Length or empty body");
        return ESP_FAIL;
    }

    /* ─── Gate 4: atomic claim of the OTA slot ────────────────────────────
     * Only one OTA upload may proceed at a time. atomic_compare_exchange
     * publishes the claim and verifies it was uncontested in one indivisible
     * step — closes the TOCTOU window the previous "if (busy) ...; busy=true"
     * pattern had, in which two authenticated uploads could both pass the
     * gate and corrupt each other's session at esp_ota_begin/write.
     *
     * From here on, every failure path MUST atomic_store(false) before
     * returning, or the OTA subsystem stays wedged "busy" until reboot.
     * The success path intentionally leaves it true so the ~1 s pre-reboot
     * window correctly surfaces "busy" to any concurrent uploader. */
    bool expected = false;
    if (!atomic_compare_exchange_strong(&g_ota_in_progress, &expected, true)) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_send(req,
                        "OTA already in progress",
                        HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    update = esp_ota_get_next_update_partition(NULL);
    if (!update)
    {
        atomic_store(&g_ota_in_progress, false);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No OTA partition available. Re-flash with "
                            "partitions_ota.csv (see ota_handler.h).");
        return ESP_FAIL;
    }

    if ((size_t)content_len > update->size)
    {
        atomic_store(&g_ota_in_progress, false);
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Image too large: %d bytes > partition size %lu bytes",
                 content_len, (unsigned long)update->size);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_FAIL;
    }

    buf = (char *)malloc(OTA_RECV_BUF_SIZE);
    if (!buf)
    {
        atomic_store(&g_ota_in_progress, false);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Out of memory for OTA buffer");
        return ESP_FAIL;
    }

    ESP_LOGI(OTA_TAG, "OTA upload starting: %d bytes into partition '%s' @ 0x%lx",
             content_len, update->label, (unsigned long)update->address);

    err = esp_ota_begin(update, content_len, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(OTA_TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        atomic_store(&g_ota_in_progress, false);
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "esp_ota_begin failed");
        return ESP_FAIL;
    }

    /* Stream loop: keep reading from the socket and writing to flash.
     * httpd_req_recv returns 0 on EOF, <0 on error, >0 = bytes received. */
    while (received_total < content_len)
    {
        int want = content_len - received_total;
        if (want > OTA_RECV_BUF_SIZE) want = OTA_RECV_BUF_SIZE;

        int got = httpd_req_recv(req, buf, want);
        if (got <= 0)
        {
            if (got == HTTPD_SOCK_ERR_TIMEOUT)
            {
                /* keep going, browser is slow */
                continue;
            }
            ESP_LOGE(OTA_TAG, "Socket read failed at %d/%d bytes (rc=%d)",
                     received_total, content_len, got);
            resp_err = ESP_FAIL;
            break;
        }

        err = esp_ota_write(ota_handle, buf, got);
        if (err != ESP_OK)
        {
            ESP_LOGE(OTA_TAG, "esp_ota_write failed at offset %d: %s",
                     received_total, esp_err_to_name(err));
            resp_err = err;
            break;
        }
        received_total += got;

        /* Progress log every ~64 KB */
        if ((received_total & 0xFFFF) < OTA_RECV_BUF_SIZE)
        {
            ESP_LOGI(OTA_TAG, "OTA progress: %d / %d bytes (%d%%)",
                     received_total, content_len,
                     (int)(100LL * received_total / content_len));
        }
    }

    free(buf);

    if (resp_err != ESP_OK)
    {
        esp_ota_abort(ota_handle);
        atomic_store(&g_ota_in_progress, false);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "OTA write failed mid-stream");
        return ESP_FAIL;
    }

    if (received_total != content_len)
    {
        esp_ota_abort(ota_handle);
        atomic_store(&g_ota_in_progress, false);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Body shorter than Content-Length");
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        atomic_store(&g_ota_in_progress, false);
        ESP_LOGE(OTA_TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        /* The most common cause here is a bad image (wrong chip, wrong
         * secure-boot signature, truncated). */
        const char *why = (err == ESP_ERR_OTA_VALIDATE_FAILED)
                            ? "Image validation failed (wrong chip target?)"
                            : "esp_ota_end failed";
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, why);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK)
    {
        atomic_store(&g_ota_in_progress, false);
        ESP_LOGE(OTA_TAG, "esp_ota_set_boot_partition failed: %s",
                 esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Could not set boot partition");
        return ESP_FAIL;
    }

    ESP_LOGI(OTA_TAG, "OTA success! New boot partition: %s. Rebooting.",
             update->label);

    /* Tell the browser, then schedule reboot from a separate task so the
     * HTTP response gets flushed before reset. */
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK -- rebooting into new firmware");

    /* Intentionally leave g_ota_in_progress = true so concurrent uploaders
     * see "busy" during the ~1 s pre-reboot window. */
    xTaskCreate(ota_reboot_task, "ota_reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 *  GET /api/ota/info  --  JSON describing partitions + current OTA state
 * --------------------------------------------------------------------------- */
static esp_err_t ota_info_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next_upd = esp_ota_get_next_update_partition(NULL);
    const esp_app_desc_t *app = esp_app_get_description();

    char buf[512];
    int off = 0;

    off += snprintf(buf + off, sizeof(buf) - off, "{");

    if (running)
    {
        off += snprintf(buf + off, sizeof(buf) - off,
                        "\"running\":{\"label\":\"%s\",\"address\":\"0x%lx\",\"size\":%lu,\"subtype\":%d}",
                        running->label,
                        (unsigned long)running->address,
                        (unsigned long)running->size,
                        (int)running->subtype);
    }
    else
    {
        off += snprintf(buf + off, sizeof(buf) - off, "\"running\":null");
    }

    if (next_upd)
    {
        off += snprintf(buf + off, sizeof(buf) - off,
                        ",\"next_update\":{\"label\":\"%s\",\"address\":\"0x%lx\",\"size\":%lu,\"subtype\":%d}",
                        next_upd->label,
                        (unsigned long)next_upd->address,
                        (unsigned long)next_upd->size,
                        (int)next_upd->subtype);
    }
    else
    {
        off += snprintf(buf + off, sizeof(buf) - off, ",\"next_update\":null");
    }

    off += snprintf(buf + off, sizeof(buf) - off,
                    ",\"app_version\":\"%s\",\"project\":\"%s\",\"compile_time\":\"%s %s\"",
                    app ? app->version : "?",
                    app ? app->project_name : "?",
                    app ? app->date : "?",
                    app ? app->time : "?");

    off += snprintf(buf + off, sizeof(buf) - off,
                    ",\"ota_in_progress\":%s,\"ota_supported\":%s}",
                    atomic_load(&g_ota_in_progress) ? "true" : "false",
                    next_upd ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, buf, off);
}

#endif /* OTA_HANDLER_H */
