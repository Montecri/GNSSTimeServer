#!/usr/bin/env python3
"""
Convert HTML pages into C string-literal headers for the PTP MASTER firmware.

   ota.html   -> ota_page.h    (OTA_PAGE_HTML)
   index.html -> status_page.h (STATUS_PAGE_HTML)

Usage:
   build_ota_page.py ota.html ota_page.h
   build_ota_page.py index.html status_page.h
   build_ota_page.py --both

The status_page.h emitter uses byte-exact opening and closing blocks
embedded as triple-quoted strings below (STATUS_OPENING / STATUS_CLOSING).
This preserves the master-specific C-preprocessor tricks that would
otherwise be lost on regeneration:

  * The HEALTH_POLL_INTERVAL_MS default #define and the SP_STR / SP_STR2
    stringize macros, injected between the include guard and the extern
    "C" block, so `idf.py build -DHEALTH_POLL_INTERVAL_MS=500` can retune
    the poll interval without editing generated code.
  * One line inside the JS string literal that "tears" the C string:
    "const HEALTH_POLL_MS = " SP_STR(HEALTH_POLL_INTERVAL_MS) ";\\n"
    -- three C string tokens concatenated by the compiler, with the
    numeric macro value slotted in the middle. See CTOKEN_LINE_RULES.
  * `#undef SP_STR` / `#undef SP_STR2` after the array so downstream
    files that also include ours don't inherit the macros.
  * The JSON-contract comment that documents the /api/status shape.

The OTA path uses the master's fancier "GPS satellite + arrow" favicon SVG.
"""

import os
import sys

# --- OTA favicon (fancy master design) ---
OTA_FAVICON = (
    '<link rel="icon" type="image/svg+xml" '
    'href="data:image/svg+xml,%3Csvg%20xmlns=\'http://www.w3.org/2000/svg\'%20viewBox=\'0%200%2064%2064\'%3E'
    '%3Cdefs%3E%3ClinearGradient%20id=\'g\'%20x1=\'0\'%20y1=\'0\'%20x2=\'1\'%20y2=\'1\'%3E'
    '%3Cstop%20offset=\'0\'%20stop-color=\'%23ffb86c\'/%3E'
    '%3Cstop%20offset=\'1\'%20stop-color=\'%23ff5577\'/%3E'
    '%3C/linearGradient%3E%3C/defs%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'32\'%20r=\'28\'%20fill=\'%230b1018\'%20stroke=\'url(%23g)\'%20stroke-width=\'4\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'11\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'53\'%20cy=\'32\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'53\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Ccircle%20cx=\'11\'%20cy=\'32\'%20r=\'2\'%20fill=\'%23ffb86c\'%20opacity=\'0.6\'/%3E'
    '%3Cline%20x1=\'32\'%20y1=\'32\'%20x2=\'32\'%20y2=\'18\'%20stroke=\'%23e6edf7\'%20stroke-width=\'3\'%20stroke-linecap=\'round\'%20opacity=\'0.3\'/%3E'
    '%3Cline%20x1=\'32\'%20y1=\'32\'%20x2=\'42\'%20y2=\'32\'%20stroke=\'%237aa2f7\'%20stroke-width=\'3\'%20stroke-linecap=\'round\'%20opacity=\'0.3\'/%3E'
    '%3Ccircle%20cx=\'32\'%20cy=\'32\'%20r=\'3\'%20fill=\'%23e6edf7\'%20opacity=\'0.3\'/%3E'
    '%3Cpath%20d=\'M%2032%2050%20L%2032%2022%20M%2022%2032%20L%2032%2022%20L%2042%2032\'%20stroke=\'%23ffb86c\'%20stroke-width=\'5\'%20stroke-linecap=\'round\'%20stroke-linejoin=\'round\'%20fill=\'none\'/%3E'
    '%3C/svg%3E">'
)

# --- CTOKEN torn-string rule (status_page.h only) ---
CTOKEN_LINE_RULES = {
    'const HEALTH_POLL_MS = " SP_STR(HEALTH_POLL_INTERVAL_MS) ";':
        '    "const HEALTH_POLL_MS = " SP_STR(HEALTH_POLL_INTERVAL_MS) ";\\n"\n',
}

# --- Byte-exact opening for status_page.h ---
# Everything from the top of the file down to and including the array-
# declaration line ("static const char STATUS_PAGE_HTML[] ="). Includes
# the header comment with the integration example, the include guard, the
# HEALTH_POLL_INTERVAL_MS default + SP_STR macros, and the extern "C" block.
STATUS_OPENING = r"""/* ============================================================================
 *  status_page.h  --  Embedded HTML status page for ESP32-P4 PTP Grandmaster
 *
 *  This header embeds, as a single C raw-string literal, a self-contained
 *  HTML / CSS / JavaScript page that renders runtime diagnostics for the
 *  PTPv2 master running on this device. The page polls "/api/status" every
 *  5 seconds (XHR/fetch) and updates every field in place.
 *
 *  Integration (add to main.c):
 *
 *      #include "status_page.h"
 *      #include "esp_http_server.h"
 *      ...
 *      static esp_err_t root_handler(httpd_req_t *req) {
 *          httpd_resp_set_type(req, "text/html; charset=utf-8");
 *          return httpd_resp_send(req, STATUS_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
 *      }
 *      static esp_err_t status_handler(httpd_req_t *req) {
 *          // Build JSON from your globals and send it. See the JSON contract
 *          // documented at the bottom of this file.
 *      }
 *      static void start_webserver(void) {
 *          httpd_handle_t s = NULL;
 *          httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
 *          if (httpd_start(&s, &cfg) == ESP_OK) {
 *              httpd_uri_t root   = { .uri="/",            .method=HTTP_GET, .handler=root_handler   };
 *              httpd_uri_t status = { .uri="/api/status",  .method=HTTP_GET, .handler=status_handler };
 *              httpd_register_uri_handler(s, &root);
 *              httpd_register_uri_handler(s, &status);
 *          }
 *      }
 *
 *  Project: https://github.com/Montecri/GNSSTimeServer
 *  Target : ESP32-P4-Function-EV-Board, ESP-IDF v5.5.4
 * ============================================================================ */

#ifndef STATUS_PAGE_H
#define STATUS_PAGE_H

/* ── On-page "Load / Soak" monitor poll interval (milliseconds) ──
 * The health poller is OFF by default and only runs after the user clicks
 * "Start monitoring" on the page, so the page never self-loads the device
 * unless opted in. This value sets the cadence used once started. Override at
 * build time, e.g. idf.py build -DHEALTH_POLL_INTERVAL_MS=500, or change the
 * default here. */
#ifndef HEALTH_POLL_INTERVAL_MS
#define HEALTH_POLL_INTERVAL_MS 1000
#endif
/* Stringize so the numeric #define can be concatenated into the embedded JS. */
#define SP_STR2(x) #x
#define SP_STR(x) SP_STR2(x)

#ifdef __cplusplus
extern "C" {
#endif

/* C99 raw-string-equivalent: we use a long C-string built from the source
 * HTML at generate time. Because C has no built-in raw-string literal, the
 * HTML below has been pre-processed: backslashes and double-quotes inside
 * the page are escaped. The page itself is otherwise unmodified. */
static const char STATUS_PAGE_HTML[] =
"""

# --- Byte-exact closing for status_page.h ---
# Everything from `#undef SP_STR` to the include guard's closing #endif,
# including the JSON-contract comment that documents the /api/status shape.
STATUS_CLOSING = r"""
#undef SP_STR
#undef SP_STR2

#ifdef __cplusplus
}
#endif

/* ----------------------------------------------------------------------------
 * JSON CONTRACT for GET /api/status
 *
 * The page expects this JSON shape. Any field may be omitted; the UI shows
 * an em-dash for missing values. All numbers are plain JSON numbers (not
 * strings). No key appears both at top level and inside "config"/"bmca" --
 * that would be duplicated information (see the BMCA/JSON-dashboard
 * handover, section 1: "no duplicated information inside any one endpoint").
 *
 * {
 *   "chip":              "ESP32-P4",
 *   "chip_rev":          "v1.0",
 *   "cpu_mhz":           360,           // esp_rom_get_cpu_ticks_per_us(), boot-cached
 *   "project":           "PTP_Master",
 *   "app_version":       "819246f-dirty",
 *   "idf_version":       "v5.5.4",
 *   "compile_time":      "May 12 2026 16:24",
 *   "elf_sha":           "d25969555aabbccdd...",
 *   "flash_mb":          16,            // esp_flash_get_size(), boot-cached
 *   "flash_mode":        "dio",         // CONFIG_ESPTOOLPY_FLASHMODE
 *   "flash_mhz":         80,            // CONFIG_ESPTOOLPY_FLASHFREQ_VAL
 *   "uptime_s":          10053,
 *
 *   "heap_free":         428032,   // bytes -- heap_caps_get_free_size(MALLOC_CAP_INTERNAL)
 *   "heap_min":          398336,   // bytes -- heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)
 *   "heap_lfb":          262144,   // bytes -- heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)
 *   "heap_leak":         0,        // bytes delta vs. boot+10s baseline, or null before that baseline exists
 *   "heap_internal_total":507904,  // heap_caps_get_total_size(MALLOC_CAP_INTERNAL), boot-cached
 *   "retent_total":      172032,   // bytes with MALLOC_CAP_RETENTION
 *   "retent_free":       145120,   // currently free bytes in that capability heap
 *   "rtc_heap_total":    null,     // MALLOC_CAP_RTCRAM heap total, or null if not heap-managed here
 *   "tcm_heap_total":    null,     // MALLOC_CAP_TCM heap total, or null if not heap-managed here
 *
 *   "link_up":           true,
 *   "l2tap_ready":       true,
 *   "ip":                "192.168.5.142",
 *   "ip_source":         "DHCP",        // "DHCP" | "Static" | null (no IP yet)
 *   "mac":               "30:ED:A0:E2:33:D3",
 *   "hostname":          "radiant-captain-james-smith",
 *   "emac_restarts":     0,        // g_emac_restart_count -- the watchdog's OWN recovery count, not duplicated in "wdt"
 *   "promiscuous":       false,
 *   "wdt": {
 *     "state":              "ok",  // "arming" | "ok" | "phc_frozen" | "heap_critical" | "restart_limit"
 *     "alerts":             0,     // count of 30s passes that found a problem
 *     "last_alert_uptime_s":null  // uptime seconds of the last alert, or null if never
 *   },
 *
 *   "role":              "MASTER",      // or "PASSIVE" (never "SLAVE" -- this is a GM-only firmware)
 *   "clock_id":          "30ED A0FF FEE2 33D3",
 *   "gnss_utc_s":        1778901234,    // epoch seconds from GNSS, or null when GNSS time isn't valid
 *   "tai_offset":        37,            // g_utc_offset
 *   "sync_seq":          10031,         // g_sync_seq
 *   "announce_seq":      5015,          // g_announce_seq
 *   "slaves_active":     3,             // slave_active_count()
 *
 *   "servo_state":       "LOCKED",      // NO_LINK | ACQUIRING | LOCKED | HOLDOVER | FAULT
 *   "locked_for_s":      2537,          // 0 when not LOCKED
 *   "offset_ns":         128,           // g_last_offset (averaged once LOCKED)
 *   "freq_ppb":          -12.4,         // g_freq_ppb
 *   "integrator":        -3.21,         // g_integral
 *   "lock_qual":         61,            // good samples (within lock_qual_good_ns) among the last
 *                                       // config.servo.lock_qual_window -- same as the OLED
 *   "pps_count":         2548,          // g_pps_capture_seq
 *   "pps_coast_extra":   0,             // g_pps_coast_extra -- extra (glitch) PPS edges seen
 *                                       // (lifetime, since boot)
 *   "pps_coast_missing": 0,             // g_pps_coast_missing -- missing PPS edges, in whole seconds
 *                                       // (lifetime, since boot)
 *   "reject_total":      0,             // g_total_rejects (lifetime, since boot)
 *   "reject_streak":     0,             // g_consecutive_rejects (live run)
 *   "phc_residual_ns":   42,            // g_phc_offset_ns -- RAW per-edge offset (pre-average); differs
 *                                       // from offset_ns while LOCKED, where offset_ns is the filtered mean
 *   "sec_agree":         true,          // g_seconds_agree -- receiver and PHC agree on the second
 *   "sec_diff":          0,             // g_seconds_diff -- receiver second minus PHC second (0 = agree)
 *   "sec_disagree_n":    0,             // g_seconds_disagree_n -- consecutive checks with that difference
 *   "settled":           true,          // g_settled_after_holdover
 *   "gnss_age_s":         2,
 *   "gnss_fix":           true,
 *
 *   "bmca": {
 *     "local":   {"priority1":128,"clock_class":6,"accuracy":33,"variance":65535,"priority2":128},
 *     "peer":    {"clock_id":"30ED A0FF FEA1 075C","priority1":128,"clock_class":6,
 *                 "accuracy":33,"variance":65535,"priority2":128,"steps_removed":0},
 *     "peer_better": true,
 *     "decided_by": "clock_identity",   // steps_removed|priority1|clock_class|accuracy|variance|priority2|clock_identity|null
 *     "candidate": {"clock_id":"30ED A0FF FEA1 075C","count":3},
 *     "last_announce_rx_age_ms": 1400,
 *     "role_changes": 2,
 *     "events": [
 *       {"t_ms":2467000,"to":"PASSIVE","reason":"better_announce",
 *        "peer":"30ED A0FF FEA1 075C","decided_by":"clock_identity"}
 *     ]
 *   },
 *
 *   "config": {
 *     "ptp":   {"domain":0,"transport":"L2","delay_mechanism":"E2E","two_step":true,
 *               "sync_interval_ms":1000,"announce_interval_ms":2000,
 *               "announce_timeout_s":6,"bmca_stability_threshold":3},
 *     "servo": {"acq_kp":0.4,"acq_ki":0.3,
 *               "lock_kp":0.05,"lock_ki":0.001,"lock_kd":2.0,"lock_kd_enabled":false,
 *               "avg_window":32,"lock_qual_window":64,"lock_qual_good_ns":2000,
 *               "reject_reacquire_at":45,"holdover_timeout_s":5,"post_holdover_settle_ms":30000},
 *     "hw":    {"pps_gpio":20,"pps_led_gpio":3,"pps_led_on_ms":50,"button_gpio":1,
 *               "gnss_uart":1,"gnss_baud":115200,"display_screens":5},
 *     "net":   {"dhcp_fallback_timeout_ms":30000,"slaves_max":8,"slave_timeout_ms":30000},
 *     "mon":   {"heap_mon_interval_ms":60000}
 *   },
 *
 *   "tasks": [
 *     { "name": "ptp_rx",   "priority": 7, "stack_bytes": 8192, "hwm_bytes": 4734, "core": 0 },
 *     { "name": "ptp_tx",   "priority": 5, "stack_bytes": 8192, "hwm_bytes": 5298, "core": 0 },
 *     { "name": "bmca_log", "priority": 1, "stack_bytes": 2048, "hwm_bytes": 1500, "core": 0 },
 *     { "name": "gnss",      "priority": 5, "stack_bytes": 6144, "hwm_bytes": 2752, "core": 1 },
 *     { "name": "gnss_sig", "priority": 1, "stack_bytes": 3072, "hwm_bytes": 2400, "core": 0 },
 *     { "name": "servo",    "priority": 6, "stack_bytes": 6144, "hwm_bytes": 3211, "core": 1 },
 *     { "name": "pps_task", "priority": 7, "stack_bytes": 4096, "hwm_bytes": 2868, "core": 1 },
 *     { "name": "pps_led",  "priority": 1, "stack_bytes": 2048, "hwm_bytes": 1600, "core": 0 },
 *     { "name": "led",      "priority": 1, "stack_bytes": 2048, "hwm_bytes": 1644, "core": 0 },
 *     { "name": "display",  "priority": 1, "stack_bytes": 4096, "hwm_bytes": 2560, "core": 0 },
 *     { "name": "btn",      "priority": 1, "stack_bytes": 2048, "hwm_bytes": 1680, "core": 0 },
 *     { "name": "dhcp_fb",  "priority": 2, "stack_bytes": 3072, "hwm_bytes": 2304, "core": 0 },
 *     { "name": "heap_mon", "priority": 1, "stack_bytes": 3072, "hwm_bytes": 2334, "core": 0 },
 *     { "name": "wdt",      "priority": 1, "stack_bytes": 4096, "hwm_bytes": 2950, "core": 0 },
 *     { "name": "httpd",    "priority": 5, "stack_bytes": 8192, "hwm_bytes": 3800, "core": 0 },
 *     { "name": "main",     "priority": 1, "stack_bytes": 3584, "hwm_bytes": 2900, "core": 0 }
 *   ]
 * }
 *
 * Removed since the previous contract (do not reintroduce): ram1_kb, ram2_kb,
 * rtc_kb, tcm_kb (invented literals -- replaced by heap_internal_total/
 * rtc_heap_total/tcm_heap_total), utc_time (replaced by gnss_utc_s), locked_for
 * (replaced by locked_for_s), path_delay_ns (this is a GM-only firmware --
 * always null, so removed rather than reported), wdt_healthy (replaced by the
 * "wdt" object), pps_led_active (was a duplicate of servo_state=="LOCKED"),
 * sync_interval_ms/announce_interval_ms/lock_qual_window/reject_reacquire_at/
 * holdover_timeout_s/slaves_max (moved into "config", which is where every
 * compile-time constant lives now).
 *
 * GET /api/bmca returns { "uptime_ms":N, "role":"MASTER|PASSIVE",
 * "clock_id":"...", "bmca":{...same shape as above...} } -- for the
 * dashboard's 1 Hz BMCA-card poll, without the task-table walk /api/status
 * does. POST /api/bmca/priority?p1=N&p2=M (Basic Auth, same credentials as
 * OTA) sets the runtime priority1/priority2 used by the next Announce sent
 * and the next one evaluated; RAM-only, reset to 128/128 on reboot.
 *
 * Use uxTaskGetSystemState() to populate the "tasks" array. hwm_bytes is
 * the FreeRTOS high-water-mark (uxTaskGetStackHighWaterMark) multiplied by
 * sizeof(StackType_t) -- i.e. the smallest amount of free stack ever seen.
 * stack_bytes comes from the task_create_tracked() registry and is null for
 * any task not registered there.
 * --------------------------------------------------------------------------- */

#endif /* STATUS_PAGE_H */
"""


HEADER_TEMPLATES = {
    "ota_page.h": {
        "constant": "OTA_PAGE_HTML",
        "guard": "OTA_PAGE_H",
        "inject_favicon": True,
        "terminator_style": "own_line",
        "comment": (
            "/* ============================================================================\n"
            " *  ota_page.h  --  Embedded HTML for the OTA firmware update page\n"
            " *\n"
            " *  Served at GET /ota by the web server. The page POSTs the firmware .bin\n"
            " *  to /api/ota and polls /api/ota/info for partition status. See\n"
            " *  ota_handler.h for the C-side endpoints.\n"
            " *\n"
            " *  Project: https://github.com/Montecri/GNSSTimeServer\n"
            " *\n"
            " *  NOTE: Auto-generated from ota.html. Do not edit by hand -- re-run\n"
            " *        build_ota_page.py whenever ota.html changes, then rebuild & flash.\n"
            " * ============================================================================ */\n"
        ),
    },
    "status_page.h": {
        "constant": "STATUS_PAGE_HTML",
        "guard": "STATUS_PAGE_H",
        "inject_favicon": False,
        "terminator_style": "inline",
        "comment": None,
    },
}


def stringify_line(line):
    line = line.rstrip("\r\n")
    line = line.replace("\\", "\\\\").replace('"', '\\"')
    return '    "' + line + '\\n"\n'


def stringify_line_terminal(line):
    line = line.rstrip("\r\n")
    line = line.replace("\\", "\\\\").replace('"', '\\"')
    return '    "' + line + '\\n";\n'


def detect_template(header_path):
    base = os.path.basename(header_path)
    for key, tmpl in HEADER_TEMPLATES.items():
        if base.endswith(key):
            return key, tmpl
    raise SystemExit("error: don't know how to build '" + base +
                     "'. Expected filename ending in one of: " + str(list(HEADER_TEMPLATES)))


def convert(html_path, header_path):
    key, tmpl = detect_template(header_path)

    with open(html_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    out = []
    use_exact = (key == "status_page.h" and STATUS_OPENING is not None and STATUS_CLOSING is not None)

    if use_exact:
        out.append(STATUS_OPENING)
    else:
        out.append(tmpl["comment"])
        out.append("\n")
        out.append("#ifndef " + tmpl["guard"] + "\n")
        out.append("#define " + tmpl["guard"] + "\n")
        out.append("\n")
        out.append("#ifdef __cplusplus\n")
        out.append('extern "C" {\n')
        out.append("#endif\n")
        out.append("\n")
        out.append("static const char " + tmpl["constant"] + "[] =\n")

    inject = tmpl["inject_favicon"]
    if inject and any('rel="icon"' in ln or "rel='icon'" in ln for ln in lines):
        inject = False

    favicon_inserted = False
    is_inline_term = (tmpl["terminator_style"] == "inline")

    if is_inline_term:
        last_content_idx = len(lines) - 1
        while last_content_idx >= 0 and lines[last_content_idx].strip() == "":
            last_content_idx -= 1
    else:
        last_content_idx = -1

    for i, line in enumerate(lines):
        stripped = line.rstrip("\r\n")
        if key == "status_page.h" and stripped in CTOKEN_LINE_RULES:
            out.append(CTOKEN_LINE_RULES[stripped])
        elif is_inline_term and i == last_content_idx:
            out.append(stringify_line_terminal(line))
        else:
            out.append(stringify_line(line))
        if inject and not favicon_inserted and 'name="viewport"' in line:
            out.append(stringify_line(OTA_FAVICON))
            favicon_inserted = True

    if not is_inline_term:
        out.append("    ;\n")

    if use_exact:
        out.append(STATUS_CLOSING)
    else:
        out.append("\n")
        out.append("#ifdef __cplusplus\n")
        out.append("}\n")
        out.append("#endif\n")
        out.append("\n")
        out.append("#endif /* " + tmpl["guard"] + " */\n")

    with open(header_path, "w", encoding="utf-8") as f:
        f.writelines(out)

    print("Wrote " + header_path + "  (" + key + " pattern, " + str(len(lines)) + " HTML lines)")


def main(argv):
    if len(argv) == 2 and argv[1] == "--both":
        convert("ota.html", "ota_page.h")
        convert("index.html", "status_page.h")
        return
    if len(argv) != 3:
        print("usage: " + argv[0] + " <input.html> <output.h>", file=sys.stderr)
        print("   or: " + argv[0] + " --both", file=sys.stderr)
        sys.exit(1)
    convert(argv[1], argv[2])


if __name__ == "__main__":
    main(sys.argv)
