/* ============================================================================
 *  web_server.h  --  Drop-in HTTP server for ESP32-P4 PTP Grandmaster
 *
 *  Serves two endpoints:
 *
 *    GET /            -> the status_page.h HTML (served from flash/RAM)
 *    GET /api/status  -> JSON snapshot of every runtime value the page needs
 *
 *  HOW TO USE (one-line integration in main.c):
 *
 *      #include "status_page.h"   // already created
 *      #include "web_server.h"    // this file
 *      ...
 *      // Inside app_main(), AFTER ethernet_init() and AFTER xTaskCreate(...)
 *      // calls, anywhere is fine -- the server self-gates on g_link_up:
 *      web_server_start();
 *
 *  No other changes are required to main.c. The header is a single-include
 *  unit: it pulls in all ESP-IDF and FreeRTOS headers it needs and contains
 *  both the function definitions and the start call. It is designed to be
 *  included EXACTLY ONCE, from main.c, AFTER all the static globals it reads
 *  (g_last_offset, g_freq_ppb, g_integral, g_emac_restart_count, g_sync_seq,
 *  g_announce_seq, g_utc_offset, g_gnss_valid, g_gnss_seconds, g_link_up,
 *  g_l2tap_ready, s_led_state, g_phc_offset_ns, g_lock_qual_ring,
 *  g_lock_acquired_ms, g_settled_after_holdover, g_boot_time_ms, g_eth_netif,
 *  g_src_mac, g_clock_id, g_hostname) and after the function
 *  slave_active_count() is
 *  defined. The simplest place is right above app_main(), at the bottom of
 *  main.c.
 *
 *  Required component dependency: add "esp_http_server" and "app_update" to
 *  your component's CMakeLists.txt REQUIRES list (or your main component's
 *  idf_component_register REQUIRES). Example:
 *
 *      idf_component_register(SRCS "main.c"
 *                             INCLUDE_DIRS "."
 *                             REQUIRES esp_http_server app_update esp_netif
 *                                      esp_eth esp_timer esp_system esp_event
 *                                      freertos driver)
 *
 *  Memory cost: ~6 KB stack for the httpd task plus ~32 KB rodata for the
 *  embedded HTML. Total RAM impact is negligible.
 *
 *  Project: https://github.com/Montecri/GNSSTimeServer
 *  Target : ESP32-P4-Function-EV-Board, ESP-IDF v5.5.4
 * ============================================================================ */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_mac.h"

/* CPU frequency: read from menuconfig (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ).
 * The boot log shows 360 MHz on this P4 build; we surface the configured
 * value rather than calling the private esp_clk_cpu_freq() API. */
#ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
  #define WEB_CPU_MHZ  CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#else
  #define WEB_CPU_MHZ  360
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "status_page.h"   /* defines STATUS_PAGE_HTML */
#include "ota_page.h"      /* defines OTA_PAGE_HTML */

#include <stdarg.h>

/* ── Bounded JSON append ──
 * snprintf returns the WOULD-HAVE-written length when it truncates; the
 * naive `off += snprintf(buf + off, CAP - off, ...)` pattern therefore
 * lets `off` sail past CAP on the first truncation, after which the next
 * call's `CAP - off` underflows (size_t) and writes out of bounds. The
 * buffers here are generously sized, so that was latent — this helper
 * makes it impossible: `off` saturates at CAP-1 (the NUL) and further
 * appends become no-ops, yielding truncated-but-safe JSON. Defined before
 * ota_handler.h so ota_info_handler can use it too (single-TU include
 * chain, see the file header). */
__attribute__((format(printf, 4, 5)))
static size_t web_append(char *buf, size_t cap, size_t off, const char *fmt, ...)
{
    if (off >= cap)
        return off;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + off, cap - off, fmt, ap);
    va_end(ap);
    if (n < 0)
        return off;
    off += (size_t)n;
    if (off >= cap)
        off = cap - 1;
    return off;
}

#include "ota_handler.h"   /* defines ota_upload_handler, ota_info_handler */

/* The static globals below are defined in main.c. They are referenced
 * directly because this header is included from main.c (single TU). */

static const char *WEB_TAG = "WEB";

/* ----------------------------------------------------------------------------
 *  GET /  --  serves the embedded HTML page
 * --------------------------------------------------------------------------- */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, STATUS_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ----------------------------------------------------------------------------
 *  Small helpers used by status_handler
 * --------------------------------------------------------------------------- */

/* Translate the led_state_t enum (defined in main.c) into the strings the
 * front-end expects. */
static const char *web_state_str(led_state_t s)
{
    switch (s)
    {
    case LED_STATE_NO_LINK:   return "NO_LINK";
    case LED_STATE_ACQUIRING: return "ACQUIRING";
    case LED_STATE_LOCKED:    return "LOCKED";
    case LED_STATE_HOLDOVER:  return "HOLDOVER";
    case LED_STATE_FAULT:     return "FAULT";
    default:                  return "UNKNOWN";
    }
}

/* Lookup table to expose a logical CPU core per task. ESP-IDF exposes
 * xTaskGetCoreID() on multi-core targets; on single-core (or when the
 * task has no affinity) we report -1. */
static int web_task_core(TaskHandle_t h)
{
#if (CONFIG_FREERTOS_NUMBER_OF_CORES > 1) || (defined(configNUM_CORES) && (configNUM_CORES > 1))
    BaseType_t c = xTaskGetCoreID(h);
    if (c == tskNO_AFFINITY) return -1;
    return (int)c;
#else
    (void)h;
    return 0;
#endif
}

/* ----------------------------------------------------------------------------
 *  web_append_bmca  --  shared "bmca":{...} JSON fragment
 *
 *  Used by both /api/status and GET /api/bmca so the two endpoints can never
 *  disagree about BMCA state. Never calls build_local_dataset() or
 *  advertised_clock_class() (side effect on g_settled_after_holdover) and
 *  never writes into *v (single-writer seqlock — httpd only reads).
 * --------------------------------------------------------------------------- */
static size_t web_append_bmca(char *buf, size_t cap, size_t off,
                              bool i_am_master, int64_t uptime_ms,
                              const bmca_view_t *v)
{
    off = web_append(buf, cap, off, "\"bmca\":{");

    /* local: MASTER reads the packed dataset last placed on the wire;
     * PASSIVE reads the seqlock snapshot (refreshed on every accepted
     * Announce via bmca_update -> build_local_dataset in ptp_rx_task). */
    if (i_am_master)
    {
        uint32_t packed = atomic_load(&g_local_ds_packed);
        if (packed == 0)
        {
            off = web_append(buf, cap, off, "\"local\":null,");
        }
        else
        {
            off = web_append(buf, cap, off,
                "\"local\":{\"priority1\":%u,\"clock_class\":%u,\"accuracy\":%u,"
                "\"variance\":65535,\"priority2\":%u},",
                (unsigned)((packed >> 16) & 0xFF), (unsigned)(packed & 0xFF),
                (unsigned)((packed >> 8) & 0xFF), (unsigned)((packed >> 24) & 0xFF));
        }
    }
    else if (v->local_valid)
    {
        off = web_append(buf, cap, off,
            "\"local\":{\"priority1\":%u,\"clock_class\":%u,\"accuracy\":%u,"
            "\"variance\":%u,\"priority2\":%u},",
            v->local.priority1, v->local.clock_class, v->local.accuracy,
            v->local.variance, v->local.priority2);
    }
    else
    {
        off = web_append(buf, cap, off, "\"local\":null,");
    }

    /* peer + peer_better */
    if (v->peer_valid)
    {
        char peer_str[24];
        snprintf(peer_str, sizeof(peer_str), "%02X%02X %02X%02X %02X%02X %02X%02X",
                 v->peer.clock_id[0], v->peer.clock_id[1], v->peer.clock_id[2], v->peer.clock_id[3],
                 v->peer.clock_id[4], v->peer.clock_id[5], v->peer.clock_id[6], v->peer.clock_id[7]);
        off = web_append(buf, cap, off,
            "\"peer\":{\"clock_id\":\"%s\",\"priority1\":%u,\"clock_class\":%u,"
            "\"accuracy\":%u,\"variance\":%u,\"priority2\":%u,\"steps_removed\":%u},"
            "\"peer_better\":%s,",
            peer_str, v->peer.priority1, v->peer.clock_class, v->peer.accuracy,
            v->peer.variance, v->peer.priority2, v->peer.steps_removed,
            v->peer_better ? "true" : "false");
    }
    else
    {
        off = web_append(buf, cap, off, "\"peer\":null,\"peer_better\":null,");
    }

    const char *decided = bmca_field_str(v->decided_by);
    off = decided ? web_append(buf, cap, off, "\"decided_by\":\"%s\",", decided)
                 : web_append(buf, cap, off, "\"decided_by\":null,");

    if (v->cand_valid)
    {
        char cand_str[24];
        snprintf(cand_str, sizeof(cand_str), "%02X%02X %02X%02X %02X%02X %02X%02X",
                 v->cand_id[0], v->cand_id[1], v->cand_id[2], v->cand_id[3],
                 v->cand_id[4], v->cand_id[5], v->cand_id[6], v->cand_id[7]);
        off = web_append(buf, cap, off, "\"candidate\":{\"clock_id\":\"%s\",\"count\":%u},",
                         cand_str, (unsigned)v->cand_count);
    }
    else
    {
        off = web_append(buf, cap, off, "\"candidate\":null,");
    }

    if (v->last_ann_rx_ms > 0)
        off = web_append(buf, cap, off, "\"last_announce_rx_age_ms\":%lld,",
                         (long long)(uptime_ms - v->last_ann_rx_ms));
    else
        off = web_append(buf, cap, off, "\"last_announce_rx_age_ms\":null,");

    off = web_append(buf, cap, off, "\"role_changes\":%" PRIu32 ",\"events\":[",
                     v->role_changes);

    /* events[]: ring holds up to BMCA_EVENT_RING entries, oldest overwritten
     * first; ev_next is the slot the NEXT write will use, so ev_next-1 is
     * the newest entry. Walk backwards for newest-first order. */
    bool first = true;
    for (uint8_t k = 0; k < v->ev_count; k++)
    {
        int idx = ((int)v->ev_next - 1 - (int)k + BMCA_EVENT_RING) % BMCA_EVENT_RING;
        const bmca_event_t *e = &v->ev[idx];
        const char *reason = bmca_reason_str(e->reason);
        const char *field = bmca_field_str(e->decided_by);

        off = web_append(buf, cap, off, "%s{\"t_ms\":%lld,\"to\":\"%s\",\"reason\":\"%s\",",
                         first ? "" : ",", (long long)e->t_ms,
                         e->to_master ? "MASTER" : "PASSIVE", reason ? reason : "?");
        if (e->peer_valid)
            off = web_append(buf, cap, off, "\"peer\":\"%02X%02X %02X%02X %02X%02X %02X%02X\",",
                             e->peer_id[0], e->peer_id[1], e->peer_id[2], e->peer_id[3],
                             e->peer_id[4], e->peer_id[5], e->peer_id[6], e->peer_id[7]);
        else
            off = web_append(buf, cap, off, "\"peer\":null,");
        off = field ? web_append(buf, cap, off, "\"decided_by\":\"%s\"}", field)
                   : web_append(buf, cap, off, "\"decided_by\":null}");
        first = false;
    }

    off = web_append(buf, cap, off, "]}");
    return off;
}

/* ----------------------------------------------------------------------------
 *  GET /api/status  --  JSON snapshot
 *
 *  Builds the JSON document the front-end expects (see contract in
 *  status_page.h). Uses snprintf into a fixed buffer rather than cJSON to
 *  keep heap pressure minimal during the high-frequency poll.
 * --------------------------------------------------------------------------- */
static esp_err_t status_handler(httpd_req_t *req)
{
    /* ---------- snapshot every global into local variables ---------- */
    bool   link_up      = atomic_load(&g_link_up);
    bool   l2tap_ready  = atomic_load(&g_l2tap_ready);
    led_state_t st      = atomic_load(&s_led_state);
    bool   i_am_master  = g_i_am_master;

    int64_t offset_ns   = g_last_offset;
    double  freq_ppb    = g_freq_ppb;
    double  integ       = g_integral;
    int     tai_off     = g_utc_offset;
    bool    gnss_valid   = g_gnss_valid;
    time_t  gnss_sec     = g_gnss_seconds;
    time_t  last_gnss_up = g_last_gnss_update;
    int64_t phc_resid   = g_phc_offset_ns;
    uint8_t lock_qual   = 0;  /* good samples (within LOCK_QUAL_GOOD_NS) in the quality window, as on the OLED */
    for (int i = 0; i < LOCK_QUAL_WINDOW; i++)
        lock_qual += g_lock_qual_ring[i];
    int64_t lock_ms     = g_lock_acquired_ms;
    bool    settled     = g_settled_after_holdover;
    uint16_t sync_seq   = g_sync_seq;
    uint16_t ann_seq    = g_announce_seq;
    uint32_t emac_rst   = g_emac_restart_count;
    int      slaves     = slave_active_count();
    uint32_t pps_seq    = g_pps_capture_seq;     /* real PPS edge count */
    uint32_t coast_xtra = g_pps_coast_extra;     /* extra (glitch) PPS edges seen */
    uint32_t coast_miss = g_pps_coast_missing;   /* missing PPS edges seen */
    bool     sec_agree  = g_seconds_agree;       /* receiver and PHC agree on the second */
    int32_t  sec_diff   = g_seconds_diff;        /* receiver second - PHC second (0 = agree) */
    uint32_t sec_dis_n  = g_seconds_disagree_n;  /* consecutive checks with that difference */
    uint32_t rej_total  = g_total_rejects;       /* lifetime servo rejects */
    uint32_t rej_streak = g_consecutive_rejects; /* live reject streak */
    bool     promisc    = g_promiscuous;

    int64_t  now_ms     = esp_timer_get_time() / 1000;
    int64_t  uptime_ms  = now_ms - g_boot_time_ms;
    int64_t  uptime_s   = uptime_ms / 1000;

    /* BMCA seqlock snapshot, taken once and shared with web_append_bmca. A
     * failed snapshot (extremely rare — see bmca_view_snapshot) degrades to
     * "bmca":null rather than failing the whole /api/status response. */
    bmca_view_t bmca_snap;
    bool bmca_ok = bmca_view_snapshot(&bmca_snap);

    /* Heap stats */
    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_lfb  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    /* heap_leak: delta vs. the baseline heap_monitor_task took ~10 s after
     * boot; null until that baseline exists. */
    uint32_t heap_baseline = atomic_load(&g_heap_baseline_free);
    bool     have_leak     = (heap_baseline != 0);
    int32_t  heap_leak     = have_leak ? (int32_t)heap_free - (int32_t)heap_baseline : 0;

    /* ESP32-P4 retention-capable internal SRAM. This reports the live
     * capability heap, rather than a build-specific address copied from the
     * startup log. */
#ifdef MALLOC_CAP_RETENTION
    size_t retent_total = heap_caps_get_total_size(MALLOC_CAP_RETENTION);
    size_t retent_free  = heap_caps_get_free_size(MALLOC_CAP_RETENTION);
#else
    size_t retent_total = 0;
    size_t retent_free  = 0;
#endif

    /* Watchdog health */
    uint8_t  wdt_state  = atomic_load(&g_wdt_state);
    uint32_t wdt_alerts = atomic_load(&g_wdt_alerts);
    uint32_t wdt_last_s = atomic_load(&g_wdt_last_alert_s);
    const char *wdt_state_str;
    switch ((wdt_state_t)wdt_state)
    {
    case WDT_ARMING:        wdt_state_str = "arming";        break;
    case WDT_OK:             wdt_state_str = "ok";            break;
    case WDT_PHC_FROZEN:     wdt_state_str = "phc_frozen";    break;
    case WDT_HEAP_CRITICAL:  wdt_state_str = "heap_critical"; break;
    case WDT_RESTART_LIMIT:  wdt_state_str = "restart_limit"; break;
    default:                 wdt_state_str = "arming";        break;
    }

    /* App info (project name, version, compile time, ELF SHA) */
    const esp_app_desc_t *app = esp_app_get_description();

    /* Chip info */
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    /* MAC address */
    char mac_str[18] = {0};
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             g_src_mac[0], g_src_mac[1], g_src_mac[2],
             g_src_mac[3], g_src_mac[4], g_src_mac[5]);

    /* Clock ID (8-byte EUI-64 derived) */
    char clk_str[24] = {0};
    snprintf(clk_str, sizeof(clk_str), "%02X%02X %02X%02X %02X%02X %02X%02X",
             g_clock_id[0], g_clock_id[1], g_clock_id[2], g_clock_id[3],
             g_clock_id[4], g_clock_id[5], g_clock_id[6], g_clock_id[7]);

    /* IP address + source ("Static"/"DHCP", null while unassigned) */
    char ip_str[20] = "0.0.0.0";
    bool have_ip = false;
    if (g_eth_netif)
    {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(g_eth_netif, &ipinfo) == ESP_OK && ipinfo.ip.addr != 0)
        {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ipinfo.ip));
            have_ip = true;
        }
    }
    bool static_ip = g_static_applied;

    /* GNSS UTC epoch seconds (number), null when GNSS time isn't valid */
    bool have_gnss_utc = gnss_valid && gnss_sec > 1577836800LL;

    /* "Locked for", now a plain second count (0 when not LOCKED) */
    int64_t locked_for_s = 0;
    if (st == LED_STATE_LOCKED && lock_ms > 0)
    {
        locked_for_s = (now_ms - lock_ms) / 1000;
        if (locked_for_s < 0) locked_for_s = 0;
    }

    /* GNSS age in seconds (clamped to a sane range) */
    time_t now_w = time(NULL);
    long gnss_age = (long)(now_w - last_gnss_up);
    if (gnss_age < 0) gnss_age = 0;
    if (gnss_age > 99999) gnss_age = 99999;
    bool gnss_fix = (gnss_age < 3) && gnss_valid;

    /* ---------- FreeRTOS task table ---------- */
    /* uxTaskGetSystemState fills an array of TaskStatus_t; we allocate room
     * for the current number of tasks. */
    UBaseType_t n_tasks = uxTaskGetNumberOfTasks();
    /* clamp for safety */
    if (n_tasks > 40) n_tasks = 40;
    TaskStatus_t *tarr = (TaskStatus_t *)calloc(n_tasks, sizeof(TaskStatus_t));
    UBaseType_t got = 0;
    if (tarr)
    {
        got = uxTaskGetSystemState(tarr, n_tasks, NULL);
    }

    /* ---------- build JSON ---------- */
    /* config (~0.7 KB) + bmca (up to ~1.8 KB with 8 events) push the worst
     * case well past the old 8 KB. */
    const size_t BUF_SZ = 12288;
    char *buf = (char *)malloc(BUF_SZ);
    if (!buf)
    {
        free(tarr);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t off = 0;

    /* ELF SHA256: app_desc holds a 32-byte binary; we want the first 9 hex
     * chars (matches what the front-end displays). */
    char sha_str[20] = {0};
    if (app)
    {
        snprintf(sha_str, sizeof(sha_str),
                 "%02x%02x%02x%02x%02x",
                 app->app_elf_sha256[0], app->app_elf_sha256[1],
                 app->app_elf_sha256[2], app->app_elf_sha256[3],
                 app->app_elf_sha256[4]);
    }

    /* Compile date+time string */
    char compile_str[40] = {0};
    if (app)
    {
        snprintf(compile_str, sizeof(compile_str), "%s %s",
                 app->date, app->time);
    }

    off = web_append(buf, BUF_SZ, off,
        "{"
        "\"chip\":\"ESP32-P4\","
        "\"chip_rev\":\"v%d.%d\","
        "\"cpu_mhz\":%" PRIu32 ","
        "\"project\":\"%s\","
        "\"app_version\":\"%s\","
        "\"idf_version\":\"%s\","
        "\"compile_time\":\"%s\","
        "\"elf_sha\":\"%s\","
        "\"flash_mb\":%" PRIu32 ","
        "\"flash_mode\":\"%s\","
        "\"flash_mhz\":%" PRIu32 ","
        "\"uptime_s\":%lld,"
        "\"heap_free\":%u,"
        "\"heap_min\":%u,"
        "\"heap_lfb\":%u,",
        chip.revision / 100, chip.revision % 100,
        g_boot_facts.cpu_mhz,
        app ? app->project_name : "PTP_Master",
        app ? app->version      : "unknown",
        app ? app->idf_ver      : esp_get_idf_version(),
        compile_str,
        sha_str,
        g_boot_facts.flash_bytes / (1024u * 1024u),
        g_boot_facts.flash_mode,
        g_boot_facts.flash_mhz,
        (long long)uptime_s,
        (unsigned)heap_free,
        (unsigned)heap_min,
        (unsigned)heap_lfb);

    if (have_leak)
        off = web_append(buf, BUF_SZ, off, "\"heap_leak\":%" PRId32 ",", heap_leak);
    else
        off = web_append(buf, BUF_SZ, off, "\"heap_leak\":null,");

    off = web_append(buf, BUF_SZ, off,
        "\"heap_internal_total\":%u,"
        "\"retent_total\":%u,"
        "\"retent_free\":%u,",
        (unsigned)g_boot_facts.heap_internal_total,
        (unsigned)retent_total,
        (unsigned)retent_free);

    if (g_boot_facts.rtc_heap_total >= 0)
        off = web_append(buf, BUF_SZ, off, "\"rtc_heap_total\":%ld,", g_boot_facts.rtc_heap_total);
    else
        off = web_append(buf, BUF_SZ, off, "\"rtc_heap_total\":null,");
    if (g_boot_facts.tcm_heap_total >= 0)
        off = web_append(buf, BUF_SZ, off, "\"tcm_heap_total\":%ld,", g_boot_facts.tcm_heap_total);
    else
        off = web_append(buf, BUF_SZ, off, "\"tcm_heap_total\":null,");

    off = web_append(buf, BUF_SZ, off,
        "\"link_up\":%s,"
        "\"l2tap_ready\":%s,"
        "\"ip\":\"%s\",",
        link_up     ? "true" : "false",
        l2tap_ready ? "true" : "false",
        ip_str);

    if (have_ip)
        off = web_append(buf, BUF_SZ, off, "\"ip_source\":\"%s\",", static_ip ? "Static" : "DHCP");
    else
        off = web_append(buf, BUF_SZ, off, "\"ip_source\":null,");

    off = web_append(buf, BUF_SZ, off,
        "\"mac\":\"%s\","
        "\"hostname\":\"%s\","
        "\"emac_restarts\":%" PRIu32 ","
        "\"promiscuous\":%s,"
        "\"wdt\":{\"state\":\"%s\",\"alerts\":%" PRIu32 ",",
        mac_str,
        g_hostname[0] ? g_hostname : "",
        emac_rst,
        promisc ? "true" : "false",
        wdt_state_str, wdt_alerts);

    if (wdt_last_s > 0)
        off = web_append(buf, BUF_SZ, off, "\"last_alert_uptime_s\":%" PRIu32 "},", wdt_last_s);
    else
        off = web_append(buf, BUF_SZ, off, "\"last_alert_uptime_s\":null},");

    off = web_append(buf, BUF_SZ, off,
        "\"role\":\"%s\","
        "\"clock_id\":\"%s\",",
        i_am_master ? "MASTER" : "PASSIVE",
        clk_str);

    if (have_gnss_utc)
        off = web_append(buf, BUF_SZ, off, "\"gnss_utc_s\":%lld,", (long long)gnss_sec);
    else
        off = web_append(buf, BUF_SZ, off, "\"gnss_utc_s\":null,");

    off = web_append(buf, BUF_SZ, off,
        "\"tai_offset\":%d,"
        "\"sync_seq\":%u,"
        "\"announce_seq\":%u,"
        "\"slaves_active\":%d,"
        /* servo_gain_set was evaluated (see §4.7 of the BMCA/JSON handover)
         * and deliberately NOT added: LED-derived servo_state already tells
         * the dashboard which gain set is active (ACQUIRING => acquire
         * gains, LOCKED/HOLDOVER => lock gains) without exposing servo_task's
         * internal state enum. */
        "\"servo_state\":\"%s\","
        "\"locked_for_s\":%lld,"
        "\"offset_ns\":%lld,"
        "\"freq_ppb\":%.2f,"
        "\"integrator\":%.2f,"
        "\"lock_qual\":%u,"
        "\"pps_count\":%" PRIu32 ","
        "\"pps_coast_extra\":%" PRIu32 ","
        "\"pps_coast_missing\":%" PRIu32 ","
        "\"reject_total\":%" PRIu32 ","
        "\"reject_streak\":%" PRIu32 ","
        "\"phc_residual_ns\":%lld,"
        "\"sec_agree\":%s,"
        "\"sec_diff\":%ld,"
        "\"sec_disagree_n\":%" PRIu32 ","
        "\"settled\":%s,"
        "\"gnss_age_s\":%ld,"
        "\"gnss_fix\":%s,",
        tai_off,
        (unsigned)sync_seq,
        (unsigned)ann_seq,
        slaves,
        web_state_str(st),
        (long long)locked_for_s,
        (long long)offset_ns,
        freq_ppb,
        integ,
        (unsigned)lock_qual,
        pps_seq,
        coast_xtra,
        coast_miss,
        rej_total,
        rej_streak,
        (long long)phc_resid,
        sec_agree ? "true" : "false",
        (long)sec_diff,
        sec_dis_n,
        settled ? "true" : "false",
        gnss_age,
        gnss_fix ? "true" : "false");

    if (bmca_ok)
        off = web_append_bmca(buf, BUF_SZ, off, i_am_master, uptime_ms, &bmca_snap);
    else
        off = web_append(buf, BUF_SZ, off, "\"bmca\":null");
    off = web_append(buf, BUF_SZ, off, ",");

    off = web_append(buf, BUF_SZ, off, "%s,", g_config_json);

    off = web_append(buf, BUF_SZ, off, "\"tasks\":[");

    /* Append task array */
    bool first = true;
    for (UBaseType_t i = 0; i < got && off < BUF_SZ - 200; i++)
    {
        const TaskStatus_t *t = &tarr[i];
        /* high-water mark is reported in StackType_t units; convert to bytes */
        uint32_t hwm_bytes = (uint32_t)t->usStackHighWaterMark * sizeof(StackType_t);
        const char *name = t->pcTaskName ? t->pcTaskName : "?";
        long stack_bytes = task_reg_lookup(name); /* -1 => unknown => JSON null */

        int core = web_task_core(t->xHandle);

        off = web_append(buf, BUF_SZ, off,
            "%s{\"name\":\"%s\",\"priority\":%u,\"stack_bytes\":",
            first ? "" : ",", name, (unsigned)t->uxCurrentPriority);
        if (stack_bytes >= 0)
            off = web_append(buf, BUF_SZ, off, "%ld", stack_bytes);
        else
            off = web_append(buf, BUF_SZ, off, "null");
        off = web_append(buf, BUF_SZ, off,
            ",\"hwm_bytes\":%" PRIu32 ",\"core\":%d}",
            hwm_bytes, core);
        first = false;
    }

    off = web_append(buf, BUF_SZ, off, "]}");

    free(tarr);

    /* web_append() saturates `off` at BUF_SZ-1 on truncation. Never send a
     * truncated (invalid) JSON document. */
    if (off >= BUF_SZ - 2)
    {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    esp_err_t r = httpd_resp_send(req, buf, off);
    free(buf);
    return r;
}

/* ----------------------------------------------------------------------------
 *  GET /api/bmca  --  standalone BMCA snapshot
 *
 *  Lets the dashboard's BMCA card poll at 1 Hz without the task-table walk
 *  /api/status does. Same seqlock, same web_append_bmca() fragment as
 *  /api/status, so the two endpoints can never disagree.
 * --------------------------------------------------------------------------- */
static esp_err_t bmca_handler(httpd_req_t *req)
{
    bool i_am_master = g_i_am_master;
    int64_t uptime_ms = esp_timer_get_time() / 1000 - g_boot_time_ms;

    bmca_view_t v;
    if (!bmca_view_snapshot(&v))
    {
        /* Never emit a torn object. */
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char clk_str[24];
    snprintf(clk_str, sizeof(clk_str), "%02X%02X %02X%02X %02X%02X %02X%02X",
             g_clock_id[0], g_clock_id[1], g_clock_id[2], g_clock_id[3],
             g_clock_id[4], g_clock_id[5], g_clock_id[6], g_clock_id[7]);

    const size_t BUF_SZ = 2048;
    char *buf = (char *)malloc(BUF_SZ);
    if (!buf)
    {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t off = 0;

    off = web_append(buf, BUF_SZ, off,
        "{\"uptime_ms\":%lld,\"role\":\"%s\",\"clock_id\":\"%s\",",
        (long long)uptime_ms, i_am_master ? "MASTER" : "PASSIVE", clk_str);
    off = web_append_bmca(buf, BUF_SZ, off, i_am_master, uptime_ms, &v);
    off = web_append(buf, BUF_SZ, off, "}");

    if (off >= BUF_SZ - 2)
    {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t r = httpd_resp_send(req, buf, off);
    free(buf);
    return r;
}

/* ----------------------------------------------------------------------------
 *  POST /api/bmca/priority?p1=N&p2=M  --  runtime priority1/priority2
 *
 *  Same credentials as OTA (ota_check_basic_auth). RAM-only: a reboot
 *  restores PTP_PRIORITY1_DEFAULT/PTP_PRIORITY2_DEFAULT (see g_priority1/
 *  g_priority2 in main.c) — no NVS write, by design.
 * --------------------------------------------------------------------------- */
static esp_err_t bmca_priority_handler(httpd_req_t *req)
{
    if (ota_check_basic_auth(req) != ESP_OK)
        return ESP_FAIL; /* helper already sent 401/503 */

    /* No request body is expected; drain one if present so the connection
     * can be reused cleanly. */
    int remaining = req->content_len;
    char drain[64];
    while (remaining > 0)
    {
        int chunk = remaining < (int)sizeof(drain) ? remaining : (int)sizeof(drain);
        int r = httpd_req_recv(req, drain, chunk);
        if (r <= 0)
            break;
        remaining -= r;
    }

    char query[64] = {0};
    bool have_p1 = false, have_p2 = false;
    long p1 = 0, p2 = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        char val[8];
        if (httpd_query_key_value(query, "p1", val, sizeof(val)) == ESP_OK)
        {
            char *end = NULL;
            long v = strtol(val, &end, 10);
            if (end == val || *end != '\0' || v < 0 || v > 255)
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "p1 must be an integer 0-255");
                return ESP_FAIL;
            }
            p1 = v;
            have_p1 = true;
        }
        if (httpd_query_key_value(query, "p2", val, sizeof(val)) == ESP_OK)
        {
            char *end = NULL;
            long v = strtol(val, &end, 10);
            if (end == val || *end != '\0' || v < 0 || v > 255)
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "p2 must be an integer 0-255");
                return ESP_FAIL;
            }
            p2 = v;
            have_p2 = true;
        }
    }

    if (!have_p1 && !have_p2)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "specify p1 and/or p2 (0-255)");
        return ESP_FAIL;
    }

    /* Takes effect on the next Announce sent and the next Announce
     * received — no forced BMCA re-evaluation needed. */
    if (have_p1) atomic_store(&g_priority1, (uint8_t)p1);
    if (have_p2) atomic_store(&g_priority2, (uint8_t)p2);

    char resp[64];
    int n = snprintf(resp, sizeof(resp), "{\"priority1\":%u,\"priority2\":%u}",
                     (unsigned)atomic_load(&g_priority1), (unsigned)atomic_load(&g_priority2));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, n);
}

/* ----------------------------------------------------------------------------
 *  GET /api/health  --  lightweight JSON for load / soak monitoring
 *
 *  Surfaces the four things a flood or multi-hour soak needs to watch:
 *    - heap: free / minimum-ever-free / largest-free-block (catches the
 *      "no mem for receive buffer" RX-descriptor exhaustion drift)
 *    - sync_jitter: spacing of consecutive Sync HW TX timestamps, in µs
 *      (widening/scattering = ptp_tx_task starved under Delay_Req load)
 *    - per-task cpu_pct: INSTANTANEOUS share of CPU since the previous poll
 *      (watch ptp_rx climb), plus stack_hwm per task
 *    - rx_task: a convenience block pulling ptp_rx's stack_hwm + cpu_pct out
 *
 *  Kept separate from /api/status so it stays cheap to poll at 1 Hz and never
 *  perturbs the status page. cpu_pct requires CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
 *  (and CONFIG_FREERTOS_USE_TRACE_FACILITY, already on); if stats are off,
 *  "runtime_stats":false and every cpu_pct is null — all other fields still work.
 * --------------------------------------------------------------------------- */
#if (configGENERATE_RUN_TIME_STATS == 1)
/* Previous per-task run-time counters, so we report instantaneous CPU% (share
 * of CPU time since the last poll) rather than a lifetime average that barely
 * moves. Keyed by task handle; stale entries from exited tasks are harmless.
 * The 32-bit counters wrap (~tens of minutes), but unsigned subtraction yields
 * the correct delta across a single wrap, which 1 Hz polling never exceeds. */
typedef struct { TaskHandle_t h; uint32_t prev_run; } web_cpu_prev_t;
static web_cpu_prev_t s_cpu_prev[48];
static uint32_t s_cpu_prev_n = 0;
static uint32_t s_cpu_prev_total = 0;

static uint32_t *web_cpu_slot(TaskHandle_t h)
{
    for (uint32_t i = 0; i < s_cpu_prev_n; i++)
        if (s_cpu_prev[i].h == h) return &s_cpu_prev[i].prev_run;
    if (s_cpu_prev_n < (sizeof(s_cpu_prev) / sizeof(s_cpu_prev[0])))
    {
        s_cpu_prev[s_cpu_prev_n].h = h;
        s_cpu_prev[s_cpu_prev_n].prev_run = 0;
        return &s_cpu_prev[s_cpu_prev_n++].prev_run;
    }
    return NULL;
}
#endif

static esp_err_t health_handler(httpd_req_t *req)
{
    int64_t now_ms   = esp_timer_get_time() / 1000;
    int64_t uptime_s = (now_ms - g_boot_time_ms) / 1000;

    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_lfb  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    /* Sync-interval jitter (µs), measured in main.c at the Sync TX path. */
    int64_t sj_min, sj_max, sj_mean, sj_last;
    uint32_t sj_win;
    uint32_t sj_count = ptp_sync_jitter_stats(&sj_min, &sj_max, &sj_mean, &sj_last, &sj_win);

    /* FreeRTOS task table (+ total run time for CPU%). */
    UBaseType_t n_tasks = uxTaskGetNumberOfTasks();
    if (n_tasks > 40) n_tasks = 40;
    TaskStatus_t *tarr = (TaskStatus_t *)calloc(n_tasks, sizeof(TaskStatus_t));
    UBaseType_t got = 0;
    uint32_t total_run = 0;
    if (tarr)
        got = uxTaskGetSystemState(tarr, n_tasks, &total_run);

#if (configGENERATE_RUN_TIME_STATS == 1)
    uint32_t dtotal = total_run - s_cpu_prev_total;       /* unsigned: wrap-safe */
    bool have_cpu = (s_cpu_prev_total != 0) && (dtotal != 0);
#else
    bool have_cpu = false;
    (void)total_run;
#endif

    const size_t BUF_SZ = 6144;
    char *buf = (char *)malloc(BUF_SZ);
    if (!buf)
    {
        free(tarr);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t off = 0;

    uint32_t rx_hwm = 0;
    double   rx_cpu = -1.0;

    off = web_append(buf, BUF_SZ, off,
        "{"
        "\"uptime_s\":%lld,"
        "\"runtime_stats\":%s,"
        "\"heap\":{\"free\":%u,\"min\":%u,\"lfb\":%u},"
        "\"sync_jitter\":{\"nominal_us\":1000000,\"window\":%" PRIu32 ",\"count\":%" PRIu32 ","
            "\"last_us\":%lld,\"min_us\":%lld,\"max_us\":%lld,\"mean_us\":%lld,\"p2p_us\":%lld},"
        "\"tasks\":[",
        (long long)uptime_s,
        have_cpu ? "true" : "false",
        (unsigned)heap_free, (unsigned)heap_min, (unsigned)heap_lfb,
        sj_win, sj_count,
        (long long)sj_last, (long long)sj_min, (long long)sj_max, (long long)sj_mean,
        (long long)(sj_win ? (sj_max - sj_min) : 0));

    bool first = true;
    for (UBaseType_t i = 0; i < got && off < BUF_SZ - 160; i++)
    {
        const TaskStatus_t *t = &tarr[i];
        const char *name = t->pcTaskName ? t->pcTaskName : "?";
        uint32_t hwm_bytes = (uint32_t)t->usStackHighWaterMark * sizeof(StackType_t);

        double cpu = -1.0;
#if (configGENERATE_RUN_TIME_STATS == 1)
        uint32_t *prev = web_cpu_slot(t->xHandle);
        if (prev)
        {
            uint32_t drun = (uint32_t)t->ulRunTimeCounter - *prev; /* wrap-safe */
            if (have_cpu) cpu = (100.0 * (double)drun) / (double)dtotal;
            *prev = (uint32_t)t->ulRunTimeCounter;
        }
#endif
        if (!strcmp(name, "ptp_rx")) { rx_hwm = hwm_bytes; rx_cpu = cpu; }

        if (cpu < 0.0)
            off = web_append(buf, BUF_SZ, off,
                "%s{\"name\":\"%s\",\"prio\":%u,\"stack_hwm\":%" PRIu32 ",\"cpu_pct\":null}",
                first ? "" : ",", name, (unsigned)t->uxCurrentPriority, hwm_bytes);
        else
            off = web_append(buf, BUF_SZ, off,
                "%s{\"name\":\"%s\",\"prio\":%u,\"stack_hwm\":%" PRIu32 ",\"cpu_pct\":%.1f}",
                first ? "" : ",", name, (unsigned)t->uxCurrentPriority, hwm_bytes, cpu);
        first = false;
    }

#if (configGENERATE_RUN_TIME_STATS == 1)
    s_cpu_prev_total = total_run;
#endif

    off = web_append(buf, BUF_SZ, off,
        "],\"rx_task\":{\"stack_size\":8192,\"stack_hwm\":%" PRIu32 ",", rx_hwm);
    if (rx_cpu < 0.0)
        off = web_append(buf, BUF_SZ, off, "\"cpu_pct\":null}}");
    else
        off = web_append(buf, BUF_SZ, off, "\"cpu_pct\":%.1f}}", rx_cpu);

    free(tarr);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    esp_err_t r = httpd_resp_send(req, buf, off);
    free(buf);
    return r;
}

/* ----------------------------------------------------------------------------
 *  GET /ota  --  serves the embedded OTA upload page
 * --------------------------------------------------------------------------- */
static esp_err_t ota_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, OTA_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ----------------------------------------------------------------------------
 *  POST /api/reboot  --  restart the device
 *
 *  Same credentials as OTA (ota_check_basic_auth). Answers first, then
 *  restarts from a small task after 1 s (as an OTA update does), so the client
 *  gets the reply. Refused with 409 while an OTA upload is in progress.
 * --------------------------------------------------------------------------- */
static void web_reboot_task(void *arg)
{
    (void)arg;
    ESP_LOGW(WEB_TAG, "Rebooting in 1 second (requested via /api/reboot)...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    if (ota_check_basic_auth(req) != ESP_OK)
        return ESP_FAIL; /* helper already sent 401/503 */

    /* No request body is expected; drain one if present. */
    int remaining = req->content_len;
    char drain[64];
    while (remaining > 0)
    {
        int chunk = remaining < (int)sizeof(drain) ? remaining : (int)sizeof(drain);
        int r = httpd_req_recv(req, drain, chunk);
        if (r <= 0)
            break;
        remaining -= r;
    }

    httpd_resp_set_type(req, "application/json");
    if (atomic_load(&g_ota_in_progress))
    {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "{\"error\":\"OTA update in progress\"}");
        return ESP_OK;
    }
    httpd_resp_sendstr(req, "{\"rebooting\":true,\"in_ms\":1000}");
    if (xTaskCreate(web_reboot_task, "web_reboot", 2048, NULL, 5, NULL) != pdPASS)
    {
        /* No memory for a 2 KB task: the client was promised a reboot. */
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 *  web_server_start()  --  start the httpd
 *
 *  Safe to call any time after esp_netif_init() and esp_event_loop_create()
 *  -- which the project already does inside ethernet_init(). The TCP/IP
 *  stack does not need a live link to accept the listening socket; once
 *  Ethernet comes up and DHCP assigns an IP, the server will be reachable
 *  on port 80.
 *
 *  Returns the handle for advanced use, or NULL on failure.
 * --------------------------------------------------------------------------- */
static httpd_handle_t web_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* The defaults are fine for the status page, but we need extra headroom
     * for OTA uploads: a longer recv timeout (slow uploaders, ~3 MB images)
     * and more URI handlers. */
    config.server_port      = 80;
    config.ctrl_port        = 32768;        /* default; explicit for clarity */
    config.max_open_sockets = 4;            /* page + a few XHRs */
    config.max_uri_handlers = 10;           /* OTA routes + /api/bmca + /api/bmca/priority + /api/reboot (9 used) */
    config.stack_size       = 8192;         /* OTA needs slightly more stack */
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 30;          /* OTA: large uploads can pause */
    config.send_wait_timeout = 10;

    /* The httpd task itself never goes through task_create_tracked() (IDF
     * creates it internally); register its stack size directly so the
     * dashboard's task table shows a real stack_bytes instead of null. The
     * task name is "httpd" (esp_http_server's httpd_os_thread_create). */
    task_reg_add("httpd", config.stack_size);

    ESP_LOGI(WEB_TAG, "Starting HTTP server on port %d", config.server_port);
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(WEB_TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return NULL;
    }

    static const httpd_uri_t uri_root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_status = {
        .uri      = "/api/status",
        .method   = HTTP_GET,
        .handler  = status_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_health = {
        .uri      = "/api/health",
        .method   = HTTP_GET,
        .handler  = health_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_ota_page = {
        .uri      = "/ota",
        .method   = HTTP_GET,
        .handler  = ota_page_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_ota_info = {
        .uri      = "/api/ota/info",
        .method   = HTTP_GET,
        .handler  = ota_info_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_ota_upload = {
        .uri      = "/api/ota",
        .method   = HTTP_POST,
        .handler  = ota_upload_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_bmca = {
        .uri      = "/api/bmca",
        .method   = HTTP_GET,
        .handler  = bmca_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_bmca_priority = {
        .uri      = "/api/bmca/priority",
        .method   = HTTP_POST,
        .handler  = bmca_priority_handler,
        .user_ctx = NULL,
    };
    static const httpd_uri_t uri_reboot = {
        .uri      = "/api/reboot",
        .method   = HTTP_POST,
        .handler  = reboot_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_status);
    httpd_register_uri_handler(server, &uri_health);
    httpd_register_uri_handler(server, &uri_ota_page);
    httpd_register_uri_handler(server, &uri_ota_info);
    httpd_register_uri_handler(server, &uri_ota_upload);
    httpd_register_uri_handler(server, &uri_bmca);
    httpd_register_uri_handler(server, &uri_bmca_priority);
    httpd_register_uri_handler(server, &uri_reboot);

    ESP_LOGI(WEB_TAG, "HTTP server ready -- open http://<device-ip>/ in a browser");
    ESP_LOGI(WEB_TAG, "  GET  /            -> status page");
    ESP_LOGI(WEB_TAG, "  POST /api/reboot  -> restart (OTA credentials)");
    ESP_LOGI(WEB_TAG, "  GET  /api/status  -> status JSON");
    ESP_LOGI(WEB_TAG, "  GET  /api/health  -> load/soak health JSON (heap/cpu/stack/jitter)");
    ESP_LOGI(WEB_TAG, "  GET  /api/bmca    -> BMCA snapshot JSON (1 Hz poll)");
    ESP_LOGI(WEB_TAG, "  POST /api/bmca/priority -> set priority1/priority2 (OTA auth)");
    ESP_LOGI(WEB_TAG, "  GET  /ota         -> OTA upload page");
    ESP_LOGI(WEB_TAG, "  GET  /api/ota/info-> OTA partition info JSON");
    ESP_LOGI(WEB_TAG, "  POST /api/ota     -> firmware image upload");
    return server;
}

#endif /* WEB_SERVER_H */
