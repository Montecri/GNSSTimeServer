/* ============================================================================
 *  web_server.h  --  Drop-in HTTP server for ESP32-P4 PTP Slave
 *
 *  Serves five endpoints:
 *
 *    GET /              -> the status_page.h HTML (served from flash/RAM)
 *    GET /api/status    -> JSON snapshot of every runtime value the page needs
 *    GET /api/health    -> lightweight JSON for soak / load monitoring
 *    GET /ota           -> the ota_page.h HTML (firmware upload page)
 *    GET /api/ota/info  -> JSON describing partitions + OTA state
 *    POST /api/ota      -> firmware image upload (Basic-Auth protected)
 *
 *  HOW TO USE (one-line integration in main.c):
 *
 *      #include "esp_app_desc.h"   // NOTE: also add this above (slave was missing it)
 *      #include "status_page.h"
 *      #include "web_server.h"
 *      ...
 *      // Inside app_main(), AFTER ethernet_init() and AFTER xTaskCreate(...)
 *      // calls, anywhere is fine -- the server self-gates on g_link_up:
 *      web_server_start();
 *
 *  The header is single-include and contains the function definitions. It is
 *  designed to be included EXACTLY ONCE, from main.c, AFTER all the static
 *  globals it reads. It pulls in the OTA page + handler unconditionally.
 *
 *  Required component dependency: add "esp_http_server" and "app_update" to
 *  your component's REQUIRES list (the OTA pieces need app_update).
 *
 *  Memory cost: ~6 KB stack for the httpd task plus ~70 KB rodata for the
 *  embedded HTML (status_page + ota_page). Negligible RAM impact.
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
#include <math.h>
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

/* CPU frequency: read from menuconfig (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ). */
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
 * naive `off += snprintf(buf + off, CAP - off, ...)` pattern therefore lets
 * `off` sail past CAP on the first truncation, after which the next call's
 * `CAP - off` underflows (size_t) and writes out of bounds. This helper
 * makes it impossible: `off` saturates at CAP-1 (the NUL) and further
 * appends become no-ops, yielding truncated-but-safe JSON — the caller
 * checks for truncation once, at the end, and sends 500 instead. */
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

/* The static globals below are defined in main-slave.c. They are referenced
 * directly because this header is included from main-slave.c (single TU). */

static const char *WEB_TAG = "WEB";

/* ----------------------------------------------------------------------------
 *  GET /  --  serves the embedded HTML status page
 * --------------------------------------------------------------------------- */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, STATUS_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ----------------------------------------------------------------------------
 *  Helpers shared by status_handler and health_handler
 * --------------------------------------------------------------------------- */

/* Translate the led_state_t enum into the strings the front-end expects. */
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

/* Lookup a logical CPU core per task. Reports -1 if no affinity. */
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

/* Compute mean and stddev of an int64 array. Same shape as the master's
 * helper but inlined here so this header stays self-contained. */
static void web_stats_i64(const int64_t *arr, int n,
                          double *out_mean, double *out_sd)
{
    if (n <= 0)
    {
        *out_mean = 0.0;
        *out_sd   = 0.0;
        return;
    }
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += (double)arr[i];
    double mean = sum / (double)n;
    double sq = 0.0;
    for (int i = 0; i < n; i++)
    {
        double d = (double)arr[i] - mean;
        sq += d * d;
    }
    *out_mean = mean;
    *out_sd   = (n > 1) ? sqrt(sq / (double)(n - 1)) : 0.0;
}

/* ----------------------------------------------------------------------------
 *  GET /api/status  --  JSON snapshot
 *
 *  Builds slave-shaped JSON: live PTP offset/delay, GM identity/properties,
 *  per-message counters, and a FreeRTOS task table. The HTML page in
 *  status_page.h knows how to render every field below.
 * --------------------------------------------------------------------------- */
static esp_err_t status_handler(httpd_req_t *req)
{
    /* ---------- snapshot every shared global into local variables ---------- */
    bool        link_up      = atomic_load(&g_link_up);
    bool        l2tap_ready  = atomic_load(&g_l2tap_ready);
    led_state_t st           = atomic_load(&s_led_state);

    /* PTP slave state */
    int64_t offset_ns    = g_slave_offset_ns;
    int64_t path_delay   = g_slave_path_delay_ns;
    double  freq_ppb     = g_slave_freq_ppb;
    bool    slave_locked = g_slave_locked;
    int64_t lock_ms      = g_lock_acquired_ms;
    int64_t last_sync_ms = g_last_sync_ms;

    /* Shared system state */
    uint32_t emac_rst    = g_emac_restart_count;

    /* GM identity (Announce-derived) — copy under spinlock */
    gm_props_t gp;
    portENTER_CRITICAL(&g_gm_props_lock);
    gp = g_gm_props;
    portEXIT_CRITICAL(&g_gm_props_lock);

    /* GM diagnostics: snapshot counters + windowed stats + Sync/Announce
     * sender identity. Hold the lock once, copy out, then release before
     * doing the float maths. */
    int64_t off_win[GM_DIAG_WINDOW];
    int64_t dly_win[GM_DIAG_WINDOW];
    int64_t sj_win[GM_DIAG_WINDOW];
    int64_t fl_win[GM_DIAG_WINDOW];
    int off_n, dly_n, sj_n, fl_n;

    uint32_t cnt_sync, cnt_fu, cnt_dr, cnt_ann;
    uint32_t gap_sync, gap_ann;
    uint32_t orph_fu, orph_dr;
    uint32_t one_step;
    uint32_t gm_id_chg, post_lock_sp;
    uint8_t  sync_src_id[8];
    bool     sync_src_known;
    uint32_t sync_src_changes;
    uint8_t  gm_clkid[8];
    bool     gm_clkid_known;

    portENTER_CRITICAL(&g_gm_diag_lock);
    off_n = g_gm_diag.offset_count;
    dly_n = g_gm_diag.delay_count;
    sj_n  = g_gm_diag.sync_int_count;
    fl_n  = g_gm_diag.fu_lat_count;
    memcpy(off_win, g_gm_diag.offset_window,           sizeof(int64_t) * off_n);
    memcpy(dly_win, g_gm_diag.delay_window,            sizeof(int64_t) * dly_n);
    memcpy(sj_win,  g_gm_diag.sync_interval_window,    sizeof(int64_t) * sj_n);
    memcpy(fl_win,  g_gm_diag.followup_latency_window, sizeof(int64_t) * fl_n);
    cnt_sync         = g_gm_diag.sync_rx_count;
    cnt_fu           = g_gm_diag.followup_rx_count;
    cnt_dr           = g_gm_diag.delay_resp_rx_count;
    cnt_ann          = g_gm_diag.announce_rx_count;
    gap_sync         = g_gm_diag.sync_seq_gaps;
    gap_ann          = g_gm_diag.announce_seq_gaps;
    orph_fu          = g_gm_diag.followup_orphans;
    orph_dr          = g_gm_diag.delay_resp_orphans;
    one_step         = g_gm_diag.one_step_count;
    gm_id_chg        = g_gm_diag.gm_identity_changes;
    post_lock_sp     = g_gm_diag.post_lock_spikes;
    sync_src_known   = g_gm_diag.sync_src_known;
    memcpy(sync_src_id, g_gm_diag.sync_src_id, 8);
    sync_src_changes = g_gm_diag.sync_src_changes;
    gm_clkid_known   = g_gm_diag.gm_id_known;
    memcpy(gm_clkid, g_gm_diag.current_gm_id, 8);
    portEXIT_CRITICAL(&g_gm_diag_lock);

    /* Asymmetry windows — same idea, separate lock */
    int64_t mssl[ASYMMETRY_WINDOW];
    int64_t slms[ASYMMETRY_WINDOW];
    int asy_n;
    int64_t cm2s_sum = 0, cs2m_sum = 0;
    portENTER_CRITICAL(&g_asymm_lock);
    asy_n = g_asymm_count;
    memcpy(mssl, g_ms_to_sl_window, sizeof(int64_t) * asy_n);
    memcpy(slms, g_sl_to_ms_window, sizeof(int64_t) * asy_n);
    /* TC correction windows: sum in place — mirrors the OLED path and
     * avoids two more 512 B buffers on the httpd task's stack. */
    for (int i = 0; i < asy_n; i++)
    {
        cm2s_sum += g_corr_m2s_window[i];
        cs2m_sum += g_corr_s2m_window[i];
    }
    portEXIT_CRITICAL(&g_asymm_lock);

    /* ---------- derived metrics ---------- */
    double off_mean = 0.0, off_sd = 0.0;
    double dly_mean = 0.0, dly_sd = 0.0;
    double sj_mean  = 0.0, sj_sd  = 0.0;
    double fl_mean  = 0.0, fl_sd  = 0.0;
    web_stats_i64(off_win, off_n, &off_mean, &off_sd);
    web_stats_i64(dly_win, dly_n, &dly_mean, &dly_sd);
    web_stats_i64(sj_win,  sj_n,  &sj_mean,  &sj_sd);
    web_stats_i64(fl_win,  fl_n,  &fl_mean,  &fl_sd);

    /* Quality: % of recent offsets within the tight/loose envelopes —
     * same thresholds the OLED servo screen's Q-tight/Q-loose uses. */
    int qual_pct = 0, qual_loose_pct = 0;
    if (off_n > 0)
    {
        int good_tight = 0, good_loose = 0;
        for (int i = 0; i < off_n; i++)
        {
            int64_t a = llabs(off_win[i]);
            if (a <= SLAVE_QUALITY_THRESHOLD_TIGHT_NS) good_tight++;
            if (a <= SLAVE_QUALITY_THRESHOLD_LOOSE_NS) good_loose++;
        }
        qual_pct       = (good_tight * 100) / off_n;
        qual_loose_pct = (good_loose * 100) / off_n;
    }

    /* Linreg slope — re-use the slave's helper directly */
    double slope_ppb = linreg_compute_slope_ppb();

    /* Asymmetry mean (ms→sl minus sl→ms half-trips averaged over the window) */
    int64_t asy_mean_ns = 0;
    if (asy_n > 0)
    {
        int64_t sa = 0, sb = 0;
        for (int i = 0; i < asy_n; i++) { sa += mssl[i]; sb += slms[i]; }
        asy_mean_ns = (sa - sb) / asy_n;
    }

    /* TC residence-time correction means (ns) over the same window:
     * what the transparent clock stamped into each leg. 0/0 on a direct
     * cable (nothing writes correctionField end-to-end); non-zero when a
     * TC is matching + correcting inline. */
    int64_t corr_m2s_mean_ns = 0, corr_s2m_mean_ns = 0;
    if (asy_n > 0)
    {
        corr_m2s_mean_ns = cm2s_sum / asy_n;
        corr_s2m_mean_ns = cs2m_sum / asy_n;
    }

    /* ---------- common derived values ---------- */
    int64_t now_ms   = esp_timer_get_time() / 1000;
    int64_t uptime_s = (now_ms - g_boot_time_ms) / 1000;

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
     * capability heap, rather than a build-specific address copied from a
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

    /* MAC + clock ID + IP */
    char mac_str[18] = {0};
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             g_src_mac[0], g_src_mac[1], g_src_mac[2],
             g_src_mac[3], g_src_mac[4], g_src_mac[5]);

    char clk_str[24] = {0};
    snprintf(clk_str, sizeof(clk_str), "%02X%02X %02X%02X %02X%02X %02X%02X",
             g_clock_id[0], g_clock_id[1], g_clock_id[2], g_clock_id[3],
             g_clock_id[4], g_clock_id[5], g_clock_id[6], g_clock_id[7]);

    /* Sync-source identity — clockIdentity from the last Sync's
     * sourcePortIdentity. Same string format as clock_id/gm_clock_id so the
     * dashboard can do a plain string compare against gm_clock_id. */
    char sync_src_str[24] = {0};
    if (sync_src_known)
    {
        snprintf(sync_src_str, sizeof(sync_src_str),
                 "%02X%02X %02X%02X %02X%02X %02X%02X",
                 sync_src_id[0], sync_src_id[1], sync_src_id[2], sync_src_id[3],
                 sync_src_id[4], sync_src_id[5], sync_src_id[6], sync_src_id[7]);
    }

    /* GM "clock ID" — the Announce sender's clockIdentity. */
    char gm_clkid_str[24] = {0};
    if (gm_clkid_known)
    {
        snprintf(gm_clkid_str, sizeof(gm_clkid_str),
                 "%02X%02X %02X%02X %02X%02X %02X%02X",
                 gm_clkid[0], gm_clkid[1], gm_clkid[2], gm_clkid[3],
                 gm_clkid[4], gm_clkid[5], gm_clkid[6], gm_clkid[7]);
    }

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

    /* "Locked for", now a plain second count (0 when not locked) */
    int64_t locked_for_s = 0;
    if (slave_locked && lock_ms > 0)
    {
        locked_for_s = (now_ms - lock_ms) / 1000;
        if (locked_for_s < 0) locked_for_s = 0;
    }

    /* Last-Sync age (ms) — clamp to >= 0 */
    int64_t last_sync_age_ms = (last_sync_ms > 0) ? (now_ms - last_sync_ms) : -1;
    if (last_sync_age_ms < 0 && last_sync_ms > 0) last_sync_age_ms = 0;

    /* ---------- FreeRTOS task table ---------- */
    UBaseType_t n_tasks = uxTaskGetNumberOfTasks();
    if (n_tasks > 40) n_tasks = 40;
    TaskStatus_t *tarr = (TaskStatus_t *)calloc(n_tasks, sizeof(TaskStatus_t));
    UBaseType_t got = 0;
    if (tarr) got = uxTaskGetSystemState(tarr, n_tasks, NULL);

    /* ---------- build JSON ---------- */
    /* config (~0.4 KB) + gm_flags + wdt push the worst case past the old
     * 8 KB buffer. */
    const size_t BUF_SZ = 10240;
    char *buf = (char *)malloc(BUF_SZ);
    if (!buf)
    {
        free(tarr);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t off = 0;

    /* ELF SHA256 prefix (first 5 bytes -> 10 hex; UI shows first 9 + ellipsis) */
    char sha_str[20] = {0};
    if (app)
    {
        snprintf(sha_str, sizeof(sha_str),
                 "%02x%02x%02x%02x%02x",
                 app->app_elf_sha256[0], app->app_elf_sha256[1],
                 app->app_elf_sha256[2], app->app_elf_sha256[3],
                 app->app_elf_sha256[4]);
    }

    char compile_str[40] = {0};
    if (app) snprintf(compile_str, sizeof(compile_str), "%s %s", app->date, app->time);

    off = web_append(buf, BUF_SZ, off,
        "{"
        /* device */
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
        /* memory */
        "\"heap_free\":%u,"
        "\"heap_min\":%u,"
        "\"heap_lfb\":%u,",
        chip.revision / 100, chip.revision % 100,
        g_boot_facts.cpu_mhz,
        app ? app->project_name : "PTP_Slave",
        app ? app->version      : "unknown",
        app ? app->idf_ver      : esp_get_idf_version(),
        compile_str,
        sha_str,
        g_boot_facts.flash_bytes / (1024u * 1024u),
        g_boot_facts.flash_mode,
        g_boot_facts.flash_mhz,
        (long long)uptime_s,
        (unsigned)heap_free, (unsigned)heap_min, (unsigned)heap_lfb);

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

    /* network */
    off = web_append(buf, BUF_SZ, off,
        "\"link_up\":%s,"
        "\"l2tap_ready\":%s,"
        "\"ip\":\"%s\",",
        link_up     ? "true" : "false",
        l2tap_ready ? "true" : "false",
        ip_str);

    if (have_ip)
        off = web_append(buf, BUF_SZ, off, "\"ip_source\":\"%s\",", g_static_applied ? "static" : "DHCP");
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
        g_promiscuous ? "true" : "false",
        wdt_state_str, wdt_alerts);

    if (wdt_last_s > 0)
        off = web_append(buf, BUF_SZ, off, "\"last_alert_uptime_s\":%" PRIu32 "},", wdt_last_s);
    else
        off = web_append(buf, BUF_SZ, off, "\"last_alert_uptime_s\":null},");

    /* PTP role / identity */
    off = web_append(buf, BUF_SZ, off, "\"clock_id\":\"%s\",", clk_str);

    if (gm_clkid_known)
        off = web_append(buf, BUF_SZ, off, "\"gm_clock_id\":\"%s\",", gm_clkid_str);
    else
        off = web_append(buf, BUF_SZ, off, "\"gm_clock_id\":null,");

    if (sync_src_known)
        off = web_append(buf, BUF_SZ, off, "\"sync_src_id\":\"%s\",", sync_src_str);
    else
        off = web_append(buf, BUF_SZ, off, "\"sync_src_id\":null,");
    off = web_append(buf, BUF_SZ, off, "\"sync_src_changes\":%" PRIu32 ",", sync_src_changes);

    /* GM properties — emit nulls when no Announce has arrived */
    if (gp.valid)
    {
        off = web_append(buf, BUF_SZ, off,
            "\"gm_class\":%u,"
            "\"gm_accuracy\":%u,"
            "\"gm_priority1\":%u,"
            "\"gm_priority2\":%u,"
            "\"gm_variance\":%u,"
            "\"gm_utc_offset\":%d,"
            "\"gm_time_source\":%u,"
            "\"gm_steps_removed\":%u,"
            "\"gm_flags\":{\"ptp_timescale\":%s,\"utc_offset_valid\":%s,"
                         "\"time_traceable\":%s,\"freq_traceable\":%s,"
                         "\"leap61\":%s,\"leap59\":%s},"
            "\"gm_class_transitions\":%" PRIu32 ",",
            (unsigned)gp.clock_class,
            (unsigned)gp.accuracy,
            (unsigned)gp.priority1,
            (unsigned)gp.priority2,
            (unsigned)gp.variance,
            (int)gp.utc_offset,
            (unsigned)gp.time_source,
            (unsigned)gp.steps_removed,
            (gp.announce_flags & PTP_FLAG_PTP_TIMESCALE)    ? "true" : "false",
            (gp.announce_flags & PTP_FLAG_UTC_OFFSET_VALID) ? "true" : "false",
            (gp.announce_flags & PTP_FLAG_TIME_TRACEABLE)   ? "true" : "false",
            (gp.announce_flags & PTP_FLAG_FREQ_TRACEABLE)   ? "true" : "false",
            (gp.announce_flags & PTP_FLAG_LEAP61)           ? "true" : "false",
            (gp.announce_flags & PTP_FLAG_LEAP59)           ? "true" : "false",
            gp.class_transition_count);
    }
    else
    {
        off = web_append(buf, BUF_SZ, off,
            "\"gm_class\":null,"
            "\"gm_accuracy\":null,"
            "\"gm_priority1\":null,"
            "\"gm_priority2\":null,"
            "\"gm_variance\":null,"
            "\"gm_utc_offset\":null,"
            "\"gm_time_source\":null,"
            "\"gm_steps_removed\":null,"
            "\"gm_flags\":null,"
            "\"gm_class_transitions\":0,");
    }

    /* Servo state + offsets + windows */
    off = web_append(buf, BUF_SZ, off,
        "\"servo_state\":\"%s\","
        "\"locked_for_s\":%lld,"
        "\"offset_ns\":%lld,"
        "\"path_delay_ns\":%lld,"
        "\"freq_ppb\":%.2f,"
        "\"linreg_slope_ppb\":%.2f,"
        "\"last_sync_age_ms\":%lld,"
        "\"offset_window\":%d,"
        "\"offset_mean_ns\":%.1f,"
        "\"offset_sd_ns\":%.1f,"
        "\"delay_window\":%d,"
        "\"delay_mean_ns\":%.1f,"
        "\"delay_sd_ns\":%.1f,"
        "\"quality_pct\":%d,"
        "\"quality_loose_pct\":%d,"
        "\"asymmetry_n\":%d,"
        "\"asymmetry_ns\":%lld,"
        "\"corr_m2s_ns\":%lld,"
        "\"corr_s2m_ns\":%lld,",
        web_state_str(st),
        (long long)locked_for_s,
        (long long)offset_ns,
        (long long)path_delay,
        freq_ppb,
        slope_ppb,
        (long long)last_sync_age_ms,
        off_n, off_mean, off_sd,
        dly_n, dly_mean, dly_sd,
        qual_pct, qual_loose_pct,
        asy_n, (long long)asy_mean_ns,
        (long long)corr_m2s_mean_ns, (long long)corr_s2m_mean_ns);

    /* GM diagnostics — message counters, gap/orphan counts, timing windows */
    off = web_append(buf, BUF_SZ, off,
        "\"sync_rx_count\":%" PRIu32 ","
        "\"followup_rx_count\":%" PRIu32 ","
        "\"delay_resp_rx_count\":%" PRIu32 ","
        "\"announce_rx_count\":%" PRIu32 ","
        "\"sync_seq_gaps\":%" PRIu32 ","
        "\"announce_seq_gaps\":%" PRIu32 ","
        "\"followup_orphans\":%" PRIu32 ","
        "\"delay_resp_orphans\":%" PRIu32 ","
        "\"one_step_count\":%" PRIu32 ","
        "\"gm_identity_changes\":%" PRIu32 ","
        "\"post_lock_spikes\":%" PRIu32 ","
        "\"sync_ivl_window\":%d,"
        "\"sync_ivl_mean_us\":%.0f,"
        "\"sync_ivl_sd_us\":%.0f,"
        "\"fu_latency_window\":%d,"
        "\"fu_latency_mean_us\":%.0f,"
        "\"fu_latency_sd_us\":%.0f,"
        "\"delay_req_drops\":%" PRIu32 ","
        "\"reject_total\":%" PRIu32 ","
        "\"reject_streak\":%" PRIu32 ",",
        cnt_sync, cnt_fu, cnt_dr, cnt_ann,
        gap_sync, gap_ann,
        orph_fu, orph_dr,
        one_step,
        gm_id_chg, post_lock_sp,
        sj_n, sj_mean, sj_sd,
        fl_n, fl_mean, fl_sd,
        atomic_load(&g_delay_req_drops),
        g_slave_total_rejects,
        g_slave_consecutive_rejects);

    off = web_append(buf, BUF_SZ, off, "%s,", g_config_json);

    off = web_append(buf, BUF_SZ, off, "\"tasks\":[");

    /* Per-task block */
    bool first = true;
    for (UBaseType_t i = 0; i < got && off < BUF_SZ - 200; i++)
    {
        const TaskStatus_t *t = &tarr[i];
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
 *  GET /api/health  --  lightweight JSON for load / soak monitoring
 *
 *  Same shape as the master endpoint. The "sync_jitter" block surfaces the
 *  spacing between consecutive Sync RX events (from gm_diag.sync_interval_window)
 *  instead of the master's Sync TX cadence — same physical quantity, opposite
 *  direction of the link.
 *
 *  cpu_pct requires CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS. If stats are off,
 *  runtime_stats:false and every cpu_pct is null — all other fields still work.
 * --------------------------------------------------------------------------- */
#if (configGENERATE_RUN_TIME_STATS == 1)
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

    /* Sync-RX interval jitter (µs) from gm_diag.sync_interval_window. Snapshot
     * under the gm_diag spinlock; mirror what the master exposes for TX. */
    int64_t sj_win[GM_DIAG_WINDOW];
    int sj_n;
    uint32_t sj_total;
    portENTER_CRITICAL(&g_gm_diag_lock);
    sj_n     = g_gm_diag.sync_int_count;
    sj_total = g_gm_diag.sync_rx_count;        /* lifetime Sync RX count */
    memcpy(sj_win, g_gm_diag.sync_interval_window, sizeof(int64_t) * sj_n);
    portEXIT_CRITICAL(&g_gm_diag_lock);

    int64_t sj_min = 0, sj_max = 0, sj_last = 0;
    int64_t sj_sum = 0;
    if (sj_n > 0)
    {
        sj_min = INT64_MAX; sj_max = INT64_MIN;
        for (int i = 0; i < sj_n; i++)
        {
            int64_t v = sj_win[i];
            if (v < sj_min) sj_min = v;
            if (v > sj_max) sj_max = v;
            sj_sum += v;
        }
        sj_last = sj_win[(g_gm_diag.sync_int_idx + GM_DIAG_WINDOW - 1) % GM_DIAG_WINDOW];
    }
    int64_t sj_mean = (sj_n > 0) ? (sj_sum / sj_n) : 0;

    /* FreeRTOS task table + run-time totals for CPU% */
    UBaseType_t n_tasks = uxTaskGetNumberOfTasks();
    if (n_tasks > 40) n_tasks = 40;
    TaskStatus_t *tarr = (TaskStatus_t *)calloc(n_tasks, sizeof(TaskStatus_t));
    UBaseType_t got = 0;
    uint32_t total_run = 0;
    if (tarr) got = uxTaskGetSystemState(tarr, n_tasks, &total_run);

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

    off += snprintf(buf + off, BUF_SZ - off,
        "{"
        "\"uptime_s\":%lld,"
        "\"runtime_stats\":%s,"
        "\"heap\":{\"free\":%u,\"min\":%u,\"lfb\":%u},"
        "\"sync_jitter\":{\"nominal_us\":1000000,\"window\":%d,\"count\":%" PRIu32 ","
            "\"last_us\":%lld,\"min_us\":%lld,\"max_us\":%lld,\"mean_us\":%lld,\"p2p_us\":%lld},"
        "\"tasks\":[",
        (long long)uptime_s,
        have_cpu ? "true" : "false",
        (unsigned)heap_free, (unsigned)heap_min, (unsigned)heap_lfb,
        sj_n, sj_total,
        (long long)sj_last, (long long)sj_min, (long long)sj_max, (long long)sj_mean,
        (long long)(sj_n ? (sj_max - sj_min) : 0));

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
            off += snprintf(buf + off, BUF_SZ - off,
                "%s{\"name\":\"%s\",\"prio\":%u,\"stack_hwm\":%" PRIu32 ",\"cpu_pct\":null}",
                first ? "" : ",", name, (unsigned)t->uxCurrentPriority, hwm_bytes);
        else
            off += snprintf(buf + off, BUF_SZ - off,
                "%s{\"name\":\"%s\",\"prio\":%u,\"stack_hwm\":%" PRIu32 ",\"cpu_pct\":%.1f}",
                first ? "" : ",", name, (unsigned)t->uxCurrentPriority, hwm_bytes, cpu);
        first = false;
    }

#if (configGENERATE_RUN_TIME_STATS == 1)
    s_cpu_prev_total = total_run;
#endif

    off += snprintf(buf + off, BUF_SZ - off,
        "],\"rx_task\":{\"stack_size\":8192,\"stack_hwm\":%" PRIu32 ",", rx_hwm);
    if (rx_cpu < 0.0)
        off += snprintf(buf + off, BUF_SZ - off, "\"cpu_pct\":null}}");
    else
        off += snprintf(buf + off, BUF_SZ - off, "\"cpu_pct\":%.1f}}", rx_cpu);

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
 * --------------------------------------------------------------------------- */
static httpd_handle_t web_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port       = 80;
    config.ctrl_port         = 32768;
    config.max_open_sockets  = 4;
    config.max_uri_handlers  = 8;
    config.stack_size        = 8192;        /* OTA needs slightly more stack */
    config.lru_purge_enable  = true;
    config.recv_wait_timeout = 30;          /* OTA: slow uploaders */
    config.send_wait_timeout = 10;

    /* "httpd" never goes through task_create_tracked (esp_http_server owns
     * its creation) — register its stack size directly so the dashboard's
     * task table doesn't show it with a null stack. */
    task_reg_add("httpd", config.stack_size);

    ESP_LOGI(WEB_TAG, "Starting HTTP server on port %d", config.server_port);
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(WEB_TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return NULL;
    }

    static const httpd_uri_t uri_root = {
        .uri = "/",             .method = HTTP_GET,  .handler = root_handler,       .user_ctx = NULL };
    static const httpd_uri_t uri_status = {
        .uri = "/api/status",   .method = HTTP_GET,  .handler = status_handler,     .user_ctx = NULL };
    static const httpd_uri_t uri_health = {
        .uri = "/api/health",   .method = HTTP_GET,  .handler = health_handler,     .user_ctx = NULL };
    static const httpd_uri_t uri_ota_page = {
        .uri = "/ota",          .method = HTTP_GET,  .handler = ota_page_handler,   .user_ctx = NULL };
    static const httpd_uri_t uri_ota_info = {
        .uri = "/api/ota/info", .method = HTTP_GET,  .handler = ota_info_handler,   .user_ctx = NULL };
    static const httpd_uri_t uri_ota_upload = {
        .uri = "/api/ota",      .method = HTTP_POST, .handler = ota_upload_handler, .user_ctx = NULL };
    static const httpd_uri_t uri_reboot = {
        .uri = "/api/reboot",   .method = HTTP_POST, .handler = reboot_handler,     .user_ctx = NULL };

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_status);
    httpd_register_uri_handler(server, &uri_health);
    httpd_register_uri_handler(server, &uri_ota_page);
    httpd_register_uri_handler(server, &uri_ota_info);
    httpd_register_uri_handler(server, &uri_ota_upload);
    httpd_register_uri_handler(server, &uri_reboot);

    ESP_LOGI(WEB_TAG, "HTTP server ready -- open http://<device-ip>/ in a browser");
    ESP_LOGI(WEB_TAG, "  GET  /            -> slave status page");
    ESP_LOGI(WEB_TAG, "  POST /api/reboot  -> restart (OTA credentials)");
    ESP_LOGI(WEB_TAG, "  GET  /api/status  -> slave status JSON");
    ESP_LOGI(WEB_TAG, "  GET  /api/health  -> load/soak health JSON (heap/cpu/stack/sync_jitter)");
    ESP_LOGI(WEB_TAG, "  GET  /ota         -> OTA upload page (Basic-Auth)");
    ESP_LOGI(WEB_TAG, "  GET  /api/ota/info-> OTA partition info JSON");
    ESP_LOGI(WEB_TAG, "  POST /api/ota     -> firmware image upload");
    return server;
}

#endif /* WEB_SERVER_H */
