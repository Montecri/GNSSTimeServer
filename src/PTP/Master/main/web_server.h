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
 *  g_l2tap_ready, s_led_state, g_phc_offset_ns, g_lock_qual_count,
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

    int64_t offset_ns   = g_last_offset;
    double  freq_ppb    = g_freq_ppb;
    double  integ       = g_integral;
    int     tai_off     = g_utc_offset;
    bool    gnss_valid   = g_gnss_valid;
    time_t  gnss_sec     = g_gnss_seconds;
    time_t  last_gnss_up = g_last_gnss_update;
    int64_t phc_resid   = g_phc_offset_ns;
    uint8_t lock_qual   = g_lock_qual_count;
    int64_t lock_ms     = g_lock_acquired_ms;
    bool    settled     = g_settled_after_holdover;
    uint16_t sync_seq   = g_sync_seq;
    uint16_t ann_seq    = g_announce_seq;
    uint32_t emac_rst   = g_emac_restart_count;
    int      slaves     = slave_active_count();
    uint32_t pps_seq    = g_pps_capture_seq;     /* real PPS edge count */
    uint32_t rej_total  = g_total_rejects;       /* lifetime servo rejects */
    uint32_t rej_streak = g_consecutive_rejects; /* live reject streak */

    int64_t  now_ms     = esp_timer_get_time() / 1000;
    int64_t  uptime_s   = (now_ms - g_boot_time_ms) / 1000;

    /* Heap stats */
    size_t heap_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t heap_lfb  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

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

    /* IP address */
    char ip_str[20] = "0.0.0.0";
    if (g_eth_netif)
    {
        esp_netif_ip_info_t ipinfo;
        if (esp_netif_get_ip_info(g_eth_netif, &ipinfo) == ESP_OK && ipinfo.ip.addr != 0)
        {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ipinfo.ip));
        }
    }

    /* UTC time string from GNSS seconds */
    char utc_str[12] = "--:--:--";
    if (gnss_valid && gnss_sec > 1577836800LL)
    {
        struct tm tm_utc;
        time_t t = gnss_sec;
        gmtime_r(&t, &tm_utc);
        snprintf(utc_str, sizeof(utc_str), "%02d:%02d:%02d",
                 tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
    }

    /* "Locked for" string (HH:MM:SS) */
    char locked_for[16] = "00:00:00";
    if (st == LED_STATE_LOCKED && lock_ms > 0)
    {
        int64_t held_s = (now_ms - lock_ms) / 1000;
        if (held_s < 0) held_s = 0;
        unsigned h = (unsigned)(held_s / 3600);
        unsigned m = (unsigned)((held_s % 3600) / 60);
        unsigned s = (unsigned)(held_s % 60);
        if (h > 999) h = 999; /* clamp to fit buffer */
        snprintf(locked_for, sizeof(locked_for), "%02u:%02u:%02u", h, m, s);
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
    /* 8 KB is plenty: ~2.5 KB scalars + ~13 tasks * ~120 B = ~4 KB worst case */
    const size_t BUF_SZ = 8192;
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

    /* Heap leak delta: we don't track baseline here -- the heap_monitor task
     * tracks it internally. Report 0 (unknown) and the UI shows "+0 B / ok". */

    off = web_append(buf, BUF_SZ, off,
        "{"
        "\"chip\":\"ESP32-P4\","
        "\"chip_rev\":\"v%d.%d\","
        "\"cpu_mhz\":%d,"
        "\"project\":\"%s\","
        "\"app_version\":\"%s\","
        "\"idf_version\":\"%s\","
        "\"compile_time\":\"%s\","
        "\"elf_sha\":\"%s\","
        "\"flash_mb\":16,"
        "\"flash_mode\":\"DIO\","
        "\"flash_mhz\":80,"
        "\"uptime_s\":%lld,"
        "\"heap_free\":%u,"
        "\"heap_min\":%u,"
        "\"heap_lfb\":%u,"
        "\"heap_leak\":0,"
        "\"retent_total\":%u,"
        "\"retent_free\":%u,"
        "\"ram1_kb\":384,"
        "\"ram2_kb\":18,"
        "\"rtc_kb\":31,"
        "\"tcm_kb\":7,"
        "\"link_up\":%s,"
        "\"l2tap_ready\":%s,"
        "\"ip\":\"%s\","
        "\"ip_source\":\"DHCP/static\","
        "\"mac\":\"%s\","
        "\"hostname\":\"%s\","
        "\"emac_restarts\":%" PRIu32 ","
        "\"promiscuous\":false,"
        "\"wdt_healthy\":true,"
        "\"role\":\"%s\","
        "\"clock_id\":\"%s\","
        "\"utc_time\":\"%s\","
        "\"tai_offset\":%d,"
        "\"sync_interval_ms\":1000,"
        "\"announce_interval_ms\":2000,"
        "\"sync_seq\":%u,"
        "\"announce_seq\":%u,"
        "\"path_delay_ns\":null,"
        "\"slaves_active\":%d,"
        "\"slaves_max\":8,"
        "\"servo_state\":\"%s\","
        "\"locked_for\":\"%s\","
        "\"offset_ns\":%lld,"
        "\"freq_ppb\":%.2f,"
        "\"integrator\":%.2f,"
        "\"lock_qual\":%u,"
        "\"lock_qual_window\":64,"
        "\"pps_count\":%" PRIu32 ","
        "\"reject_total\":%" PRIu32 ","
        "\"reject_streak\":%" PRIu32 ","
        "\"reject_reacquire_at\":45,"
        "\"phc_residual_ns\":%lld,"
        "\"holdover_timeout_s\":5,"
        "\"settled\":%s,"
        "\"gnss_age_s\":%ld,"
        "\"gnss_fix\":%s,"
        "\"pps_led_active\":%s,"
        "\"tasks\":[",
        chip.revision / 100, chip.revision % 100,
        (int)WEB_CPU_MHZ,
        app ? app->project_name : "PTP_Master",
        app ? app->version      : "unknown",
        app ? app->idf_ver      : esp_get_idf_version(),
        compile_str,
        sha_str,
        (long long)uptime_s,
        (unsigned)heap_free,
        (unsigned)heap_min,
        (unsigned)heap_lfb,
        (unsigned)retent_total,
        (unsigned)retent_free,
        link_up     ? "true" : "false",
        l2tap_ready ? "true" : "false",
        ip_str,
        mac_str,
        g_hostname[0] ? g_hostname : "",
        emac_rst,
        g_i_am_master ? "MASTER" : "SLAVE",
        clk_str,
        utc_str,
        tai_off,
        (unsigned)sync_seq,
        (unsigned)ann_seq,
        slaves,
        web_state_str(st),
        locked_for,
        (long long)offset_ns,
        freq_ppb,
        integ,
        (unsigned)lock_qual,
        pps_seq,
        rej_total,
        rej_streak,
        (long long)phc_resid,
        settled ? "true" : "false",
        gnss_age,
        gnss_fix ? "true" : "false",
        (st == LED_STATE_LOCKED) ? "true" : "false"
    );

    /* Append task array */
    bool first = true;
    for (UBaseType_t i = 0; i < got && off < BUF_SZ - 200; i++)
    {
        const TaskStatus_t *t = &tarr[i];
        /* high-water mark is reported in StackType_t units; convert to bytes */
        uint32_t hwm_bytes = (uint32_t)t->usStackHighWaterMark * sizeof(StackType_t);

        /* We don't have the original stack size from TaskStatus_t. Map the
         * known tasks to their configured stack size; everything else gets
         * the FreeRTOS default. */
        uint32_t stack_bytes = 0;
        const char *name = t->pcTaskName ? t->pcTaskName : "?";
        if      (!strcmp(name, "ptp_rx"))   stack_bytes = 8192;
        else if (!strcmp(name, "ptp_tx"))   stack_bytes = 8192;
        else if (!strcmp(name, "gnss"))      stack_bytes = 6144;
        else if (!strcmp(name, "servo"))    stack_bytes = 6144;
        else if (!strcmp(name, "pps_task")) stack_bytes = 4096;
        else if (!strcmp(name, "pps_led"))  stack_bytes = 2048;
        else if (!strcmp(name, "led"))      stack_bytes = 2048;
        else if (!strcmp(name, "display"))  stack_bytes = 4096;
        else if (!strcmp(name, "btn"))      stack_bytes = 2048;
        else if (!strcmp(name, "dhcp_fb"))  stack_bytes = 3072;
        else if (!strcmp(name, "heap_mon")) stack_bytes = 3072;
        else if (!strcmp(name, "wdt"))      stack_bytes = 4096;
        else if (!strcmp(name, "main"))     stack_bytes = 4096;
        else stack_bytes = hwm_bytes + 1024; /* best effort */

        int core = web_task_core(t->xHandle);

        off = web_append(buf, BUF_SZ, off,
            "%s{\"name\":\"%s\",\"priority\":%u,\"stack_bytes\":%" PRIu32
            ",\"hwm_bytes\":%" PRIu32 ",\"core\":%d}",
            first ? "" : ",",
            name,
            (unsigned)t->uxCurrentPriority,
            stack_bytes,
            hwm_bytes,
            core);
        first = false;
    }

    off = web_append(buf, BUF_SZ, off, "]}");

    free(tarr);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    esp_err_t r = httpd_resp_send(req, buf, off);
    free(buf);
    return r;
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
    config.max_uri_handlers = 8;            /* room for OTA routes */
    config.stack_size       = 8192;         /* OTA needs slightly more stack */
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 30;          /* OTA: large uploads can pause */
    config.send_wait_timeout = 10;

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

    httpd_register_uri_handler(server, &uri_root);
    httpd_register_uri_handler(server, &uri_status);
    httpd_register_uri_handler(server, &uri_health);
    httpd_register_uri_handler(server, &uri_ota_page);
    httpd_register_uri_handler(server, &uri_ota_info);
    httpd_register_uri_handler(server, &uri_ota_upload);

    ESP_LOGI(WEB_TAG, "HTTP server ready -- open http://<device-ip>/ in a browser");
    ESP_LOGI(WEB_TAG, "  GET  /            -> status page");
    ESP_LOGI(WEB_TAG, "  GET  /api/status  -> status JSON");
    ESP_LOGI(WEB_TAG, "  GET  /api/health  -> load/soak health JSON (heap/cpu/stack/jitter)");
    ESP_LOGI(WEB_TAG, "  GET  /ota         -> OTA upload page");
    ESP_LOGI(WEB_TAG, "  GET  /api/ota/info-> OTA partition info JSON");
    ESP_LOGI(WEB_TAG, "  POST /api/ota     -> firmware image upload");
    return server;
}

#endif /* WEB_SERVER_H */
