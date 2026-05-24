/**
 * @file allystar_timing.c
 *
 * TAU1201 timing-mode configurator. See allystar_timing.h for context.
 *
 * Every CFG message ID and payload layout below has been cross-referenced
 * against the ALLYSTAR GNSS Receiver Protocol Specification V2.3:
 *   - Frame structure & checksum:  §3
 *   - ACK / NAK:                    §5.3
 *   - CFG-PPS:                      §5.4.3 (Cynosure II/III, 15-byte form)
 *   - CFG-DOP:                      §5.4.5
 *   - CFG-ELEV:                     §5.4.6
 *   - CFG-NAVSAT:                   §5.4.7
 *   - CFG-SPDHOLD:                  §5.4.10
 *   - CFG-SURVEY:                   §5.4.13 (HD9300/HD9400 series)
 *   - CFG-CARRSMOOTH:               §5.4.18
 *   - NAV-POSLLH (poll & response): §5.1.2
 *
 * NOTE on CFG-SURVEY: the spec marks it "HD9300/HD9400 series only". The
 * TAU1201 is part of that family per the datasheet; if your specific module
 * NAKs CFG-SURVEY, the error callback will fire with step name "CFG-SURVEY"
 * and you'll need to fall back to CFG-FIXEDLLA / CFG-FIXEDECEF with surveyed
 * coordinates as firmware constants.
 *
 * Survey completion strategy:
 *   The spec exposes no progress sentence for the survey. We poll NAV-POSLLH
 *   periodically and consider the survey "good enough" when either:
 *     (a) the configured 15-minute timer expires, OR
 *     (b) >= 5 minutes have elapsed AND hAcc <= SURVEY_HACC_TARGET_MM.
 *   The module continues converging internally regardless; this just lets us
 *   stop waiting and proceed with the rest of the boot sequence.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "allystar_timing.h"

static const char *TAG = "ALLYSTAR_TIMING";

/* ============================ Configuration ============================== */

#define EX_UART_NUM                     UART_NUM_1
#define STREAM_BUF_BYTES                (2048)

/* Survey */
#define SURVEY_MAX_SECONDS  ALLYSTAR_SURVEY_MAX_SECONDS
#define SURVEY_MIN_SECONDS              (5 * 60)    /* earliest early-exit   */
#define SURVEY_HACC_TARGET_MM           (200)       /* 20 cm horizontal acc  */
#define SURVEY_POLL_PERIOD_MS           (5000)      /* NAV-POSLLH poll rate  */
#define SURVEY_ACCLIMIT_MM              (100)       /* tells module its goal */

/* 3D fix wait (boot timeout). Each tick of the wait loop is 100 ms below. */
#define FIX_WAIT_TIMEOUT_TICKS          (5 * 60 * 10)   /* 5 minutes         */
#define FIX_STABLE_COUNT                (10)            /* consecutive GGAs  */

/* PPS */
#define PPS_PERIOD_US                   (1000000UL)     /* 1 Hz              */
#define PPS_PULSE_WIDTH_US              (100000UL)      /* 100 ms high       */
#define PPS_OFFSET_NS                   (25)            /* cable delay comp  */
#define PPS_POLARITY                    (1)             /* 1 = rising edge   */
#define PPS_GPIO                        (0)             /* module-default    */
#define PPS_SYNC_KEEP_WHEN_UNFIXED      (1)             /* 1 = always tick   */

/* NAVSAT enable mask. Spec §5.4.7:
 *   GPS L1     = 0x00000001
 *   GLONASS G1 = 0x00000002
 *   BEIDOU B1  = 0x00000004
 *   GALILEO E1 = 0x00000010
 *   QZSS L1    = 0x00000020
 *   SBAS L1    = 0x00000040
 * Default: all four major constellations on L1. */

/* TEMPORARY DEBUGGING VALUES — loosen until fix is stable, then tighten */
#define NAVSAT_ENABLE_MASK    (0xFFFFFFFFUL)   /* everything the module supports */
#define ELEV_TRK_RAD          (0.0f)           /* 0° — track anything */
#define ELEV_NAV_RAD          (0.0f)           /* 0° — use anything in nav */
#define DOP_PDOP_RAW          (5000)           /* pDOP 50 (very loose) */
#define DOP_TDOP_RAW          (5000)           /* tDOP 50 */

/* Static-hold speed. Spec §5.4.10 example shows raw value = cm/s directly
 * despite "scale 0.01" annotation. 10 = 10 cm/s = 0.1 m/s. */
#define SPDHOLD_RAW_CMS                 (10)

/* Carrier smoothing. Spec §5.4.18:
 *   -1 = auto, 0 = disable, >=1 = window (x+1). */
#define CARRSMOOTH_WINDOWS              (-1)            /* auto              */

/* ACK behaviour */
#define MAX_COMMAND_RETRIES             (3)
#define ACK_TIMEOUT_MS                  (300)

/* ============================ Protocol bytes ============================= */

#define UBP_SYNC1   0xF1
#define UBP_SYNC2   0xD9                    /* per spec §3 */

#define CLS_NAV     0x01
#define CLS_ACK     0x05
#define CLS_CFG     0x06

#define ACK_ACK     0x01
#define ACK_NAK     0x00

#define NAV_POSLLH      0x02
#define CFG_PPS         0x07
#define CFG_DOP         0x0A
#define CFG_ELEV        0x0B
#define CFG_NAVSAT      0x0C
#define CFG_SPDHOLD     0x0F
#define CFG_SURVEY      0x12
#define CFG_CARRSMOOTH  0x17

/* ============================ Step names ================================= */

static const char *STEP_3D_FIX       = "3D-FIX";
static const char *STEP_CFG_NAVSAT   = "CFG-NAVSAT";
static const char *STEP_CFG_ELEV     = "CFG-ELEV";
static const char *STEP_CFG_DOP      = "CFG-DOP";
static const char *STEP_CFG_SPDHOLD  = "CFG-SPDHOLD";
static const char *STEP_CFG_CARRSM   = "CFG-CARRSMOOTH";
//static const char *STEP_CFG_PPS      = "CFG-PPS";
static const char *STEP_CFG_SURVEY   = "CFG-SURVEY";

/* ============================ Module state =============================== */

volatile bool       initialization_sequence_done = false;
const char *volatile g_timing_error_step          = NULL;
volatile bool       g_timing_config_ok            = false;

static SemaphoreHandle_t   ack_sem        = NULL;
static SemaphoreHandle_t   uart_tx_mutex  = NULL;
static StreamBufferHandle_t timing_stream = NULL;
static allystar_timing_error_cb_t app_error_cb = NULL;
static TaskHandle_t        timing_task_handle = NULL;

/* Updated by the RX parser when an ACK/NAK is seen. Read by the synchronous
 * send-with-ack helper. Group/Sub of the acknowledged message live in
 * last_ack_group/sub so we can verify it matches what we just sent. */
static volatile bool    last_cmd_acked = false;
static volatile uint8_t last_ack_group = 0;
static volatile uint8_t last_ack_sub   = 0;

/* Updated by the RX parser when a NAV-POSLLH response is seen. */
static volatile bool     last_posllh_valid = false;
static volatile uint32_t last_posllh_hacc_mm = 0xFFFFFFFFu;

/* ============================ Frame builder ============================== */

static void calculate_ubp_checksum(const uint8_t *packet, uint16_t length,
                                   uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    /* Per spec §3.1.5: from start of Message ID (idx 2) through last payload
     * byte (idx length-3). The last two bytes are the checksum itself. */
    for (uint16_t i = 2; i < (uint16_t)(length - 2); i++) {
        a = (uint8_t)(a + packet[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

/* Build and transmit a single binary frame. Returns true on TX success. */
static bool send_ubp_packet(uint8_t group_id, uint8_t sub_id,
                            const uint8_t *payload, uint16_t payload_len)
{
    const uint16_t packet_size = (uint16_t)(6 + payload_len + 2);
    uint8_t *packet = malloc(packet_size);
    if (!packet) {
        ESP_LOGE(TAG, "send_ubp_packet: malloc failed");
        return false;
    }

    packet[0] = UBP_SYNC1;
    packet[1] = UBP_SYNC2;
    packet[2] = group_id;
    packet[3] = sub_id;
    packet[4] = (uint8_t)(payload_len & 0xFF);
    packet[5] = (uint8_t)((payload_len >> 8) & 0xFF);

    if (payload_len > 0 && payload != NULL) {
        memcpy(&packet[6], payload, payload_len);
    }

    uint8_t ck_a = 0, ck_b = 0;
    calculate_ubp_checksum(packet, packet_size, &ck_a, &ck_b);
    packet[packet_size - 2] = ck_a;
    packet[packet_size - 1] = ck_b;

    bool ok = false;
    if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int written = uart_write_bytes(EX_UART_NUM, (const char *)packet, packet_size);
        xSemaphoreGive(uart_tx_mutex);
        ok = (written == (int)packet_size);
    } else {
        ESP_LOGE(TAG, "send_ubp_packet: UART mutex timeout");
    }

    free(packet);
    return ok;
}

/* ============================ RX parser ================================== */

/* The parser is fed one byte at a time and recognises ALLYSTAR binary frames
 * embedded in the larger UART stream (which is mostly NMEA). NMEA text is
 * silently discarded; we only care about ACK/NAK and NAV-POSLLH responses.
 *
 * Frame layout we care about:
 *   F1 D9 | GROUP | SUB | LEN_LO LEN_HI | PAYLOAD[LEN] | CK_A CK_B
 *
 * State machine:
 *   0  scan for F1
 *   1  expect D9
 *   2  group
 *   3  sub
 *   4  len low
 *   5  len high
 *   6  collecting payload (idx < len)
 *   7  CK_A
 *   8  CK_B → dispatch
 */

#define PARSER_MAX_PAYLOAD  256

static struct {
    uint8_t  state;
    uint8_t  group;
    uint8_t  sub;
    uint16_t len;
    uint16_t idx;
    uint8_t  payload[PARSER_MAX_PAYLOAD];
    uint8_t  ck_a_calc;
    uint8_t  ck_b_calc;
    uint8_t  ck_a_rx;
} parser;

static void parser_reset(void)
{
    memset(&parser, 0, sizeof(parser));
}

static void parser_dispatch_frame(void)
{
    /* Checksum already verified by caller (state 8). Just dispatch. */
    if (parser.group == CLS_ACK && parser.len == 2) {
        last_ack_group = parser.payload[0];
        last_ack_sub   = parser.payload[1];
        last_cmd_acked = (parser.sub == ACK_ACK);
        xSemaphoreGive(ack_sem);
    } else if (parser.group == CLS_NAV && parser.sub == NAV_POSLLH
               && parser.len == 28) {
        /* Payload §5.1.2:
         *   0  U4 iTow
         *   4  S4 lon (1e-7 deg)
         *   8  S4 lat (1e-7 deg)
         *  12  S4 height (mm)
         *  16  S4 hMSL  (mm)
         *  20  U4 hAcc  (mm)   ← what we care about
         *  24  U4 vAcc  (mm)
         */
        uint32_t hacc = ((uint32_t)parser.payload[20])
                      | ((uint32_t)parser.payload[21] << 8)
                      | ((uint32_t)parser.payload[22] << 16)
                      | ((uint32_t)parser.payload[23] << 24);
        last_posllh_hacc_mm = hacc;
        last_posllh_valid   = true;
    }
}

static void parser_feed_byte(uint8_t b)
{
    switch (parser.state) {
    case 0:
        if (b == UBP_SYNC1) parser.state = 1;
        break;
    case 1:
        parser.state = (b == UBP_SYNC2) ? 2 : (b == UBP_SYNC1 ? 1 : 0);
        break;
    case 2:
        parser.group = b;
        parser.state = 3;
        break;
    case 3:
        parser.sub = b;
        parser.state = 4;
        break;
    case 4:
        parser.len = b;
        parser.state = 5;
        break;
    case 5:
        parser.len |= (uint16_t)b << 8;
        parser.idx = 0;
        if (parser.len == 0) {
            parser.state = 7;
        } else if (parser.len <= PARSER_MAX_PAYLOAD) {
            parser.state = 6;
        } else {
            /* oversize — drop and resync */
            parser.state = 0;
        }
        break;
    case 6:
        parser.payload[parser.idx++] = b;
        if (parser.idx >= parser.len) parser.state = 7;
        break;
    case 7:
        parser.ck_a_rx = b;
        parser.state = 8;
        break;
    case 8: {
        /* Compute expected checksum and compare. */
        uint8_t a = 0, bb = 0;
        a  = (uint8_t)(a + parser.group); bb = (uint8_t)(bb + a);
        a  = (uint8_t)(a + parser.sub);   bb = (uint8_t)(bb + a);
        a  = (uint8_t)(a + (parser.len & 0xFF));        bb = (uint8_t)(bb + a);
        a  = (uint8_t)(a + ((parser.len >> 8) & 0xFF)); bb = (uint8_t)(bb + a);
        for (uint16_t i = 0; i < parser.len; i++) {
            a  = (uint8_t)(a + parser.payload[i]);
            bb = (uint8_t)(bb + a);
        }
        if (a == parser.ck_a_rx && bb == b) {
            parser_dispatch_frame();
        }
        parser.state = 0;
        break;
    }
    default:
        parser.state = 0;
        break;
    }
}

static void drain_stream_into_parser(uint32_t max_ms)
{
    /* Pull whatever's queued, plus what arrives within max_ms. */
    uint8_t buf[64];
    TickType_t start = xTaskGetTickCount();
    TickType_t deadline = start + pdMS_TO_TICKS(max_ms);
    do {
        size_t n = xStreamBufferReceive(timing_stream, buf, sizeof(buf),
                                        pdMS_TO_TICKS(20));
        for (size_t i = 0; i < n; i++) parser_feed_byte(buf[i]);
        if (n == 0) break;
    } while (xTaskGetTickCount() < deadline);
}

/* ============================ Send-with-ACK ============================== */

static bool send_cfg_with_ack(uint8_t sub, const uint8_t *payload, uint16_t len)
{
    for (int retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
        /* Drain any stale ACK that arrived before this send. */
        xSemaphoreTake(ack_sem, 0);
        last_cmd_acked = false;
        last_ack_group = 0;
        last_ack_sub   = 0;

        if (!send_ubp_packet(CLS_CFG, sub, payload, len)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Pump RX bytes through the parser until we get the ACK or timeout. */
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ACK_TIMEOUT_MS);
        while (xTaskGetTickCount() < deadline) {
            drain_stream_into_parser(20);
            if (last_ack_group == CLS_CFG && last_ack_sub == sub) {
                return last_cmd_acked;
            }
        }
        ESP_LOGW(TAG, "CFG-%02X retry %d/%d (no ACK)", sub, retry + 1,
                 MAX_COMMAND_RETRIES);
    }
    return false;
}

/* Poll NAV-POSLLH and wait briefly for the response. Updates the global
 * last_posllh_* fields. Returns true if a fresh response was seen. */
static bool poll_navposllh(uint32_t timeout_ms)
{
    last_posllh_valid = false;
    /* Poll request: empty payload, see spec §5.1.2. */
    if (!send_ubp_packet(CLS_NAV, NAV_POSLLH, NULL, 0)) return false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        drain_stream_into_parser(50);
        if (last_posllh_valid) return true;
    }
    return false;
}

/* ============================ NMEA helpers =============================== */

/* Check a single $xxGGA line for fix quality 1..5 (any 3D fix accepted). */
static bool gga_has_3d_fix(const char *line)
{
    if (!(strstr(line, "GGA"))) return false;
    /* Count commas; field 6 (0-indexed) is fix quality. */
    int commas = 0;
    for (const char *p = line; *p; p++) {
        if (*p == ',') {
            commas++;
            if (commas == 6) {
                char q = *(p + 1);
                return (q >= '1' && q <= '5');
            }
        }
    }
    return false;
}

/* ============================ Feed hook ================================== */

void allystar_timing_feed_data(const uint8_t *buffer, size_t length)
{
    if (initialization_sequence_done || timing_stream == NULL ||
        buffer == NULL || length == 0) {
        return;
    }
    /* Non-blocking; if the buffer is full we silently drop. Worst case is
     * we miss an ACK and retry. */
    xStreamBufferSend(timing_stream, buffer, length, 0);
}

/* ============================ Sequence task ============================== */

static void cleanup_after_sequence(void)
{
    /* Stop forwarding bytes. The stream buffer will be torn down. */
    initialization_sequence_done = true;
    if (timing_stream) {
        vStreamBufferDelete(timing_stream);
        timing_stream = NULL;
    }
    parser_reset();
}

static void fail_step(const char *step)
{
    ESP_LOGE(TAG, "Step failed: %s", step);
    g_timing_error_step = step;
    g_timing_config_ok  = false;
    cleanup_after_sequence();
    if (app_error_cb) app_error_cb(step);
}

/* Pull NMEA lines out of the stream buffer until we observe FIX_STABLE_COUNT
 * consecutive GGAs with a 3D fix. Returns false on timeout. */
static bool wait_for_3d_fix(void)
{
    ESP_LOGI(TAG, "Waiting for stable 3D fix...");
    char line[128];
    size_t lidx = 0;
    int stable = 0;
    uint32_t ticks = 0;

    while (ticks < FIX_WAIT_TIMEOUT_TICKS) {
        uint8_t b;
        size_t n = xStreamBufferReceive(timing_stream, &b, 1,
                                        pdMS_TO_TICKS(100));
        if (n == 0) {
            ticks++;
            continue;
        }
        /* Also feed the binary parser so we don't drop ACKs/NAV frames. */
        parser_feed_byte(b);

        if (b == '$') {
            lidx = 0;
            line[lidx++] = (char)b;
        } else if (lidx > 0) {
            if (b == '\n' || b == '\r') {
                line[lidx] = '\0';
                if (gga_has_3d_fix(line)) {
                    stable++;
                    if (stable >= FIX_STABLE_COUNT) {
                        ESP_LOGI(TAG, "3D fix stable");
                        return true;
                    }
                } else if (strstr(line, "GGA")) {
                    stable = 0;
                }
                lidx = 0;
            } else if (lidx < sizeof(line) - 1) {
                line[lidx++] = (char)b;
            } else {
                lidx = 0;
            }
        }
    }
    return false;
}

/* Run CFG-SURVEY, then loop polling NAV-POSLLH until done-enough. */
static bool run_survey(void)
{
    uint8_t payload[8];
    uint32_t mindur = SURVEY_MAX_SECONDS;
    uint32_t acclim = SURVEY_ACCLIMIT_MM;
    payload[0] = (uint8_t)(mindur & 0xFF);
    payload[1] = (uint8_t)((mindur >>  8) & 0xFF);
    payload[2] = (uint8_t)((mindur >> 16) & 0xFF);
    payload[3] = (uint8_t)((mindur >> 24) & 0xFF);
    payload[4] = (uint8_t)(acclim & 0xFF);
    payload[5] = (uint8_t)((acclim >>  8) & 0xFF);
    payload[6] = (uint8_t)((acclim >> 16) & 0xFF);
    payload[7] = (uint8_t)((acclim >> 24) & 0xFF);

    if (!send_cfg_with_ack(CFG_SURVEY, payload, sizeof(payload))) {
        return false;
    }

    ESP_LOGI(TAG, "Survey started: max %u s, target %u mm",
             (unsigned)SURVEY_MAX_SECONDS, (unsigned)SURVEY_HACC_TARGET_MM);

    int64_t t0 = esp_timer_get_time();
    uint32_t last_log = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SURVEY_POLL_PERIOD_MS));

        /* Drain whatever NMEA/binary arrived while we slept. */
        drain_stream_into_parser(50);
        poll_navposllh(500);

        uint32_t elapsed_s = (uint32_t)((esp_timer_get_time() - t0) / 1000000LL);
        uint32_t minute = elapsed_s / 60;
        if (minute != last_log) {
            ESP_LOGI(TAG, "Survey: %u min elapsed, hAcc=%u mm",
                     (unsigned)minute,
                     (unsigned)last_posllh_hacc_mm);
            last_log = minute;
        }

        bool time_up = (elapsed_s >= SURVEY_MAX_SECONDS);
        bool good_enough = (elapsed_s >= SURVEY_MIN_SECONDS) &&
                           last_posllh_valid &&
                           (last_posllh_hacc_mm <= SURVEY_HACC_TARGET_MM);
        if (time_up || good_enough) {
            ESP_LOGI(TAG, "Survey done: elapsed %u s, hAcc=%u mm (%s)",
                     (unsigned)elapsed_s,
                     (unsigned)last_posllh_hacc_mm,
                     time_up ? "timeout" : "accuracy");
            return true;
        }
    }
}

static void allystar_timing_sequence_task(void *pv)
{
    (void)pv;

    /* -------- Phase A: send config that doesn't need a fix -------- */

/* CFG-NAVSAT */
    {
        uint8_t p[4];
        p[0] = (uint8_t)(NAVSAT_ENABLE_MASK & 0xFF);
        p[1] = (uint8_t)((NAVSAT_ENABLE_MASK >>  8) & 0xFF);
        p[2] = (uint8_t)((NAVSAT_ENABLE_MASK >> 16) & 0xFF);
        p[3] = (uint8_t)((NAVSAT_ENABLE_MASK >> 24) & 0xFF);
        if (!send_cfg_with_ack(CFG_NAVSAT, p, sizeof(p))) {
            fail_step(STEP_CFG_NAVSAT); vTaskDelete(NULL); return;
        }
        ESP_LOGI(TAG, "CFG-NAVSAT OK (mask=0x%08lX)", (unsigned long)NAVSAT_ENABLE_MASK);
    }

    /* CFG-ELEV */
    {
        uint8_t p[8];
        float trk = ELEV_TRK_RAD;
        float nav = ELEV_NAV_RAD;
        memcpy(&p[0], &trk, 4);
        memcpy(&p[4], &nav, 4);
        if (!send_cfg_with_ack(CFG_ELEV, p, sizeof(p))) {
            fail_step(STEP_CFG_ELEV); vTaskDelete(NULL); return;
        }
        ESP_LOGI(TAG, "CFG-ELEV OK (trk=%.4f rad, nav=%.4f rad)", trk, nav);
    }

    /* CFG-DOP */
    {
        uint8_t p[4];
        p[0] = (uint8_t)(DOP_PDOP_RAW & 0xFF);
        p[1] = (uint8_t)((DOP_PDOP_RAW >> 8) & 0xFF);
        p[2] = (uint8_t)(DOP_TDOP_RAW & 0xFF);
        p[3] = (uint8_t)((DOP_TDOP_RAW >> 8) & 0xFF);
        if (!send_cfg_with_ack(CFG_DOP, p, sizeof(p))) {
            fail_step(STEP_CFG_DOP); vTaskDelete(NULL); return;
        }
        ESP_LOGI(TAG, "CFG-DOP OK (pDOP=%d, tDOP=%d, raw)", DOP_PDOP_RAW, DOP_TDOP_RAW);
    }

    /* CFG-SPDHOLD */
    {
        uint8_t p[2];
        p[0] = (uint8_t)(SPDHOLD_RAW_CMS & 0xFF);
        p[1] = (uint8_t)((SPDHOLD_RAW_CMS >> 8) & 0xFF);
        if (!send_cfg_with_ack(CFG_SPDHOLD, p, sizeof(p))) {
            fail_step(STEP_CFG_SPDHOLD); vTaskDelete(NULL); return;
        }
        ESP_LOGI(TAG, "CFG-SPDHOLD OK (%d cm/s)", SPDHOLD_RAW_CMS);
    }

    /* CFG-CARRSMOOTH */
    {
        int8_t v = (int8_t)CARRSMOOTH_WINDOWS;
        if (!send_cfg_with_ack(CFG_CARRSMOOTH, (uint8_t *)&v, 1)) {
            fail_step(STEP_CFG_CARRSM); vTaskDelete(NULL); return;
        }
        ESP_LOGI(TAG, "CFG-CARRSMOOTH OK (windows=%d)", v);
    }

    /* CFG-PPS is intentionally disabled on TAU1201.
     *
     * Empirical: any CFG-PPS we send disables PPS output until a full power
     * cycle of the receiver. ESP32 warm reset does not recover it.
     *
     * Root cause (per TAU1201 Datasheet V1.7 and Protocol Spec V2.3):
     * The TAU1201 has a DEDICATED PPS pin (pin 3, labelled "PPS") rather
     * than a configurable GPIO. The CFG-PPS payload's GPIO byte is a
     * generic Cynosure II/III field intended for other Allystar modules
     * that can route PPS to one of several GPIOs. The TAU1201 datasheet
     * gives no GPIO index for its dedicated PPS pin, and the protocol
     * spec has no per-module mapping. Setting the field to ANY value is
     * a guess; the value 0 we tried evidently routes the PPS away from
     * the dedicated pin (or disables it).
     *
     * Additionally, the duty-cycle field's encoding is ambiguous (the
     * spec's formula and its worked example disagree by a factor of
     * 100×). Even with a correct GPIO value, the duty cycle might still
     * cause rejection.
     *
     * The TAU1201 factory default is what we want anyway:
     *   - 1 Hz period, rising-edge, 100 ms pulse
     *   - On the dedicated PPS pin (pin 3)
     *   - "Synchronized at rising edge" (datasheet §5)
     *
     * The 25 ns cable delay we wanted to compensate via PPS_OFFSET_NS is
     * trivial — the PTP servo absorbs it as part of the steady-state
     * offset during ACQUIRING. No accuracy loss.
     */
    // {
    //     uint8_t p[15];
    //     uint32_t period = PPS_PERIOD_US;
    //     int32_t  offset = PPS_OFFSET_NS;
    //     uint32_t duty = (uint32_t)(((uint64_t)PPS_PULSE_WIDTH_US * 1000000ULL)
    //                                / (uint64_t)PPS_PERIOD_US);

    //     p[0]  = (uint8_t)(period & 0xFF);
    //     p[1]  = (uint8_t)((period >>  8) & 0xFF);
    //     p[2]  = (uint8_t)((period >> 16) & 0xFF);
    //     p[3]  = (uint8_t)((period >> 24) & 0xFF);
    //     p[4]  = (uint8_t)(offset & 0xFF);
    //     p[5]  = (uint8_t)(((uint32_t)offset >>  8) & 0xFF);
    //     p[6]  = (uint8_t)(((uint32_t)offset >> 16) & 0xFF);
    //     p[7]  = (uint8_t)(((uint32_t)offset >> 24) & 0xFF);
    //     p[8]  = (uint8_t)(duty & 0xFF);
    //     p[9]  = (uint8_t)((duty >>  8) & 0xFF);
    //     p[10] = (uint8_t)((duty >> 16) & 0xFF);
    //     p[11] = (uint8_t)((duty >> 24) & 0xFF);
    //     p[12] = (uint8_t)PPS_POLARITY;
    //     p[13] = (uint8_t)PPS_GPIO;
    //     p[14] = (uint8_t)PPS_SYNC_KEEP_WHEN_UNFIXED;
    //     if (!send_cfg_with_ack(CFG_PPS, p, sizeof(p))) {
    //         fail_step(STEP_CFG_PPS); vTaskDelete(NULL); return;
    //     }
    //     ESP_LOGI(TAG, "CFG-PPS OK");
    // }

    /* -------- Phase B: wait for fix, run survey -------- */

    if (!wait_for_3d_fix()) {
        fail_step(STEP_3D_FIX); vTaskDelete(NULL); return;
    }
    if (!run_survey()) {
        fail_step(STEP_CFG_SURVEY); vTaskDelete(NULL); return;
    }

    ESP_LOGI(TAG, "Timing-mode configuration complete.");
    g_timing_error_step = NULL;
    g_timing_config_ok  = true;
    cleanup_after_sequence();
    vTaskDelete(NULL);
}

/* ============================ Public init ================================ */

void initialize_allystar_timing_service(allystar_timing_error_cb_t cb)
{
    if (timing_task_handle != NULL) {
        ESP_LOGW(TAG, "Service already initialised; ignoring re-init");
        return;
    }
    app_error_cb = cb;
    if (ack_sem       == NULL) ack_sem       = xSemaphoreCreateBinary();
    if (uart_tx_mutex == NULL) uart_tx_mutex = xSemaphoreCreateMutex();
    if (timing_stream == NULL) timing_stream = xStreamBufferCreate(STREAM_BUF_BYTES, 1);
    parser_reset();

    xTaskCreate(allystar_timing_sequence_task, "allystar_tcfg",
                4096, NULL, 5, &timing_task_handle);
}
