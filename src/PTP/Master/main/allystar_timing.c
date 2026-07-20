/**
 * @file allystar_timing.c
 *
 * TAU1201 timing-mode configurator. See allystar_timing.h for context.
 *
 * Big picture: the TAU1201 is a GNSS receiver chip (it listens to GPS/
 * GLONASS/Galileo/BeiDou/QZSS satellites to figure out where it is and,
 * more importantly for us, exactly what time it is). This project uses that
 * precise time as the reference clock for a PTP Grandmaster - a device
 * that hands out accurate time to other devices on the network (PTP =
 * Precision Time Protocol, IEEE 1588). A plain "I have a GPS fix" isn't
 * good enough for that: Some GNSS chips have a "timing mode" that trades some
 * navigation features for a much more stable, accurate 1-pulse-per-second
 * (PPS) signal - a hardware pulse, once a second, precisely aligned to the
 * start of the UTC second, which is what actually disciplines the clock.
 * This file's job is to talk to the receiver over a serial (UART) link at
 * boot and configure it into that timing mode.
 *
 * The receiver only understands a specific binary command protocol (not
 * plain text) for configuration, described below. This file builds those
 * binary commands, sends them, and waits for the receiver to confirm each
 * one before moving to the next - see "Frame builder" and "RX parser"
 * further down for how individual bytes on the wire are turned into
 * commands and responses.
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
 * ("Survey", here, means the receiver measures its own antenna position
 * very precisely over several minutes, by averaging many fixes, instead of
 * trusting a single noisy GPS fix. A timing receiver that knows exactly
 * where its antenna is can compute time more accurately, since it no
 * longer has to solve for its own position at the same time.)
 *
 * Survey completion strategy:
 *   The spec exposes no progress sentence for the survey. We poll NAV-POSLLH
 *   periodically and consider the survey "good enough" when either:
 *     (a) the configured 15-minute timer expires, OR
 *     (b) >= 5 minutes have elapsed AND hAcc <= SURVEY_HACC_TARGET_MM.
 *   The module continues converging internally regardless; this just lets us
 *   stop waiting and proceed with the rest of the boot sequence.
 *   (hAcc = "horizontal accuracy": the receiver's own estimate, in
 *   millimeters, of how far off its calculated position might be.)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* FreeRTOS is the real-time operating system ESP-IDF runs on top of - it's
 * what lets this firmware run several independent "tasks" (think: mini
 * threads) at once, e.g. one task reading GNSS UART data while another
 * handles the network stack. semphr.h gives us semaphores/mutexes
 * (thread-safety primitives - see "Module state" below); stream_buffer.h
 * gives us a thread-safe byte queue used to hand UART bytes from the main
 * GNSS task into this module's own task. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "driver/uart.h"   /* ESP-IDF's serial-port (UART) driver */
#include "esp_log.h"       /* ESP_LOGI/ESP_LOGW/ESP_LOGE: tagged console logging */
#include "esp_timer.h"      /* esp_timer_get_time(): microsecond-resolution clock */

#include "allystar_timing.h"

static const char *TAG = "ALLYSTAR_TIMING";  /* prefixes every log line from this file */

/* ============================ Configuration ============================== */

#define EX_UART_NUM                     UART_NUM_1
#define STREAM_BUF_BYTES                (2048)

/* Survey */
#define SURVEY_MAX_SECONDS  ALLYSTAR_SURVEY_MAX_SECONDS
#define SURVEY_MIN_SECONDS              (5 * 60)    /* earliest early-exit   */
#define SURVEY_HACC_TARGET_MM           (200)       /* 20 cm horizontal acc  */
#define SURVEY_POLL_PERIOD_MS           (5000)      /* NAV-POSLLH poll rate  */
#define SURVEY_ACCLIMIT_MM              (100)       /* tells module its goal */

/* 3D fix wait. "3D fix" = the receiver has locked onto enough satellites
 * to compute latitude, longitude, AND altitude (as opposed to a weaker 2D
 * fix). We require this before trusting the receiver's timing enough to
 * survey. The wait is INDEFINITE by design: the survey must start whenever
 * the fix arrives, however long acquisition takes — a cold start under a
 * poor sky view can exceed any fixed budget, and giving up would cost the
 * whole uptime its surveyed timing mode (see wait_for_3d_fix). */
#define FIX_STABLE_COUNT                (10)            /* consecutive GGAs  */

/* PPS = "Pulse Per Second", the once-a-second hardware tick described in
 * the file header above - the actual signal a PTP grandmaster disciplines
 * its clock against. These constants describe what we'd *like* the pulse
 * to look like, but see the "CFG-PPS is intentionally disabled" comment
 * further down for why they're currently unused. */
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
/* This is a bitmask: each satellite constellation (GPS, GLONASS, etc.) is
 * one bit. Setting a bit tells the receiver "you're allowed to use this
 * constellation"; more constellations generally means more visible
 * satellites and a faster, more robust fix, at the cost of a bit more
 * receiver processing. */

/* TEMPORARY DEBUGGING VALUES — loosen until fix is stable, then tighten */
/* DOP = "Dilution of Precision" - a unitless number describing how the
 * current satellite geometry affects fix accuracy (lower is better;
 * satellites spread out across the sky give low DOP, satellites clustered
 * together give high DOP). pDOP = position DOP, tDOP = time DOP. These
 * limits tell the receiver "don't bother reporting a fix as good if DOP is
 * worse than this" - set very loose (50) here for easier debugging; tighten
 * once everything is confirmed working, since lower limits mean stricter,
 * more accurate fixes only. "Elevation" (ELEV_*) similarly filters out
 * satellites too close to the horizon, where signals are weaker and
 * noisier - 0° here means "don't filter anything out." */
#define NAVSAT_ENABLE_MASK    (0xFFFFFFFFUL)   /* everything the module supports */
#define ELEV_TRK_RAD          (0.0f)           /* 0° — track anything */
#define ELEV_NAV_RAD          (0.0f)           /* 0° — use anything in nav */
#define DOP_PDOP_RAW          (5000)           /* pDOP 50 (very loose) */
#define DOP_TDOP_RAW          (5000)           /* tDOP 50 */

/* Static-hold speed. Spec §5.4.10 example shows raw value = cm/s directly
 * despite "scale 0.01" annotation. 10 = 10 cm/s = 0.1 m/s. */
/* "Static hold" tells the receiver to assume it isn't moving once its
 * speed drops below this threshold - reasonable for a fixed timing
 * antenna, and it lets the receiver's internal filters lock down harder
 * on a stable position/time instead of continuously re-solving for motion. */
#define SPDHOLD_RAW_CMS                 (10)

/* Carrier smoothing. Spec §5.4.18:
 *   -1 = auto, 0 = disable, >=1 = window (x+1). */
/* Carrier-phase smoothing blends the noisy raw pseudorange measurements
 * with the much cleaner (but ambiguous on its own) carrier-phase signal,
 * averaged over a moving window, to reduce position/time jitter. -1 lets
 * the receiver's firmware pick a sensible window size automatically. */
#define CARRSMOOTH_WINDOWS              (-1)            /* auto              */

/* ACK behaviour */
/* Every CFG-* command we send should get an ACK (acknowledged) or NAK
 * (rejected) reply. If none arrives within ACK_TIMEOUT_MS, we assume the
 * command (or its reply) was lost on the wire and resend, up to
 * MAX_COMMAND_RETRIES times, before giving up on that step entirely. */
#define MAX_COMMAND_RETRIES             (3)
#define ACK_TIMEOUT_MS                  (300)

/* ============================ Protocol bytes ============================= */
/* Every command/response on this UART link is a small binary "frame" - a
 * fixed structure of bytes, not human-readable text (that's the NMEA
 * sentences also flowing on this same wire, which this module ignores
 * except for the GGA fix-quality check further down). A frame looks like:
 *
 *   0xF1 0xD9 | GroupID | SubID | Length(2 bytes) | Payload | Checksum(2)
 *
 * SYNC1/SYNC2 are fixed bytes that mark "a frame starts here" - used by
 * the RX parser below to find frame boundaries in the raw byte stream.
 * GroupID+SubID together identify what kind of message it is (a specific
 * CFG-* command, an ACK/NAK, a NAV-POSLLH position report, etc.) - think
 * of them like a 2-byte "message type" opcode. */

#define UBP_SYNC1   0xF1
#define UBP_SYNC2   0xD9                    /* per spec §3 */

#define CLS_NAV     0x01   /* "NAV" group: navigation results (e.g. position) */
#define CLS_ACK     0x05   /* "ACK" group: acknowledge/reject a command       */
#define CLS_CFG     0x06   /* "CFG" group: configuration commands we send    */

#define ACK_ACK     0x01   /* SubID meaning "command accepted" within CLS_ACK */
#define ACK_NAK     0x00   /* SubID meaning "command rejected" within CLS_ACK */

/* SubIDs for the specific messages this file sends/receives. */
#define NAV_POSLLH      0x02   /* position report: lat/lon/height + accuracy */
#define CFG_PPS         0x07   /* configure the 1-pulse-per-second output   */
#define CFG_DOP         0x0A   /* set DOP (fix-quality) acceptance limits   */
#define CFG_ELEV        0x0B   /* set minimum satellite elevation to use    */
#define CFG_NAVSAT      0x0C   /* enable/disable GNSS constellations        */
#define CFG_SPDHOLD     0x0F   /* set the "assume stationary below this speed" threshold */
#define CFG_SURVEY      0x12   /* start the self-survey (see file header)   */
#define CFG_CARRSMOOTH  0x17   /* set carrier-phase smoothing window        */

/* ============================ Step names ================================= */
/* Human-readable labels for each configuration step, reported through the
 * error callback (see allystar_timing.h) so calling code - and whoever's
 * looking at the OLED display or serial log - can tell at a glance which
 * step failed, without decoding raw protocol IDs. */

static const char *STEP_CFG_NAVSAT   = "CFG-NAVSAT";
static const char *STEP_CFG_ELEV     = "CFG-ELEV";
static const char *STEP_CFG_DOP      = "CFG-DOP";
static const char *STEP_CFG_SPDHOLD  = "CFG-SPDHOLD";
static const char *STEP_CFG_CARRSM   = "CFG-CARRSMOOTH";
//static const char *STEP_CFG_PPS      = "CFG-PPS";
static const char *STEP_CFG_SURVEY   = "CFG-SURVEY";

/* ============================ Module state =============================== */
/* `volatile` below marks variables that get written by one FreeRTOS task
 * (this module's own configuration task) and read by another (e.g. the
 * main GNSS task, or OLED display code) - it tells the compiler not to
 * cache these values in a register, since another task could change them
 * at any time. A semaphore/mutex is a small lock: `ack_sem` lets the
 * "send a command and wait" code block until the RX parser signals that a
 * reply arrived, and `uart_tx_mutex` makes sure only one task writes to
 * the UART at a time (this task, and whatever else on the board might
 * also transmit on it), so bytes from two different sends can't interleave. */

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
/* This section builds the outgoing binary frames described above and
 * writes them to the UART. */

/* An 8-bit Fletcher checksum: a running pair of sums (a, b) computed over
 * every byte in the frame (except the sync bytes and the checksum bytes
 * themselves). The receiver recomputes the same sums over what it
 * received and compares - if a byte got corrupted in transit, the sums
 * won't match and the receiver silently drops the frame (which is why
 * send_cfg_with_ack() below has to retry on timeout: a corrupted command
 * looks identical to a lost one from the sender's point of view). */
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

/* Build and transmit a single binary frame. Returns true on TX success.
 * "TX success" only means the bytes were handed to the UART driver - it
 * says nothing about whether the receiver understood or accepted the
 * command; that's what the ACK/NAK dance in send_cfg_with_ack() is for. */
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
 * "Fed one byte at a time" is deliberate: UART data arrives in whatever
 * small chunks the hardware/driver happens to deliver, not neatly aligned
 * to frame boundaries, so the parser can't assume it ever sees a whole
 * frame at once. Instead it's a small state machine (a fancy way of
 * saying: a variable that remembers "which part of a frame am I currently
 * expecting next?", advanced one step per incoming byte). This makes it
 * naturally resilient to bytes arriving one, ten, or a hundred at a time -
 * the parser doesn't care, it just keeps its place between calls.
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

/* Pulls bytes out of the FreeRTOS stream buffer (filled by
 * allystar_timing_feed_data(), fed from the main GNSS UART-read loop) and
 * runs each one through parser_feed_byte(). This is how this module gets
 * its own copy of the raw serial traffic without interfering with the
 * normal NMEA parsing elsewhere in the firmware. */
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
/* Turns the fire-and-forget send_ubp_packet() into a reliable, synchronous
 * "send this command and don't return until we know whether the receiver
 * accepted it" call - retrying automatically if no reply shows up in time.
 * Every CFG-* step in the boot sequence goes through this. */

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
/* NMEA is the standard plain-text sentence format GNSS receivers use to
 * report position/time/status (as opposed to the binary ALLYSTAR frames
 * handled above) - e.g. a line like "$GPGGA,123519,...*47\r\n". Comma-
 * separated fields, a specific sentence per message type. Most of the
 * firmware's normal GPS handling reads these sentences directly; here we
 * only care about one thing from one sentence type: whether the receiver
 * currently has a 3D fix, read from a GGA sentence. */

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
/* The public entry point declared in allystar_timing.h - the main GNSS
 * task calls this with every chunk of raw UART bytes it reads, so this
 * module can watch the same stream (for ACKs and NAV-POSLLH replies)
 * without taking over the UART or duplicating the normal NMEA read loop. */

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
/* The actual boot-time configuration sequence: a dedicated FreeRTOS task
 * (spawned by initialize_allystar_timing_service() at the bottom of this
 * file) that runs through every CFG-* step in order, waits for a 3D fix,
 * runs the survey, and then exits. Everything above this point is
 * plumbing the sequence below relies on. */

/* Called once the sequence is over, whether it succeeded or failed. Flips
 * initialization_sequence_done so allystar_timing_feed_data() becomes a
 * cheap early-out from then on (see the feed hook above), and resets the
 * parser. The stream buffer is intentionally retained — see inside. */
static void cleanup_after_sequence(void)
{
    /* Stop forwarding bytes: from here on feed_data() early-outs on
     * initialization_sequence_done before ever touching the stream.
     *
     * The stream buffer is deliberately NOT deleted. gnss_task, on the
     * other core, may be BETWEEN feed_data()'s NULL-check and its
     * xStreamBufferSend() at this exact moment; deleting the buffer here
     * was a textbook TOCTOU use-after-free — a once-per-boot lottery for
     * heap corruption at the instant the sequence ended. No flag/delay
     * dance closes that window provably (the producer can always be
     * preempted between check and use for longer than any chosen delay).
     * Keeping the single 2 KB, once-per-boot buffer allocated for the
     * rest of the uptime closes it by construction: a racing send lands
     * harmlessly in a live buffer that nobody will ever read again. */
    initialization_sequence_done = true;
    parser_reset();
}

/* Records that a named step failed (readable by OLED/status code via
 * g_timing_error_step), tears down sequence resources, and notifies the
 * caller-supplied error callback, if any, so the rest of the firmware can
 * react (e.g. show an error, fall back to a less accurate clock source). */
static void fail_step(const char *step)
{
    ESP_LOGE(TAG, "Step failed: %s", step);
    g_timing_error_step = step;
    g_timing_config_ok  = false;
    cleanup_after_sequence();
    if (app_error_cb) app_error_cb(step);
}

/* Pull NMEA lines out of the stream buffer until we observe FIX_STABLE_COUNT
 * consecutive GGAs with a 3D fix. Waits INDEFINITELY, by design: the survey
 * must start whenever the fix arrives, however long acquisition takes — a
 * 15-minute cold start under a poor sky view is still a perfectly good
 * survey candidate, and giving up would cost the whole uptime its surveyed
 * timing mode. A once-a-minute log line keeps a genuinely fixless box (no
 * antenna, indoors) diagnosable from the serial console; the OLED keeps
 * showing RUNNING meanwhile.
 * Requiring several consecutive good fixes (not just one) filters out a
 * fix that flickers briefly - e.g. from a bad satellite geometry moment -
 * before we commit to trusting the receiver's timing. */
static void wait_for_3d_fix(void)
{
    ESP_LOGI(TAG, "Waiting for stable 3D fix (no timeout — survey starts "
                  "whenever the fix arrives)...");
    char line[128];
    size_t lidx = 0;
    int stable = 0;
    const int64_t t0_us = esp_timer_get_time();
    int64_t last_note_us = t0_us;

    for (;;) {
        uint8_t b;
        size_t n = xStreamBufferReceive(timing_stream, &b, 1,
                                        pdMS_TO_TICKS(100));

        /* Progress heartbeat, once a minute, whether or not bytes flow. */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_note_us >= 60LL * 1000000LL) {
            ESP_LOGW(TAG, "Still waiting for 3D fix (%lld min elapsed)",
                     (long long)((now_us - t0_us) / 60000000LL));
            last_note_us = now_us;
        }

        if (n == 0) {
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
                        ESP_LOGI(TAG, "3D fix stable after %lld s",
                                 (long long)((now_us - t0_us) / 1000000LL));
                        return;
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
}

/* Run CFG-SURVEY, then loop polling NAV-POSLLH until done-enough.
 * See the "Survey completion strategy" note in the file header for the
 * exact stop condition (max duration, or good-enough accuracy after a
 * minimum wait). */
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

/* The FreeRTOS task function itself: runs top to bottom exactly once per
 * boot, sending each CFG-* command in turn (bailing out via fail_step()
 * and vTaskDelete(NULL) - FreeRTOS's "this task is done, clean me up" call
 * - the moment anything fails), then waiting for a fix and running the
 * survey, before marking the whole sequence successful. */
static void allystar_timing_sequence_task(void *pv)
{
    (void)pv;   /* unused FreeRTOS task parameter */

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

    /* -------- Phase B: wait for fix (indefinitely), then run survey -------- */

    wait_for_3d_fix();
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
/* The other public entry point from allystar_timing.h: called once during
 * boot to create the semaphores/mutex/stream buffer this module needs and
 * spawn the sequence task above. Safe to call more than once - only the
 * first call actually does anything, since the receiver only needs to be
 * configured once per power-up. */

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

    /* 4096-byte stack, priority 5, no task-notification handle needed by
     * the caller - the handle is only kept so we can detect re-init above. */
    xTaskCreate(allystar_timing_sequence_task, "allystar_tcfg",
                4096, NULL, 5, &timing_task_handle);
}
