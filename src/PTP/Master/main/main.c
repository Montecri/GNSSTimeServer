/*
 * ═════════════════════════════════════════════════════════════════════════
 * PTPv2 Grandmaster Server for ESP32-P4
 * ═════════════════════════════════════════════════════════════════════════
 * Cristiano Reis Monteiro <cristianomonteiro@gmail.com> - May/2026
 * ═════════════════════════════════════════════════════════════════════════
 *
 * IEEE 1588-2008, Annex F (L2/Ethernet transport).
 * Target board : Guition JC-ESP32P4-M3-DEV board
 * ESP-IDF      : 5.5.4
 *
 * Build target: PTPv2 GRANDMASTER (master-only firmware).
 *
 * GNSS-disciplined grandmaster: drives the PHC from a 1-PPS edge captured on
 * the GNSS module, transmits Sync / Follow_Up / Announce, and answers
 * Delay_Req from connected slaves.
 *
 * High-level structure of this file (top to bottom):
 *   - Includes
 *   - Configuration #defines, grouped by subsystem
 *   - Type definitions (PTP wire structs, BMCA, servo, LED, GNSS)
 *   - Global state, grouped by subsystem with its protecting lock
 *   - Forward declarations
 *   - Small helpers (time conversion, filter resets, holdover advertise)
 *   - EMAC PHC + PTP wire helpers
 *   - BMCA
 *   - Slave tracking
 *   - PTP message handlers (Delay_Resp, Sync/FollowUp, Announce)
 *   - NMEA parsers (ZDA, RMC, GSV, GSA)
 *   - PPS ISR + pps_task + servo_task  (the timing core)
 *   - Background tasks (gnss, gnss_signal, ptp_rx/tx, heap, watchdog)
 *   - LED status indicator
 *   - OLED display + button
 *   - Network init (DHCP fallback, Ethernet, L2TAP, PHC start)
 *   - Hardware init (GNSS UART, PPS GPIO)
 *   - app_main
 *
 * ─────────────────────────────────────────────────────────────────────────
 * Performance / tuning notes (historical, kept for context)
 * ─────────────────────────────────────────────────────────────────────────
 * PTP is very time-sensitive. If PPS jitter is higher than desired, move the
 * timestamping logic in ptp_rx_task as high as possible after read() returns
 * to capture the state quickly. For a standard L2 implementation on the P4
 * the current structure is already optimized for sub-microsecond accuracy.
 *
 * What GNSS timing mode buys:
 *   - Holdover stability: with much lower PPS jitter, short GNSS outages leave
 *     the master with a far better free-running clock (timing-mode receivers
 *     typically advertise <100 ns holdover for several minutes vs. hundreds
 *     of ns in nav mode).
 *   - Headroom to tighten servo gains. The current acquisition-phase gains
 *     (SERVO_KP=0.4 / SERVO_KI=0.3, used only in SERVO_ACQUIRING; steady-state
 *     uses the much slower KP_SLOW/KI_SLOW pair) were tuned around ~100-200 ns
 *     PPS jitter; with ~10 ns PPS jitter Kp=0.6 / Ki=0.4 and faster lock
 *     acquisition would work without overshoot. Not needed — current
 *     performance is already excellent — but the headroom exists.
 *   - A shorter holdover threshold (currently 5 s) could be used safely
 *     without false alarms: with timing-mode PPS, missing 2 PPS edges
 *     genuinely indicates a problem rather than jitter eating a sample.
 *
 * None of these are required: the system performs at the practical limit of
 * the hardware combination. The takeaway is that GNSS timing mode is the
 * single highest-impact configuration change a PTP grandmaster can make.
 * ═════════════════════════════════════════════════════════════════════════
 */

/* ═════════════════════════════════════════════════════════════════════════
 * Includes
 * ═════════════════════════════════════════════════════════════════════════ */
#include <u8g2.h>
#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include <string.h>
#include <driver/spi_master.h>
#include <stdatomic.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_flash.h"
#include <inttypes.h>
#include "esp_netif.h"
#include "esp_netif_types.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_app_desc.h" /* esp_app_get_description() — firmware version from version.txt */
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_eth_mac.h"
#include "esp_eth_mac_esp.h"

#include "esp_vfs_l2tap.h"
#include "lwip/prot/ethernet.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "u8g2_esp32_hal.h"

#include "allystar_timing.h"
#include "mac_hostname.h" /* deterministic MAC -> hostname (components/mac_hostname) */

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Configuration #defines
 *
 * Compile-time constants grouped by subsystem. Servo PI(D) tuning gets the
 * most explanation because it encodes hard-won field-tuning history.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─── IDF version compatibility ─── */
#ifndef L2TAP_IREC_TYPE_TX_CONFIRM
#define L2TAP_IREC_TYPE_TX_CONFIRM 3
#endif

/* ─── Ethernet / PHY (IP101) ─── */
#define ETH_PHY_ADDR 1
#define ETH_PHY_RST_GPIO 51
#define ETH_MDC_GPIO 31
#define ETH_MDIO_GPIO 52
#define ETH_TYPE_PTP 0x88F7

/* ─── Network / DHCP fallback ───
 * Fallback static IP — used if DHCP doesn't reply in time. Pick a private
 * subnet you don't use elsewhere. */
#define DHCP_FALLBACK_TIMEOUT_MS 30000 /* wait up to 30 s for DHCP */
#define STATIC_IP_ADDR ESP_IP4TOADDR(192, 168, 5, 2)
#define STATIC_IP_NETMASK ESP_IP4TOADDR(255, 255, 255, 0)
#define STATIC_IP_GATEWAY ESP_IP4TOADDR(192, 168, 5, 1)

/* ─── PTP protocol (IEEE 1588-2008) ───
 * Sync cadence is 1 Hz, Announce 0.5 Hz. ptp_tx_task ticks at
 * PTP_SYNC_INTERVAL_MS and sends an Announce every PTP_ANNOUNCE_DIV ticks.
 *
 * NOTE: the on-wire logMessageInterval fields in build_ptp_header() calls
 * (0 for Sync/FollowUp, 1 for Announce) are hardcoded to match these
 * defaults (log2(1s)=0, log2(2s)=1). If you change the intervals to values
 * that no longer satisfy log2(interval_sec) ∈ {0,1}, update those literals
 * in send_sync_and_followup() and send_announce() as well so slaves see
 * the advertised cadence match the actual one. */
#define PTP_SYNC_INTERVAL_MS 1000
#define PTP_ANNOUNCE_INTERVAL_MS 2000

/* Announce-every-N-Sync-ticks divisor, computed at compile time.
 * Round to nearest so a non-integer ratio picks the closest viable cadence. */
#define PTP_ANNOUNCE_DIV \
    (((PTP_ANNOUNCE_INTERVAL_MS) + (PTP_SYNC_INTERVAL_MS) / 2) / (PTP_SYNC_INTERVAL_MS))

_Static_assert(PTP_SYNC_INTERVAL_MS > 0,
               "PTP_SYNC_INTERVAL_MS must be positive");
_Static_assert(PTP_ANNOUNCE_INTERVAL_MS >= PTP_SYNC_INTERVAL_MS,
               "PTP_ANNOUNCE_INTERVAL_MS must be >= PTP_SYNC_INTERVAL_MS "
               "(the tx task ticks at the Sync rate)");
_Static_assert(PTP_ANNOUNCE_DIV >= 1,
               "PTP_ANNOUNCE_DIV must be >= 1 (check the two intervals)");
// Defined at task creation
// #define PTP_TASK_PRIORITY 5
#define PTP_DOMAIN 0

#define PTP_FLAG_TWO_STEP 0x0200

/* TimePropertiesDS flags (IEEE 1588-2008) */
#define PTP_FLAG_LEAP61 (1 << 0)
#define PTP_FLAG_LEAP59 (1 << 1)
#define PTP_FLAG_UTC_OFFSET_VALID (1 << 2)
#define PTP_FLAG_PTP_TIMESCALE (1 << 3)
#define PTP_FLAG_TIME_TRACEABLE (1 << 4)
#define PTP_FLAG_FREQ_TRACEABLE (1 << 5)

/* PTP message types (messageType field) */
#define PTP_MSG_SYNC 0x0
#define PTP_MSG_DELAY_REQ 0x1
#define PTP_MSG_FOLLOW_UP 0x8
#define PTP_MSG_DELAY_RESP 0x9
#define PTP_MSG_ANNOUNCE 0xB

#define ANNOUNCE_TIMEOUT_SEC 6

/* ─── BMCA ─── */
#define BMCA_STABILITY_THRESHOLD 3

/* ─── Slave tracking — count distinct slave clock IDs seen in last N sec ─── */
#define SLAVE_TRACK_MAX 8      /* max simultaneously-tracked slaves */
#define SLAVE_TIMEOUT_MS 30000 /* a slave is "active" if seen <30s ago */

/* ─── Heap monitor ─── */
#define HEAP_MON_INTERVAL_MS 60000 /* heap_monitor_task's steady-state loop period */

/* ─── GNSS UART + PPS pins ─── */
#define GNSS_UART_NUM UART_NUM_1
#define GNSS_UART_BAUD 115200
#define GNSS_TX_PIN 33
#define GNSS_RX_PIN 32
#define PPS_GPIO 20

/* ─── Holdover ─── */
#define HOLDOVER_TIMEOUT_SEC 5

/* Post-holdover settling: keep advertising clockClass 7 until the servo
 * has been continuously LOCKED for this long after GNSS recovery. Prevents
 * downstream ptp4l clients from reacquiring/railing their servo while our
 * PHC is still settling from a COARSE STEP. */
#define POST_HOLDOVER_SETTLE_MS 30000 /* 30 s */

/* ─── Servo PI(D) tuning ───
 *
 * Two gain sets exist. The "fast" (KP/KI) pair is used during acquisition;
 * the "slow" (KP_SLOW/KI_SLOW/KD_SLOW) pair is the steady-state servo.
 *
 * PID tuning — KD term added to handle thermal transients.
 *
 *   KP_SLOW = 0.05     — proportional, responds to current averaged offset
 *   KI_SLOW = 0.001    — integral, learns crystal drift (~1000 s / ~17 min
 *                        timescale: 1/KI at the 1 Hz sample rate)
 *   KD_SLOW = 2.0      — derivative, reacts to rate of drift change
 *   D_WINDOW = 10      — derivative computed as slope of averaged offset
 *                        over the last 10 seconds, smoothing noise spikes.
 *
 * The D term is critical for tracking thermal transients during boot
 * warmup, where the crystal's natural frequency changes by hundreds of
 * ppb/sec. The integrator (KI) alone is too slow to catch this; the
 * derivative term sees the offset moving and adds immediate frequency
 * correction proportional to *how fast* it's diverging.
 *
 * Simulation against the actual recorded thermal-transient log:
 *   PI  (no D):  max avg offset 5,940 ns, lingering steady-state error
 *   PID (KD=2):  max avg offset 5,260 ns, drives to zero
 *
 * Steady-state (±300 ns receiver noise, 1 hour after settling, partially
 * obstructed sky view, urban jungle):
 *   30% of time within ±300 ns of true GNSS time
 *   74% of time within ±1 µs
 *   89% of time within ±2 µs
 *   Mean error -74 ns.
 *
 * SERVO_KD_ENABLE gates the D term off by default (set to 1 to re-enable). */
#define SERVO_KP 0.4 // was 0.7
#define SERVO_KI 0.3
#define SERVO_KP_SLOW 0.05
#define SERVO_KI_SLOW 0.001
#define SERVO_KD_SLOW 2.0
#define SERVO_D_WINDOW 10 /* derivative smoothing samples */
#define ADJ_SLEW_PPB_PER_S 100.0
#define SERVO_KD_ENABLE 0 // set to 1 to re-enable D term

/* Hardware output clamp for the PHC frequency adjustment. The ESP32-P4
 * EMAC accepts adjustments in the ±500,000 ppb (±500 ppm) range. The
 * actual crystal needs ~50 ppm of correction, so this is a sanity rail
 * to prevent runaway, not a tuning parameter. */
#define SERVO_FREQ_LIMIT_PPB 500000.0

/* ─── Servo lock-acquisition criteria ───
 *
 * Three conditions must all hold for LOCK_THRESHOLD consecutive samples
 * before SERVO_LOCKED is declared:
 *   1. |offset|                < LOCK_OFFSET_NS        (phase converged)
 *   2. |offset - prev_offset|  < LOCK_FREQ_DELTA_NS    (frequency converged)
 *   3. Count of (1)&(2) being true ≥ LOCK_THRESHOLD    (sustained)
 *
 * Phase 5 µs: prior good runs showed steady-state ±500 ns offset. A 5 µs
 * gate guarantees we don't declare lock while the servo is still
 * mid-correction, but is loose enough that normal sample-to-sample PPS
 * jitter doesn't bounce us out.
 *
 * Frequency 2 µs delta: PI servo at steady state moves the filtered
 * offset by under 1 µs/sample. A 2 µs gate excludes the settling phase
 * (where consecutive samples may differ by 100+ µs).
 *
 * Threshold LOCK_THRESHOLD = 8, applied as `stable_count > LOCK_THRESHOLD` in
 * servo_task, so lock is actually declared on the 9th consecutive sample
 * (≈9 seconds): with both phase + freq gated, this is solid evidence of
 * convergence without being painfully slow. */
#define LOCK_THRESHOLD 8
/* Holdover: hold the learned crystal correction (integral term) instead of
 * steering the PHC to the receiver's free-running PPS. On the bench the
 * TAU1201's PPS drifted ~1.15 us/s without a fix, the free-running PHC
 * 0.2-0.5 us/s. 0 restores the previous behaviour (P term active). */
#define HOLDOVER_FREEZE_FREQ 1
/* Holdover exit realign (SERVO_LOCKED): collect REALIGN_SAMPLES mutually
 * consistent raw offsets (spread < REALIGN_SPREAD_NS, each < REALIGN_MAX_NS so
 * a GNSS time that is still settling is waited out), then step the PHC by their
 * median if it exceeds REALIGN_MIN_NS. The learned frequency is kept. Gives up
 * after REALIGN_TIMEOUT_MS with a full re-acquire. */
#define REALIGN_SAMPLES 8
#define REALIGN_MIN_NS 10000LL
#define REALIGN_MAX_NS 100000000LL
#define REALIGN_SPREAD_NS 50000LL
#define REALIGN_TIMEOUT_MS 120000
/* How the error is removed. Up to REALIGN_STEP_NS it is SLEWED: the frequency is
 * offset by at most REALIGN_SLEW_MAX_PPB for whole servo cycles (1 ppb for 1 s =
 * 1 ns), then returned to the learned value -- accurate to ~100 ns. A read-then-set
 * step lands ~25-45 us behind on this hardware (write latency), so steps are only
 * used for errors where that no longer matters. */
#define REALIGN_STEP_NS 1000000LL
#define ACQ_A_JUMP_NS 1000000LL /* ACQUIRE-A steps only if no sample-to-sample change exceeds this */
#define REALIGN_SLEW_MAX_PPB 100000LL
/* Windowed lock test (ACQUIRING): LOCKED is also declared when the last
 * LOCK_WIN_N offsets have |mean| < LOCK_OFFSET_NS and a least-squares slope
 * (= residual frequency error, ns/s = ppb) below LOCK_WIN_SLOPE_PPB. Averaging
 * over the window removes PPS capture jitter that makes the 9-consecutive-
 * sample rule above fail on noisy antennas. Never used during holdover.
 * Every sample in the window must also be within +/-LOCK_WIN_MAX_NS, so a
 * loop that is still ringing (large swings averaging to ~0) cannot pass. */
#define LOCK_WIN_N 32
#define LOCK_WIN_SLOPE_PPB 250.0
#define LOCK_WIN_MAX_NS (4 * LOCK_OFFSET_NS)
/* The window rule is a fallback: it may only declare LOCKED after this many
 * samples in ACQUIRING, so whenever the 9-sample rule can lock (clean PPS)
 * behaviour is exactly as before. */
#define LOCK_WIN_GRACE 120
#define LOCK_OFFSET_NS 5000LL
#define LOCK_FREQ_DELTA_NS 2000LL


/* Consecutive HARD_REJECTs in SERVO_LOCKED that force a full re-acquire.
 * Defined here (not inside servo_task) so render_screen_servo, which is
 * defined earlier in the file, can display the streak against this limit. */
#define REJECT_STREAK_REACQUIRE 45

/* ─── Servo offset filtering windows ───
 *
 * NOTE: the median filter described in the following paragraph is currently
 * NOT WIRED INTO THE SIGNAL PATH — the servo runs directly off the 32-sample
 * running-mean averager below (g_avg_ring). MEDIAN_WINDOW, g_offset_history[],
 * and median_filter_reset() are retained as dormant scaffolding for a possible
 * future first-stage filter; the reset calls scattered through servo_task are
 * effectively no-ops. The commentary is kept for context if the filter is
 * ever restored.
 *
 * Median filter window. 9 samples kills bursts of up to 4 consecutive bad
 * PPS edges (the median is only corrupted when 5+ of 9 samples are bad).
 * Lag is ~4.5 s in steady state — invisible to PTP slaves because they
 * see the filtered PHC, not the raw PPS. Wider window beats fiddling
 * with PI gains for noisy-PPS hardware.
 *
 * Long-window running mean averager — the master servo's working signal.
 * PI control on a 5/9-sample median can't track a reference whose noise is
 * wider than the filter. The PPS source has wide RMS jitter; even a 9-sample
 * median rings on it. A 32-sample arithmetic mean cancels zero-mean noise
 * statistically (1/√32 ≈ 5.7× reduction) and yields a clean averaged-offset
 * signal. Trade-off: ~16 s lag on real motion, acceptable because PHC crystal
 * drift is slow, slaves see the disciplined PHC (not the lagged servo state),
 * and the slew-rate-limited output (ADJ_SLEW_PPB_PER_S) means the servo
 * physically cannot react to transients faster than the averager smooths
 * them — by design. With ~2 µs RMS PPS jitter under the new low-noise antenna
 * conditions, the 32-sample window gives ~0.35 µs RMS — adequate for sub-µs
 * lock. Memory: 256 bytes per averager. Cheap. */
#define MEDIAN_WINDOW 9
#define AVG_WINDOW 32

/* ─── Lock-quality tracking (OLED screen 2) ───
 * Ratio of raw PPS samples within ±2 µs of the averaged baseline. Operates
 * on the raw (un-averaged) offset so the displayed quality reflects what the
 * receiver is actually emitting, not the smoothed working signal — which is
 * always near zero by design.
 *
 * Calibration: in clean conditions raw is typically ±500 ns; on a poor
 * antenna with constellation churn it bursts to ±5-10 µs. ±2 µs is the
 * meaningful "is the receiver quiet right now" gate. */
#define LOCK_QUAL_WINDOW 64
#define LOCK_QUAL_GOOD_NS 2000 /* ±2 µs raw — was 1000 (and on avg signal) */

/* ─── GNSS signal monitoring (GSV/GSA) ───
 * SV_CHURN_PRN_MAX covers GPS(1-32) GLO(65-96) GAL(...) QZSS BeiDou; generous. */
#define SV_CHURN_PRN_MAX 192
#define GSV_MAX_SVS_PER_TALKER 32
#define GSV_TALKER_COUNT 6

/* ─── LED status indicator ─── */
#define LED_GPIO 2        // <-- set to your LED pin
#define LED_ACTIVE_HIGH 1 // 0 if LED is wired active-low
/* 3072 since 2026-10: 2048 left only ~250 B free on the bench (one log call could overflow) */
#define LED_TASK_STACK 3072
#define PPS_LED_TASK_STACK 3072 /* was 2048 inline: ~250 B free */
#define LED_TASK_PRIO 1 // low priority — never blocks anything
#define LED_TICK_MS 10  // pattern resolution

#define PPS_LED_GPIO 3        // <-- set to your second LED pin
#define PPS_LED_ACTIVE_HIGH 1 // 0 if active-low
#define PPS_LED_ON_MS 50      // how long the pulse stays lit (tunable)

/* ─── OLED (I2C SH1106 128x64) + screen-cycle button ─── */
#define PIN_SDA 7
#define PIN_SCL 8

#define DISPLAY_BUTTON_GPIO 1       // <-- set to your button pin
#define DISPLAY_BUTTON_ACTIVE_LOW 1 // 1 if button shorts to GND with pull-up

#define DISPLAY_TASK_STACK 4096
#define DISPLAY_TASK_PRIO 1    // lowest meaningful priority
#define DISPLAY_REFRESH_MS 500 // 2 Hz refresh

#define BUTTON_TASK_STACK 3072 /* was 2048: ~250 B free */
#define BUTTON_TASK_PRIO 1
#define BUTTON_POLL_MS 20       // debounce poll interval
#define BUTTON_DEBOUNCE_TICKS 3 // 3 × 20ms = 60ms stable required

#define DISPLAY_NUM_SCREENS 5

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Type definitions
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─── PTP wire-format structures (packed, on-the-wire layout) ─── */
#pragma pack(push, 1)
typedef struct
{
    uint8_t seconds_msb[2];
    uint8_t seconds_lsb[4];
    uint8_t nanoseconds[4];
} ptp_timestamp_t;

typedef struct
{
    uint8_t clock_identity[8];
    uint16_t port_number;
} ptp_port_id_t;

typedef struct
{
    uint8_t msg_type_transport;
    uint8_t version_ptp;
    uint16_t msg_length;
    uint8_t domain_number;
    uint8_t reserved1;
    uint16_t flags;
    int64_t correction_field;
    uint32_t reserved2;
    ptp_port_id_t source_port_id;
    uint16_t sequence_id;
    uint8_t control;
    int8_t log_msg_interval;
} ptp_header_t;

typedef struct
{
    ptp_header_t hdr;
    ptp_timestamp_t origin_timestamp;
} ptp_sync_msg_t;

typedef ptp_sync_msg_t ptp_delay_req_msg_t;

typedef struct
{
    ptp_header_t hdr;
    ptp_timestamp_t receive_timestamp;
    ptp_port_id_t requesting_port_id;
} ptp_delay_resp_msg_t;

typedef struct
{
    ptp_header_t hdr;
    ptp_timestamp_t origin_timestamp;
    int16_t current_utc_offset;
    uint8_t reserved;
    uint8_t grandmaster_priority1;
    uint8_t grandmaster_clock_class;
    uint8_t grandmaster_clock_accuracy;
    uint16_t grandmaster_clock_variance;
    uint8_t grandmaster_priority2;
    uint8_t grandmaster_identity[8];
    uint16_t steps_removed;
    uint8_t time_source;
} ptp_announce_msg_t;
#pragma pack(pop)

/* ─── BMCA dataset (Best Master Clock Algorithm comparison fields) ─── */
typedef struct
{
    uint8_t priority1;
    uint8_t clock_class;
    uint8_t accuracy;
    uint16_t variance;
    uint8_t priority2;
    uint8_t clock_id[8];
    uint16_t steps_removed; /* host order; 0 for the local (GM) dataset */
} bmca_dataset_t;

/* Which comparison step decided a BMCA outcome. Order mirrors bmca_compare(). */
typedef enum
{
    BMCA_FIELD_NONE = 0,
    BMCA_FIELD_STEPS_REMOVED, /* same GM identity: decided on stepsRemoved (Fig. 28) */
    BMCA_FIELD_PRIORITY1,
    BMCA_FIELD_CLOCK_CLASS,
    BMCA_FIELD_ACCURACY,
    BMCA_FIELD_VARIANCE,
    BMCA_FIELD_PRIORITY2,
    BMCA_FIELD_CLOCK_IDENTITY,
} bmca_field_t;

typedef enum
{
    BMCA_REASON_BETTER_ANNOUNCE = 1, /* MASTER -> PASSIVE: remote dataset beat ours      */
    BMCA_REASON_WORSE_ANNOUNCE,      /* PASSIVE -> MASTER: remote dataset no longer better */
    BMCA_REASON_ANNOUNCE_TIMEOUT,    /* PASSIVE -> MASTER: no Announce for > timeout      */
} bmca_reason_t;

#define BMCA_EVENT_RING 8

typedef struct
{
    int64_t t_ms;       /* uptime ms (esp_timer based, same base as g_boot_time_ms) */
    uint8_t to_master;  /* 1 = became MASTER, 0 = became PASSIVE */
    uint8_t reason;     /* bmca_reason_t */
    uint8_t decided_by; /* bmca_field_t (NONE for timeout) */
    uint8_t peer_valid; /* 0 for timeout */
    uint8_t peer_id[8];
} bmca_event_t;

/* Everything the web layer reports about BMCA. Written ONLY by ptp_rx_task,
 * read by httpd through the seqlock below. */
typedef struct
{
    bmca_dataset_t local; /* our dataset as used in the last evaluation */
    bool           local_valid;
    bmca_dataset_t peer; /* last remote dataset that passed the stability filter */
    bool           peer_valid;
    bool           peer_better; /* true => we are PASSIVE because of this peer */
    uint8_t        decided_by;  /* bmca_field_t of the last evaluation */
    uint8_t        cand_id[8];  /* stability-filter candidate (may differ from peer) */
    uint8_t        cand_count;  /* saturates at 255 */
    bool           cand_valid;
    int64_t        last_ann_rx_ms; /* uptime ms of last accepted Announce, 0 = never */
    uint32_t       role_changes;   /* lifetime count, not limited by the ring */
    bmca_event_t   ev[BMCA_EVENT_RING];
    uint8_t        ev_next;  /* index of next slot to write */
    uint8_t        ev_count; /* 0..BMCA_EVENT_RING */
} bmca_view_t;

/* Seqlock protecting g_bmca_view — single writer (ptp_rx_task), read by httpd.
 * "Uptime ms" everywhere in bmca_view_t means
 * esp_timer_get_time() / 1000 - g_boot_time_ms, the same base /api/status
 * uses for uptime_s. */
static bmca_view_t g_bmca_view;
static _Atomic uint32_t g_bmca_seq = 0;

/* Writer side — call ONLY from ptp_rx_task. Never nest begin/end. */
static inline void bmca_view_write_begin(void)
{
    uint32_t s = atomic_load_explicit(&g_bmca_seq, memory_order_relaxed);
    atomic_store_explicit(&g_bmca_seq, s + 1u, memory_order_relaxed); /* odd */
    atomic_thread_fence(memory_order_release);
}
static inline void bmca_view_write_end(void)
{
    uint32_t s = atomic_load_explicit(&g_bmca_seq, memory_order_relaxed);
    atomic_store_explicit(&g_bmca_seq, s + 1u, memory_order_release); /* even */
}

/* Reader side — httpd. Returns false if a consistent copy could not be taken.
 * No delay in the retry loop: the writer (ptp_rx_task, prio 7) cannot be
 * preempted by httpd (prio 5) on the same core, and its write sections are a
 * few hundred bytes of stores with no blocking calls, so an odd sequence
 * resolves within microseconds. If this still returns false, the caller must
 * respond 503 (never emit a torn object). */
static bool bmca_view_snapshot(bmca_view_t *out)
{
    for (int tries = 0; tries < 16; tries++)
    {
        uint32_t s1 = atomic_load_explicit(&g_bmca_seq, memory_order_acquire);
        if (s1 & 1u)
            continue; /* writer mid-update: retry */
        memcpy(out, (const void *)&g_bmca_view, sizeof(*out));
        atomic_thread_fence(memory_order_acquire);
        uint32_t s2 = atomic_load_explicit(&g_bmca_seq, memory_order_relaxed);
        if (s1 == s2)
            return true;
    }
    return false;
}

/* ─── Servo state machine ─── */
typedef enum
{
    SERVO_UNLOCKED = 0,
    SERVO_ACQUIRING,
    SERVO_LOCKED
} servo_state_t;

/* ─── Slave tracking entry ─── */
typedef struct
{
    uint8_t clock_id[8];
    int64_t last_seen_ms;
} slave_track_t;

/* ─── Delay_Resp work item ───
 * Carries the minimum needed to build a Delay_Resp off the receive thread:
 * the RX hardware timestamp (the value the slave actually needs), the
 * requesting port identity to echo back, and the sequenceId to echo. The
 * wall-clock instant we send the response is irrelevant to PTP math, so
 * deferring the send via this queue costs zero accuracy. */
typedef struct
{
    struct timespec rx_hw_ts;
    ptp_port_id_t req_port_id; /* echoed as requestingPortIdentity */
    uint16_t req_seq;          /* host order */
    int64_t req_correction;    /* Delay_Req correctionField, RAW network
                                * byte order — echoed verbatim into the
                                * Delay_Resp (IEEE 1588-2008 11.3.2 c).
                                * E2E transparent clocks in the path add
                                * their residence time here; dropping it
                                * (the old behavior: always 0) silently
                                * biased every slave's path delay. */
} delay_resp_job_t;

/* ─── LED status states ─── */
typedef enum
{
    LED_STATE_NO_LINK = 0, // Ethernet not up
    LED_STATE_ACQUIRING,   // link up, waiting for GNSS / servo not locked
    LED_STATE_LOCKED,      // servo locked, time is good
    LED_STATE_HOLDOVER,    // was locked, GNSS lost, drifting
    LED_STATE_FAULT,       // unrecoverable error
} led_state_t;

/* ─── GSV per-talker SNR accumulator ─── */
typedef struct
{
    /* Per-SV C/N₀ for the in-progress group (one entry per SV reported).
     * Cleared when a new sentence-1 arrives, accumulated as later
     * sentences arrive, snapshotted to globals on the final sentence. */
    uint8_t snr[GSV_MAX_SVS_PER_TALKER]; /* 0 = invalid / not reported */
    int count;                           /* last-seen sv_in_view field from the
                                          * GSV sentence header (i.e. what the
                                          * receiver claims for this group);
                                          * the true accumulated count is
                                          * derived by scanning snr[] in
                                          * gsv_publish_snapshot */
    int total_expected;                  /* from "sats in view" field */
} gsv_acc_t;

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Global state
 *
 * Grouped by subsystem. Each shared-across-tasks group keeps its protecting
 * spinlock declared right beside it.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─── Constants ─── */
static const char *TAG = "PTP_SERVER";
static const char *LED_TAG = "LED";
static const uint8_t PTP_MCAST_MAC[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};

/* ─── Network / Ethernet ─── */
static esp_netif_t *g_eth_netif = NULL;
static SemaphoreHandle_t g_got_ip_sem = NULL;
static SemaphoreHandle_t g_link_down_sem = NULL; /* given by on_link_down, awaited by dhcp_fallback_task */
static volatile bool g_static_applied = false;
static int g_l2tap_fd = -1;
static esp_eth_handle_t g_eth_hndl = NULL;
static SemaphoreHandle_t g_eth_up_sem;
static _Atomic bool g_link_up = false;
static _Atomic bool g_l2tap_ready = false;
/* Timing-core readiness: the EMAC driver is started, the PHC is running, and
 * L2TAP is bound. This depends ONLY on the driver being started (esp_eth_start),
 * NOT on a peer being connected. The servo and pps tasks gate on this so the
 * clock disciplines to GNSS at power-on regardless of whether a slave/link is
 * present. PTP packet I/O (rx/tx tasks) gates on g_link_up instead, since that
 * genuinely needs a peer on the wire. */
static _Atomic bool g_phc_ready = false;
/* Set true while emac_restart_preserve_phc() is tearing down / rebuilding the
 * MAC. servo_task must not issue esp_eth_ioctl PHC commands while this is set,
 * or it races the restart's PHC write on the other core. */
static _Atomic bool g_emac_restarting = false;

/* ─── OLED display ───
 * u8g2 is a non-static global: it's the display handle shared between
 * app_main (init), display_task, and every render_screen_* helper. */
u8g2_t u8g2;
static _Atomic uint8_t g_display_screen = 0; // 0..DISPLAY_NUM_SCREENS-1

/* ─── PTP identity / sequence counters ─── */
static uint8_t g_clock_id[8];
static uint8_t g_src_mac[6];

/* MAC-derived device identity.  Populated once in ethernet_init() and
 * installed on the netif immediately after esp_eth_start() in app_main.
 * Read-only after that, no locking needed.  Persists across cable
 * unplug/replug and DHCP restarts (netif property, not link property).
 * See components/mac_hostname/INTEGRATION.md for the full rationale.
 *
 *   g_hostname   - lower-dashed form given to DHCP/DNS/mDNS
 *                  (needs CONFIG_LWIP_MAX_HOSTNAME_LEN >= 64)
 *   g_host_line1 - "first-middle" (<= 20 chars) for the 21-col OLED
 *   g_host_line2 - "last"         (<= 12 chars) for the 21-col OLED    */
static char g_hostname[MAC_HOSTNAME_BUF_LEN];
static char g_host_line1[24];
static char g_host_line2[24];
static uint16_t g_sync_seq = 0;
static uint16_t g_announce_seq = 0;

/* ─── Sync-interval jitter instrumentation (read by web /api/health) ───
 * Records the spacing between consecutive Sync HW TX timestamps in a small
 * ring. The web layer computes min/max/mean/peak-to-peak on demand. Nominal
 * spacing is the Sync interval (1000 ms); spacing that widens or scatters
 * under load is the signature of ptp_tx_task being starved by the receive /
 * Delay_Resp path — the exact thing a flood test is looking for. */
#define SYNC_IVL_WIN 64
static int64_t g_sync_ivl_us[SYNC_IVL_WIN] = {0};
static uint32_t g_sync_ivl_head = 0;  /* next write index */
static uint32_t g_sync_ivl_count = 0; /* total intervals recorded (lifetime) */
static int64_t g_sync_prev_tx_ns = 0; /* previous Sync TX timestamp, ns */
static portMUX_TYPE g_sync_ivl_mux = portMUX_INITIALIZER_UNLOCKED;

/* Called once per Sync, with that Sync's HW TX timestamp. */
static void ptp_sync_jitter_note(const struct timespec *tx)
{
    int64_t now_ns = (int64_t)tx->tv_sec * 1000000000LL + tx->tv_nsec;
    portENTER_CRITICAL(&g_sync_ivl_mux);
    if (g_sync_prev_tx_ns != 0)
    {
        g_sync_ivl_us[g_sync_ivl_head] = (now_ns - g_sync_prev_tx_ns) / 1000;
        g_sync_ivl_head = (g_sync_ivl_head + 1) % SYNC_IVL_WIN;
        g_sync_ivl_count++;
    }
    g_sync_prev_tx_ns = now_ns;
    portEXIT_CRITICAL(&g_sync_ivl_mux);
}

/* Snapshot stats over the last min(count, SYNC_IVL_WIN) intervals (µs).
 * Returns lifetime interval count; *out_window is how many samples the
 * min/max/mean/last reflect. Called from the web layer (same TU). */
static uint32_t ptp_sync_jitter_stats(int64_t *out_min, int64_t *out_max,
                                      int64_t *out_mean, int64_t *out_last,
                                      uint32_t *out_window)
{
    int64_t local[SYNC_IVL_WIN];
    uint32_t count, head, n;
    portENTER_CRITICAL(&g_sync_ivl_mux);
    count = g_sync_ivl_count;
    head = g_sync_ivl_head;
    memcpy(local, g_sync_ivl_us, sizeof(local));
    portEXIT_CRITICAL(&g_sync_ivl_mux);

    n = (count < SYNC_IVL_WIN) ? count : SYNC_IVL_WIN;
    if (n == 0)
    {
        *out_min = *out_max = *out_mean = *out_last = 0;
        *out_window = 0;
        return 0;
    }
    int64_t mn = INT64_MAX, mx = INT64_MIN, sum = 0;
    for (uint32_t i = 0; i < n; i++)
    {
        int64_t v = local[(head + SYNC_IVL_WIN - 1 - i) % SYNC_IVL_WIN];
        if (v < mn)
            mn = v;
        if (v > mx)
            mx = v;
        sum += v;
    }
    *out_min = mn;
    *out_max = mx;
    *out_mean = sum / (int64_t)n;
    *out_last = local[(head + SYNC_IVL_WIN - 1) % SYNC_IVL_WIN];
    *out_window = n;
    return count;
}

/* ─── GNSS time + PPS edge state ─── */
static volatile uint64_t g_last_pps_phc_ns = 0;
static time_t g_gnss_seconds = 0;
static bool g_gnss_valid = false;
static int g_utc_offset = 37; // TAI - UTC (update if leap second changes)
static portMUX_TYPE g_gnss_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ISR-captured PPS PHC timestamp (sampled inside the PPS ISR).
 * Written by pps_isr_handler, read by pps_task. On the dual-core P4 these can
 * run on different cores simultaneously; g_pps_capture_lock serializes the
 * 64-bit ns value with its sequence counter so the read cannot tear. */
static portMUX_TYPE g_pps_capture_lock = portMUX_INITIALIZER_UNLOCKED;
static uint64_t g_pps_capture_ns = 0;
static uint32_t g_pps_capture_seq = 0;

static SemaphoreHandle_t g_pps_sem;

/* ─── Servo working state ───
 * 64-bit values shared between pps_task and servo_task are guarded by
 * g_servo_spinlock. */
static portMUX_TYPE g_servo_spinlock = portMUX_INITIALIZER_UNLOCKED;
static double g_integral = 0;
static double g_freq_ppb = 0;
/* Last frequency adjustment ACTUALLY WRITTEN to the PHC, in ppb. Lock-free
 * mirror for emac_restart_preserve_phc (watchdog task): g_freq_ppb is a
 * software double (two 32-bit stores on RV32) written by servo_task WITHOUT
 * any lock, so a cross-core reader can tear it — and what the restart needs
 * to re-apply is precisely the int32 the hardware last received, not the
 * servo's internal double. Updated only when the ADJ ioctl actually went
 * through (servo_phc_ioctl returned true). Deliberately NOT reset on
 * re-acquire: the hardware keeps its last adjustment until the next ADJ
 * write, and this mirror tracks the hardware. */
static _Atomic int32_t g_freq_ppb_hw = 0;
static volatile int64_t g_last_offset = 0;
static volatile uint32_t g_pps_edge_count = 0;
/* PPS edges that no-fix coasting in pps_task did NOT label as a plain "+1 s"
 * (lifetime, never reset). Written only by pps_task inside g_servo_spinlock;
 * read by httpd as single aligned 32-bit loads. Diagnostic only: nothing
 * reads them back into the timing core.
 *   g_pps_coast_extra   - edges labelled with the same (or an earlier) second
 *                         than the previous edge: extra / glitch pulses.
 *   g_pps_coast_missing - whole seconds skipped between two labelled edges:
 *                         missing pulses.
 * Glitches while the fix is valid are handled by the GNSS path and are not
 * counted here. */
static volatile uint32_t g_pps_coast_extra = 0;
static volatile uint32_t g_pps_coast_missing = 0;
/* True when the most recently processed PPS edge is TRUSTED for deciding
 * which second is which: it arrived ~1 s (+/- PPS_REGULAR_TOL_US, monotonic
 * esp_timer, immune to PHC steps) after the previous edge, AND the GNSS time
 * seen at it is valid and exactly 1 s later than at the previous edge. Written
 * by pps_task inside g_servo_spinlock; servo_task snapshots it together with
 * g_last_pps_phc_ns. The coarse step and the sentence-offset derivation only
 * use trusted edges, so neither a glitch pulse nor a single bad-time sentence
 * can fix the PHC on the wrong second for the rest of the uptime. */
#define PPS_REGULAR_TOL_US 10000
static volatile bool g_last_pps_trusted = false;

/* Lifetime count of rejected PPS samples since boot. Persistent diagnostic
 * shown on OLED screen 2: lets you tell at a glance whether the board has
 * rejected anything (e.g. during an unattended holdover recovery) even after
 * the streak self-healed back to zero. Never reset on re-acquire — boot only. */
static volatile uint32_t g_total_rejects = 0;

/* The PHC is the clock: it alone says which second it is. GNSS corrects its
 * phase (sub-second, every edge) and checks its second (seconds checker in
 * pps_task); it never relabels edges. g_phc_stepped = the PHC holds real time
 * (set by the COARSE STEP). */
static volatile bool g_phc_stepped = false;
static volatile bool g_phc_residual_measured = false;

/* Seconds between the time carried by the latest RMC and the PPS edge being
 * labelled. This is a property of the receiver, not something to derive at run
 * time: the TAU1201 emits RMC ~0.2 s AFTER the PPS edge it describes (standard
 * NMEA; measured against an NTP-synchronised host on 2026-09-30), so an edge is
 * (latest RMC second) + 1. The former derivation from floor(PHC) right after
 * the coarse step returned 0 on this receiver, which put the PHC exactly 1 s
 * behind TAI. Use 0 only for a receiver that sends the time of the NEXT edge
 * before that edge. */
#define GNSS_SENTENCE_OFFSET 1

/* Seconds checker (pps_task). On every fresh fix + trusted edge the receiver's
 * second is compared with the PHC's. Any disagreement makes the clock advertise
 * class 7 at once (it may be either side that is wrong). The PHC is stepped by
 * whole seconds only after the SAME difference persisted this many checks:
 * short before the first lock (the PHC has no history to trust yet), long after
 * it (a receiver can report a wrong UTC second for minutes after an outage --
 * 1 s seen on the bench -- while the free-running PHC keeps the right one). */
#define SECONDS_CONFIRM_BOOT_S 10
#define SECONDS_CONFIRM_LOCKED_S 900
static volatile bool g_seconds_agree = true;       /* last check agreed */
static volatile int32_t g_seconds_diff = 0;        /* receiver second - PHC second (0 = agree) */
static volatile uint32_t g_seconds_disagree_n = 0; /* consecutive checks with this difference */
static volatile int32_t g_seconds_fix_req = 0;     /* whole seconds servo_task is to step the PHC by */
static volatile uint32_t g_phc_step_gen = 0;       /* bumped on every PHC set: second history restarts */
static volatile bool g_last_pps_plausible = true;  /* edge's PHC second continues the previous ones */
static volatile bool g_last_pps_regular = false;   /* edge came ~1 s after the previous one */
static volatile int64_t g_phc_offset_ns = 0; /* measured steady-state offset */

/* Live reject-streak counter for OLED screen 2. Mirrors servo_task's
 * consecutive HARD_REJECT count so the display can warn before a wedge would
 * force re-acquire. Written by servo_task, read by render_screen_servo. */
static volatile uint32_t g_consecutive_rejects = 0;

/* Servo offset median filter (sliding window). */
static int64_t g_offset_history[MEDIAN_WINDOW] = {0};
static int g_offset_history_count = 0;
static int g_offset_history_idx = 0;

/* Long-window running-mean averager (the servo's working signal). */
static int64_t g_avg_ring[AVG_WINDOW] = {0};
static int64_t g_avg_sum = 0;
static int g_avg_count = 0;
static int g_avg_idx = 0;

/* Derivative-term state: ring buffer of recent averaged offsets. The D term
 * is the slope of avg over the last SERVO_D_WINDOW samples. Smoothing the
 * derivative this way is the standard way to avoid the "D amplifies noise"
 * problem in PID controllers. (Paired with the averager — see avg_reset.) */
static int64_t g_d_ring[SERVO_D_WINDOW] = {0};
static int g_d_count = 0;
static int g_d_idx = 0;

/* ─── Lock-quality + raw-spread tracking (OLED screen 2) ─── */
static uint8_t g_lock_qual_ring[LOCK_QUAL_WINDOW] = {0};
static uint8_t g_lock_qual_idx = 0;
static uint8_t g_lock_qual_count = 0;
/* Recent raw-offset range, updated alongside the quality ring. Used by
 * screen 2 to show "PPS jitter span" without needing another data structure. */
static int64_t g_qual_raw_min = INT64_MAX;
static int64_t g_qual_raw_max = INT64_MIN;
static uint8_t g_qual_raw_samples = 0; /* counter so we reset bounds periodically */

/* ─── Holdover / post-holdover settling ─── */
static time_t g_last_gnss_update = 0;
/* esp_timer ms of the last RMC that carried a valid fix (written with
 * g_gnss_seconds under g_gnss_spinlock). pps_task treats the GNSS time as
 * invalid once it is older than GNSS_FRESH_MS: g_gnss_valid alone never
 * expires, so a silent NMEA stream (UART fault, receiver hang) with PPS still
 * running used to feed a frozen second into the reference. One RMC per
 * second puts the age at a PPS edge near 0.8 s; 1.5 s also coasts through a
 * single dropped sentence instead of labelling that edge a second late. */
#define GNSS_FRESH_MS 1500
static volatile int64_t g_last_rmc_ms = 0;
static bool g_holdover = true;
static volatile int64_t g_lock_acquired_ms = 0; /* esp_timer ms when SERVO_LOCKED entered */
static volatile bool g_settled_after_holdover = false;
/* Phase convergence as judged by servo_task (single writer; bools so other
 * tasks never read a torn 64-bit offset). clockClass 6 is only (re)declared
 * once the averaged phase is inside CLASS6_MAX_OFFSET_NS with a full averager
 * and no rejects, and is withdrawn while it is beyond CLASS6_DEMOTE_NS or the
 * servo is rejecting samples: a timer alone let the box advertise class 6
 * while 100-300 us off after a holdover. */
#define CLASS6_MAX_OFFSET_NS 10000LL
#define CLASS6_DEMOTE_NS 50000LL
#define CLASS6_DEMOTE_REJECTS 5
static volatile bool g_phase_converged = false;
static volatile bool g_phase_degraded = false;

/* Latches true the first time the servo reaches SERVO_LOCKED with valid GNSS
 * time since boot. Until then the PHC has never held real time, so we must
 * advertise clockClass 248 (never-synchronized) rather than 7 (holdover).
 * Once latched it stays true for the rest of this power cycle: a clock that
 * was once locked and is now coasting IS legitimately in holdover. Not reset
 * on re-acquire or holdover entry — those are coasting on a known-good clock. */
static volatile bool g_ever_locked = false;

/* ─── BMCA ─── */
static bmca_dataset_t g_best_master = {0};
/* Boots NOT master (PTP LISTENING): nothing is transmitted until either a
 * better master has been heard (stay PASSIVE) or no Announce has been heard
 * for ANNOUNCE_TIMEOUT_SEC since the link came up (bmca_timeout_check ->
 * MASTER). Booting as MASTER made every reboot transmit Sync with a PHC still
 * at 1970 until three Announces from the other grandmaster had been heard. */
static bool g_i_am_master = false;
static bmca_dataset_t g_candidate_master = {0};
static int g_candidate_count = 0;
static time_t g_last_announce_rx = 0;

/* Runtime-settable priority1/priority2 (POST /api/bmca/priority). RAM-only,
 * NOT persisted to NVS: a reboot restores the defaults. Deliberate — NVS
 * writes stall both cores' caches (rule 1.2.2 of the BMCA instrumentation
 * handover), and NVS is not even initialised in this firmware. */
#define PTP_PRIORITY1_DEFAULT 128
#define PTP_PRIORITY2_DEFAULT 128
static _Atomic uint8_t g_priority1 = PTP_PRIORITY1_DEFAULT;
static _Atomic uint8_t g_priority2 = PTP_PRIORITY2_DEFAULT;

/* Packed copy of the dataset last placed on the wire by send_announce():
 * clock_class | accuracy<<8 | priority1<<16 | priority2<<24. Lets httpd
 * report bmca.local while MASTER without calling build_local_dataset() (see
 * the no-side-effects rule on advertised_clock_class()). 0 until the first
 * Announce is sent (~2 s after boot). */
static _Atomic uint32_t g_local_ds_packed = 0;

/* Set (if ever enabled) at the commented-out ETH_CMD_S_PROMISCUOUS site in
 * app_main. Real state for the JSON "promiscuous" field. */
static bool g_promiscuous = false;

/* ─── Slave tracking ─── */
static slave_track_t g_slaves[SLAVE_TRACK_MAX] = {0};
static portMUX_TYPE g_slaves_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ─── Delay_Resp producer/consumer + L2TAP TX serialization ───
 * g_delay_resp_q decouples Delay_Req receipt (ptp_rx_task) from Delay_Resp
 * transmission (delay_resp_task), so a Delay_Req flood can't stall the
 * receive loop or starve Sync/Announce. g_l2tap_tx_mtx serializes writes to
 * the shared g_l2tap_fd now that two tasks (delay_resp_task and ptp_tx_task)
 * transmit concurrently on the dual-core P4. g_delay_resp_drops counts items
 * dropped when the queue is full (graceful: the slave just loses one sample). */
static QueueHandle_t g_delay_resp_q = NULL;
static SemaphoreHandle_t g_l2tap_tx_mtx = NULL;
static _Atomic uint32_t g_delay_resp_drops = 0;
#define DELAY_RESP_Q_DEPTH 64

/* ─── LED status ─── */
/* Single shared state — written by app code, read by LED task. _Atomic
 * gives lock-free safe reads/writes on a 32-bit value. */
static _Atomic led_state_t s_led_state = LED_STATE_NO_LINK;

/* ─── Display / uptime bookkeeping ─── */
static volatile int64_t g_lock_start_ms = 0; /* when servo entered LOCKED (lock-duration display) */
static volatile int64_t g_boot_time_ms = 0;  /* boot time (uptime display) */

/* ─── GNSS signal-quality public state ───
 * Read by OLED screen 3 and gnss_signal_task; populated by parse_gsv /
 * parse_gsa / gsv_publish_snapshot. */
static portMUX_TYPE g_gnss_sig_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_gnss_snr_avg = 0;
static uint8_t g_gnss_snr_min = 0;
static uint8_t g_gnss_snr_max = 0;
static uint8_t g_gnss_sv_in_view = 0;
static uint8_t g_gnss_sv_used = 0;
static int64_t g_gnss_sig_last_update_ms = 0;

/* One GSV SNR accumulator per talker (see talker_index for the mapping). */
static gsv_acc_t g_gsv_acc[GSV_TALKER_COUNT];

/* Latest receiver-reported GSV total for each talker.
 *
 * If a fresh $GNGSV combined report exists, it takes precedence over the
 * individual GP/GL/GA/GB reports so satellites are not counted twice.
 */
#define GSV_COUNT_FRESH_MS 2500

static uint8_t g_gsv_view_by_talker[GSV_TALKER_COUNT] = {0};
static int64_t g_gsv_view_seen_ms[GSV_TALKER_COUNT] = {0};

/* Used to prevent an old GGA count remaining displayed if GGA stops. */
static int64_t g_gga_last_update_ms = 0;

/* PPS edge-to-edge jitter tracking. Captures the receiver's raw timing
 * noise BEFORE the servo processes it. pps_task pushes one sample per edge;
 * gnss_signal_task reads and zeroes the accumulators every 30 s.
 *
 * The sample is (interval_ns - 1,000,000,000): zero = perfect 1 Hz cadence,
 * non-zero = receiver PPS jitter (plus a small contribution from PHC
 * frequency error, which is sub-ppb once locked and therefore negligible).
 *
 * Tracked as min/max/sum_of_abs/count rather than a ring buffer so we don't
 * burn 480 bytes of RAM and stddev arithmetic for each report. */
static portMUX_TYPE g_pps_jit_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t g_pps_jit_min_ns = INT64_MAX;
static int64_t g_pps_jit_max_ns = INT64_MIN;
static int64_t g_pps_jit_sum_abs_ns = 0; /* sum of |interval-1e9| for mean-abs */
static uint32_t g_pps_jit_count = 0;
static uint32_t g_pps_jit_outliers = 0; /* samples beyond ±100 µs */

/* SV churn — set of PRNs currently used in the fix, plus enter/exit counters
 * over the report window. Updated by parse_gsa. */
static portMUX_TYPE g_sv_churn_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_sv_entries = 0;  /* PRNs newly added to fix this window */
static uint32_t g_sv_exits = 0;    /* PRNs newly removed from fix this window */
static uint32_t g_sv_gsa_seen = 0; /* GSA sentences observed this window */

/* Per-talker active-SV bitmap, for churn detection. Same talker indexing as
 * g_gsv_acc (talker_index). PRNs above SV_CHURN_PRN_MAX are silently dropped
 * — covers all current civilian GNSS signals. */
static uint8_t g_sv_active_per_talker[GSV_TALKER_COUNT][SV_CHURN_PRN_MAX / 8];

/* ─── Health monitoring / watchdog progress tracking ─── */
static volatile uint64_t g_last_progress_phc_s = 0;
static volatile uint64_t g_last_progress_check_ms = 0;
static volatile uint32_t g_emac_restart_count = 0;

/* Heap baseline for the JSON heap_leak field — promoted out of
 * heap_monitor_task's local so status_handler can read it. 0 = not yet
 * measured (the task waits 10 s after boot before taking it). */
static _Atomic uint32_t g_heap_baseline_free = 0;

/* ─── Watchdog health, for the JSON "wdt" object ───
 * Written only by watchdog_task, read by httpd. last_alert_uptime_s is
 * uptime in whole SECONDS (not ms): _Atomic int64_t is not lock-free on this
 * 32-bit RISC-V core, and a torn 64-bit read here would show a garbage
 * "last alert" age on the dashboard. */
typedef enum
{
    WDT_ARMING = 0,
    WDT_OK,
    WDT_PHC_FROZEN,
    WDT_HEAP_CRITICAL,
    WDT_RESTART_LIMIT
} wdt_state_t;
static _Atomic uint8_t g_wdt_state = WDT_ARMING;
static _Atomic uint32_t g_wdt_alerts = 0;         /* count of checks that found a problem */
static _Atomic uint32_t g_wdt_last_alert_s = 0;   /* uptime seconds, 0 = never */

/* ─── Task handle ─── */
static TaskHandle_t servo_task_handle = NULL;

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Forward declarations
 * ═════════════════════════════════════════════════════════════════════════ */
static int bmca_compare(const bmca_dataset_t *a, const bmca_dataset_t *b, bmca_field_t *field);
static void announce_to_dataset(const ptp_announce_msg_t *ann, bmca_dataset_t *ds);
static void bmca_update(const bmca_dataset_t *remote);
static void handle_announce(const uint8_t *payload, size_t len);
static const char *bmca_field_str(uint8_t f);
static const char *bmca_reason_str(uint8_t r);
static void emac_get_time(struct timespec *ts);
static void pps_isr_handler(void *arg);
static time_t my_timegm(struct tm *tm);
static void median_filter_reset(void);
static esp_err_t l2tap_init(void);
static bool phc_start_blocking(void);
static bool timing_core_bringup(void);
static void apply_static_ip(esp_netif_t *netif);

static void display_task(void *arg);
static void button_task(void *arg);
static void render_screen_main(void);
static void render_screen_servo(void);
static void render_screen_gnss(void);
static void render_screen_network(void);
static void render_screen_timing_cfg(void);
static void slave_track_seen(const uint8_t *mac);
static int slave_active_count(void);
static void enqueue_delay_req(const uint8_t *ptp_payload, size_t len,
                              const struct timespec *rx_hw_ts);
static void delay_resp_task(void *arg);

/* LED status API (public, non-static) — defined in the LED section, which
 * appears after the servo/timing core that calls led_set_state. */
void led_set_state(led_state_t new_state);
void led_start(void);

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Task stack registry
 *
 * uxTaskGetSystemState()/TaskStatus_t exposes each task's high-water mark
 * but NOT the stack size it was created with, so the JSON "stack_bytes"
 * field (used by the dashboard to compute % stack used) has to come from
 * somewhere else. task_create_tracked() is a drop-in xTaskCreate() that also
 * records the requested stack size here; status_handler looks it up by name.
 * Tasks this file doesn't create itself ("main", "httpd") are registered
 * directly with task_reg_add(). Unknown names resolve to JSON null — never
 * an invented number.
 * ═════════════════════════════════════════════════════════════════════════ */
#define TASK_REG_MAX 24
typedef struct
{
    char name[configMAX_TASK_NAME_LEN];
    uint32_t stack_bytes;
} task_reg_t;
static task_reg_t g_task_reg[TASK_REG_MAX];
static _Atomic uint32_t g_task_reg_n = 0;

static void task_reg_add(const char *name, uint32_t stack_bytes)
{
    uint32_t i = atomic_load(&g_task_reg_n);
    if (i >= TASK_REG_MAX)
        return;
    strlcpy(g_task_reg[i].name, name, sizeof(g_task_reg[i].name));
    g_task_reg[i].stack_bytes = stack_bytes;
    atomic_store(&g_task_reg_n, i + 1); /* publish after the entry is written */
}

static BaseType_t task_create_tracked(TaskFunction_t fn, const char *name,
                                      uint32_t stack_bytes, void *arg,
                                      UBaseType_t prio, TaskHandle_t *out)
{
    task_reg_add(name, stack_bytes);
    return xTaskCreate(fn, name, stack_bytes, arg, prio, out);
}

static long task_reg_lookup(const char *name) /* -1 => unknown => JSON null */
{
    uint32_t n = atomic_load(&g_task_reg_n);
    for (uint32_t i = 0; i < n; i++)
        if (strcmp(g_task_reg[i].name, name) == 0)
            return (long)g_task_reg[i].stack_bytes;
    return -1;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Small helpers
 *
 * Self-contained utilities used across the file: UTC time conversion, servo
 * filter resets, and the holdover-advertise / local-BMCA-dataset builders.
 * ═════════════════════════════════════════════════════════════════════════ */

static time_t my_timegm(struct tm *tm)
{
    /* Pure-arithmetic UTC conversion. No heap allocation, no newlib env
     * or timezone bookkeeping (which leaks small bytes per setenv/tzset
     * cycle in ESP-IDF's newlib build).
     *
     * Computes seconds since 1970-01-01 00:00:00 UTC from a struct tm
     * already expressed in UTC. Same semantics as the old function. */
    static const int days_in_month[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    int year = tm->tm_year + 1900;
    int mon = tm->tm_mon;   /* 0-11 */
    int mday = tm->tm_mday; /* 1-31 */

    /* Defensive: never index days_in_month[] out of range, no matter what a
     * caller passes. A bad month means a bad sentence; clamp rather than read
     * out of bounds. Callers are expected to range-check first (see parse_rmc
     * / parse_zda), this is belt-and-suspenders. */
    if (mon < 0)
        mon = 0;
    else if (mon > 11)
        mon = 11;
    int hour = tm->tm_hour;
    int min = tm->tm_min;
    int sec = tm->tm_sec;

    /* Days from 1970-01-01 to Jan 1 of `year` */
    long days = 0;
    for (int y = 1970; y < year; y++)
    {
        days += 365;
        if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0))
            days += 1; /* leap year */
    }

    /* Days from Jan 1 to start of `mon` in `year` */
    for (int m = 0; m < mon; m++)
    {
        days += days_in_month[m];
        if (m == 1 &&
            ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)))
            days += 1; /* February in leap year */
    }

    /* Add day of month (1-based) */
    days += mday - 1;

    /* Convert to seconds since epoch */
    return (time_t)days * 86400 + hour * 3600 + min * 60 + sec;
}

/* CURRENTLY UNUSED: the median filter arrays are never written or read
 * outside this reset (grep for g_offset_history). Kept in place — along
 * with the calls scattered through servo_task — as dormant scaffolding
 * for a possible future first-stage filter. See the "Servo offset
 * filtering windows" comment block in the configuration section. */
static void median_filter_reset(void)
{
    g_offset_history_count = 0;
    g_offset_history_idx = 0;
    for (int i = 0; i < MEDIAN_WINDOW; i++)
        g_offset_history[i] = 0;
}

static inline void avg_reset(void)
{
    for (int i = 0; i < AVG_WINDOW; i++)
        g_avg_ring[i] = 0;
    g_avg_sum = 0;
    g_avg_count = 0;
    g_avg_idx = 0;
    /* Reset D state too — they're conceptually paired (both derived
     * from the running mean). */
    for (int i = 0; i < SERVO_D_WINDOW; i++)
        g_d_ring[i] = 0;
    g_d_count = 0;
    g_d_idx = 0;
    /* an empty averager says nothing about convergence */
    g_phase_converged = false;
    g_phase_degraded = false;
}

/* Read-only: should the clock advertise holdover (class 7) right now? Used by
 * the display mirrors as-is; advertise_as_holdover() adds the latch. When
 * can_latch is non-NULL it is set true once the settle window has passed AND
 * the phase has converged. */
static inline bool holdover_advert_ro(bool *can_latch)
{
    if (can_latch)
        *can_latch = false;
    if (g_holdover)
        return true;
    if (!g_seconds_agree)
        return true; /* receiver and PHC disagree about the second */
    if (!g_settled_after_holdover)
    {
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (g_lock_acquired_ms == 0)
            return true; /* never locked yet / phase being realigned */
        if (now_ms - g_lock_acquired_ms < POST_HOLDOVER_SETTLE_MS)
            return true;
        if (!g_phase_converged)
            return true; /* settle time passed but the phase is not there yet */
        if (can_latch)
            *can_latch = true;
        return false;
    }
    return g_phase_degraded; /* settled, but currently off / rejecting */
}

static inline bool advertise_as_holdover(void)
{
    bool latch = false;
    bool h = holdover_advert_ro(&latch);
    if (latch)
        g_settled_after_holdover = true;
    return h;
}

/* PTP clockClass selection (IEEE 1588-2008 Table 5):
 *   248 — DEFAULT: clock has never synchronized to its primary reference.
 *           Slaves treat this as unsynchronized / lowest-trust.
 *     7 — HOLDOVER: WAS locked to GNSS, now coasting within holdover spec.
 *     6 — LOCKED:   actively disciplined to GNSS.
 * The 248 state is mutually exclusive with 6/7 and takes precedence: a clock
 * that has never been correct cannot be "in holdover". */
static inline uint8_t advertised_clock_class(void)
{
    if (!g_ever_locked)
        return 248;
    return advertise_as_holdover() ? 7 : 6;
}

static void build_local_dataset(bmca_dataset_t *ds)
{
    ds->priority1 = atomic_load(&g_priority1);

    // Dynamic depending on GNSS lock + post-holdover settling + never-locked
    uint8_t cc = advertised_clock_class();
    ds->clock_class = cc;
    /* accuracy: locked → 0x21 (~100 ns), holdover → 0x23, unknown → 0xFE */
    ds->accuracy = (cc == 6) ? 0x21 : (cc == 7) ? 0x23
                                                : 0xFE;

    ds->variance = 0xFFFF;
    ds->priority2 = atomic_load(&g_priority2);
    memcpy(ds->clock_id, g_clock_id, 8);
    ds->steps_removed = 0; /* we ARE the grandmaster of this dataset */
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: EMAC PHC + PTP wire helpers
 *
 * Reading the hardware PHC clock and (de)serializing PTP wire structures.
 * emac_get_time is ISR-safe and used from the PPS ISR. l2tap_send_ptp wraps
 * a PTP payload in an Ethernet frame and transmits it via the L2TAP fd,
 * optionally returning the hardware TX timestamp.
 * ═════════════════════════════════════════════════════════════════════════ */

static IRAM_ATTR void emac_get_time(struct timespec *ts)
{
    eth_mac_time_t t1 = {0}, t2 = {0};

    /*
     * Detect torn reads across the second-boundary register update.
     * The hardware updates the seconds register a few ns after the
     * nanoseconds counter rolls over to 0, so reading tv_nsec=N
     * (large) with tv_sec=M can race against tv_nsec=0 with tv_sec=M
     * (when the hardware really meant tv_sec=M+1).
     *
     * IMPORTANT: This function is called from the PPS ISR. The
     * ETH_MAC_ESP_CMD_G_PTP_TIME path in IDF 5.5.4 reads PHC registers
     * directly via the HAL with no locks, so it's ISR-safe.
     */
    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_G_PTP_TIME, &t1);
    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_G_PTP_TIME, &t2);

    if (t2.seconds < t1.seconds ||
        (t2.seconds == t1.seconds && t2.nanoseconds < t1.nanoseconds))
    {
        eth_mac_time_t t3 = {0};
        esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_G_PTP_TIME, &t3);
        ts->tv_sec = t3.seconds;
        ts->tv_nsec = t3.nanoseconds;
        return;
    }

    ts->tv_sec = t2.seconds;
    ts->tv_nsec = t2.nanoseconds;
}

static void encode_ptp_timestamp(ptp_timestamp_t *dst, const struct timespec *src)
{
    uint64_t sec = (uint64_t)src->tv_sec;
    uint32_t ns = (uint32_t)src->tv_nsec;
    dst->seconds_msb[0] = (sec >> 40) & 0xFF;
    dst->seconds_msb[1] = (sec >> 32) & 0xFF;
    dst->seconds_lsb[0] = (sec >> 24) & 0xFF;
    dst->seconds_lsb[1] = (sec >> 16) & 0xFF;
    dst->seconds_lsb[2] = (sec >> 8) & 0xFF;
    dst->seconds_lsb[3] = (sec >> 0) & 0xFF;
    dst->nanoseconds[0] = (ns >> 24) & 0xFF;
    dst->nanoseconds[1] = (ns >> 16) & 0xFF;
    dst->nanoseconds[2] = (ns >> 8) & 0xFF;
    dst->nanoseconds[3] = (ns >> 0) & 0xFF;
}

static void build_ptp_header(ptp_header_t *h, uint8_t msg_type, uint16_t msg_len,
                             uint16_t seq_id, uint16_t flags,
                             int8_t log_interval, uint8_t control)
{
    memset(h, 0, sizeof(*h));
    h->msg_type_transport = (msg_type & 0x0F);
    h->version_ptp = 0x02;
    h->msg_length = htons(msg_len);
    h->domain_number = PTP_DOMAIN;
    h->flags = htons(flags);
    memcpy(h->source_port_id.clock_identity, g_clock_id, 8);
    h->source_port_id.port_number = htons(1);
    h->sequence_id = htons(seq_id);
    h->control = control;
    h->log_msg_interval = log_interval;
}

/* ─────────────────────────────────────────────────────────────────────────
 * L2TAP Extended Buffer Send
 * ───────────────────────────────────────────────────────────────────────── */
static int l2tap_send_ptp(const void *ptp_payload, size_t ptp_len, struct timespec *tx_ts)
{
    /* Fast-reject while emac_restart_preserve_phc() is mid-restart. Mirrors
     * the servo_phc_ioctl pattern (same atomic-flag fast-reject): a write
     * through the L2TAP fd to a stopped EMAC is at best lost and at worst
     * races the driver's internal descriptor teardown. The restart function
     * ALSO takes g_l2tap_tx_mtx around its esp_eth_stop / esp_eth_start
     * window, so a caller that passed this atomic check just before the
     * restart began will drain its in-flight write() before stop() is
     * called. */
    if (atomic_load(&g_emac_restarting))
        return -1;

    size_t frame_len = 14 + ptp_len;

    /* No heap on the hot path. Largest frame we ever send is an Announce
     * (14 + 64 = 78 bytes); 14 + 96 leaves comfortable headroom. */
    uint8_t frame[14 + 96];
    if (frame_len > sizeof(frame))
        return -1;
    memcpy(frame, PTP_MCAST_MAC, 6);
    memcpy(frame + 6, g_src_mac, 6);
    frame[12] = (ETH_TYPE_PTP >> 8) & 0xFF;
    frame[13] = ETH_TYPE_PTP & 0xFF;
    memcpy(frame + 14, ptp_payload, ptp_len);

    // Stack-allocated IREC buffer (as per official example)
    union
    {
        uint8_t buf[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } irec_u;
    memset(&irec_u, 0, sizeof(irec_u));

    l2tap_irec_hdr_t *irec = (l2tap_irec_hdr_t *)irec_u.buf;
    irec->type = L2TAP_IREC_TIME_STAMP;
    irec->len = L2TAP_IREC_LEN(sizeof(struct timespec));

    l2tap_extended_buff_t ext = {
        .info_recs_len = sizeof(irec_u.buf),
        .info_recs_buff = irec_u.buf,
        .buff_len = frame_len,
        .buff = frame,
    };

    /* Serialize the shared g_l2tap_fd: delay_resp_task and ptp_tx_task can
     * both reach here concurrently on the dual-core P4, and emac_restart_preserve_phc
     * also takes this mutex around its stop/start. The fd is O_NONBLOCK,
     * so write() returns immediately (EAGAIN if EMAC TX descriptors are full),
     * keeping the hold time tiny. The TX timestamp returns in irec for THIS
     * call's frame, so it must be read while still holding the lock. */
    int ret;
    if (g_l2tap_tx_mtx)
        xSemaphoreTake(g_l2tap_tx_mtx, portMAX_DELAY);

    // TX timestamp is returned synchronously in irec after write()
    ret = write(g_l2tap_fd, &ext, 0);

    // ret > 0 means success AND timestamp is valid
    if (ret > 0 && tx_ts != NULL && irec->type == L2TAP_IREC_TIME_STAMP)
    {
        memcpy(tx_ts, irec->data, sizeof(struct timespec));
        // ESP_LOGI("PTP_TX", "TX timestamp: %llds %09ldns",
        //          (long long)tx_ts->tv_sec, (long)tx_ts->tv_nsec);
    }
    else if (ret < 0)
    {
        // ESP_LOGE(TAG, "l2tap write FAILED errno=%d (%s)", errno, strerror(errno));
    }

    if (g_l2tap_tx_mtx)
        xSemaphoreGive(g_l2tap_tx_mtx);

    return ret;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: BMCA (Best Master Clock Algorithm)
 *
 * bmca_better is the dataset comparator. announce_to_dataset unpacks a
 * received Announce into a comparison dataset. bmca_update runs the
 * master-vs-remote decision. handle_announce applies a stability filter
 * (require BMCA_STABILITY_THRESHOLD consistent observations of the same
 * clock identity) before letting a remote dataset influence the decision.
 * ═════════════════════════════════════════════════════════════════════════ */

/* Returns <0 if a is better than b, >0 if b is better, 0 if neither is better
 * (same grandmaster AND same stepsRemoved). *field (optional) receives the
 * comparison step that decided the result. Semantics are EXACTLY those of
 * the previous bmca_better(): see the Figure 27/28 commentary below. */
static int bmca_compare(const bmca_dataset_t *a, const bmca_dataset_t *b,
                        bmca_field_t *field)
{
    /* IEEE 1588-2008 data set comparison. When both datasets name the SAME
     * grandmaster ("part 2", Figure 28), every GM-quality field is by
     * definition equal and the decision falls to stepsRemoved — the path
     * with fewer hops to that GM wins. This doubles as the anti-loop
     * guard: an Announce relaying OUR OWN clock as grandmaster (a boundary
     * clock echoing us back) carries stepsRemoved >= 1 and can never beat
     * the local dataset's 0, so we keep mastership instead of going
     * passive to our own reflection. (The full Figure 28 additionally
     * tiebreaks on sender/receiver port identity when stepsRemoved are
     * within 1 of each other; with a single port and no foreign-master
     * table, "not better" — keep the current state — is the safe
     * resolution of that tie.)
     *
     * With DIFFERENT grandmasters ("part 1", Figure 27), stepsRemoved is
     * deliberately NOT compared: the standard decides purely on GM quality
     * — priority1, class, accuracy, variance, priority2, then identity. */
    bmca_field_t f = BMCA_FIELD_NONE;
    int r = 0;

    if (memcmp(a->clock_id, b->clock_id, 8) == 0)
    {
        f = BMCA_FIELD_STEPS_REMOVED;
        if (a->steps_removed != b->steps_removed)
            r = (a->steps_removed < b->steps_removed) ? -1 : 1;
        /* equal stepsRemoved: r = 0 ("not better" — keep current state) */
    }
    else if (a->priority1 != b->priority1)     { f = BMCA_FIELD_PRIORITY1;    r = (a->priority1   < b->priority1)   ? -1 : 1; }
    else if (a->clock_class != b->clock_class) { f = BMCA_FIELD_CLOCK_CLASS;  r = (a->clock_class < b->clock_class) ? -1 : 1; }
    else if (a->accuracy != b->accuracy)       { f = BMCA_FIELD_ACCURACY;     r = (a->accuracy    < b->accuracy)    ? -1 : 1; }
    else if (a->variance != b->variance)       { f = BMCA_FIELD_VARIANCE;     r = (a->variance    < b->variance)    ? -1 : 1; }
    else if (a->priority2 != b->priority2)     { f = BMCA_FIELD_PRIORITY2;    r = (a->priority2   < b->priority2)   ? -1 : 1; }
    else
    {
        f = BMCA_FIELD_CLOCK_IDENTITY;
        r = (memcmp(a->clock_id, b->clock_id, 8) < 0) ? -1 : 1; /* ids differ here */
    }

    if (field)
        *field = f;
    return r;
}

static void announce_to_dataset(const ptp_announce_msg_t *ann, bmca_dataset_t *ds)
{
    ds->priority1 = ann->grandmaster_priority1;
    ds->clock_class = ann->grandmaster_clock_class;
    ds->accuracy = ann->grandmaster_clock_accuracy;
    ds->variance = ntohs(ann->grandmaster_clock_variance);
    ds->priority2 = ann->grandmaster_priority2;
    memcpy(ds->clock_id, ann->grandmaster_identity, 8);
    ds->steps_removed = ntohs(ann->steps_removed);
}

/* Single choke point for every write to g_i_am_master from ptp_rx_task's call
 * tree (bmca_update() and the announce-timeout reclaim). Records a BMCA
 * event into the ring + lifetime counter through the seqlock; a no-op role
 * write (to_master == current) records nothing. No logging here — this runs
 * on the receive hot path (rule 1.2.1); bmca_log_task drains the ring. */
static void bmca_set_role(bool to_master, bmca_reason_t why,
                          const uint8_t *peer_id, bmca_field_t decided_by)
{
    if (g_i_am_master == to_master)
        return; /* not a change: record nothing */

    g_i_am_master = to_master;

    int64_t now_ms = esp_timer_get_time() / 1000 - g_boot_time_ms;

    bmca_view_write_begin();
    bmca_event_t *e = &g_bmca_view.ev[g_bmca_view.ev_next];
    e->t_ms = now_ms;
    e->to_master = to_master ? 1 : 0;
    e->reason = (uint8_t)why;
    e->decided_by = (uint8_t)decided_by;
    e->peer_valid = (peer_id != NULL);
    if (peer_id)
        memcpy(e->peer_id, peer_id, 8);
    else
        memset(e->peer_id, 0, 8);
    g_bmca_view.ev_next = (uint8_t)((g_bmca_view.ev_next + 1) % BMCA_EVENT_RING);
    if (g_bmca_view.ev_count < BMCA_EVENT_RING)
        g_bmca_view.ev_count++;
    g_bmca_view.role_changes++;
    bmca_view_write_end();
}

static void bmca_update(const bmca_dataset_t *remote)
{
    bmca_dataset_t local;
    build_local_dataset(&local);

    bmca_field_t field;
    bool remote_better = (bmca_compare(remote, &local, &field) < 0);

    bmca_view_write_begin();
    g_bmca_view.local = local;
    g_bmca_view.local_valid = true;
    g_bmca_view.peer = *remote;
    g_bmca_view.peer_valid = true;
    g_bmca_view.peer_better = remote_better;
    g_bmca_view.decided_by = (uint8_t)field;
    bmca_view_write_end();

    if (remote_better)
    {
        memcpy(&g_best_master, remote, sizeof(bmca_dataset_t));
        bmca_set_role(false, BMCA_REASON_BETTER_ANNOUNCE, remote->clock_id, field);
    }
    else
    {
        memcpy(&g_best_master, &local, sizeof(bmca_dataset_t));
        bmca_set_role(true, BMCA_REASON_WORSE_ANNOUNCE, remote->clock_id, field);
    }
}

static void handle_announce(const uint8_t *payload, size_t len)
{
    if (len < sizeof(ptp_announce_msg_t))
        return;

    const ptp_announce_msg_t *ann = (const ptp_announce_msg_t *)payload;

    bmca_dataset_t remote;
    announce_to_dataset(ann, &remote);

    g_last_announce_rx = time(NULL);

    // ─────────────────────────────────────────────
    // Stability filter (clock identity based)
    // ─────────────────────────────────────────────
    if (memcmp(remote.clock_id, g_candidate_master.clock_id, 8) == 0)
    {
        g_candidate_count++;
    }
    else
    {
        memcpy(&g_candidate_master, &remote, sizeof(bmca_dataset_t));
        g_candidate_count = 1;
    }

    bmca_view_write_begin();
    memcpy(g_bmca_view.cand_id, g_candidate_master.clock_id, 8);
    g_bmca_view.cand_count = (g_candidate_count > 255) ? 255 : (uint8_t)g_candidate_count;
    g_bmca_view.cand_valid = true;
    g_bmca_view.last_ann_rx_ms = esp_timer_get_time() / 1000 - g_boot_time_ms;
    bmca_view_write_end();

    // Apply BMCA only after stability threshold
    if (g_candidate_count >= BMCA_STABILITY_THRESHOLD)
    {
        bmca_update(&remote);
    }

    // ESP_LOGI(TAG,
    //          "Announce RX: GM=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X class=%u acc=0x%02X count=%d",
    //          remote.clock_id[0], remote.clock_id[1], remote.clock_id[2], remote.clock_id[3],
    //          remote.clock_id[4], remote.clock_id[5], remote.clock_id[6], remote.clock_id[7],
    //          remote.clock_class,
    //          remote.accuracy,
    //          g_candidate_count);
}

/* String tables shared by the /api/bmca JSON writer (web_server.h) and
 * bmca_log_task below. NULL means "emit JSON null" / "NONE". */
static const char *bmca_field_str(uint8_t f)
{
    switch ((bmca_field_t)f)
    {
    case BMCA_FIELD_STEPS_REMOVED:  return "steps_removed";
    case BMCA_FIELD_PRIORITY1:      return "priority1";
    case BMCA_FIELD_CLOCK_CLASS:    return "clock_class";
    case BMCA_FIELD_ACCURACY:       return "accuracy";
    case BMCA_FIELD_VARIANCE:       return "variance";
    case BMCA_FIELD_PRIORITY2:      return "priority2";
    case BMCA_FIELD_CLOCK_IDENTITY: return "clock_identity";
    case BMCA_FIELD_NONE:
    default:                        return NULL;
    }
}

static const char *bmca_reason_str(uint8_t r)
{
    switch ((bmca_reason_t)r)
    {
    case BMCA_REASON_BETTER_ANNOUNCE:  return "better_announce";
    case BMCA_REASON_WORSE_ANNOUNCE:   return "worse_announce";
    case BMCA_REASON_ANNOUNCE_TIMEOUT: return "announce_timeout";
    default:                           return NULL;
    }
}

/* Role-change logging, off the receive hot path (rule 1.2.1: no ESP_LOG* in
 * ptp_rx_task or anything it calls). Polls the seqlock every 500 ms and
 * prints any ring events not yet printed. If more than BMCA_EVENT_RING
 * changes happened within one 500 ms window, the oldest of that burst are
 * not printed (they are still counted in role_changes). */
static void bmca_log_task(void *arg)
{
    uint32_t printed = 0;
    bmca_view_t v;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!bmca_view_snapshot(&v) || v.role_changes == printed)
            continue;
        uint32_t missed = v.role_changes - printed;
        uint32_t n = missed > v.ev_count ? v.ev_count : missed;
        for (uint32_t k = n; k > 0; k--) /* oldest first */
        {
            int idx = ((int)v.ev_next - (int)k + BMCA_EVENT_RING) % BMCA_EVENT_RING;
            const bmca_event_t *e = &v.ev[idx];
            const char *reason = bmca_reason_str(e->reason);
            const char *field = bmca_field_str(e->decided_by);
            ESP_LOGW("BMCA", "t=%lld ms -> %s reason=%s decided_by=%s peer=%02X%02X..%02X%02X",
                     (long long)e->t_ms, e->to_master ? "MASTER" : "PASSIVE",
                     reason ? reason : "?", field ? field : "none",
                     e->peer_id[0], e->peer_id[1], e->peer_id[6], e->peer_id[7]);
        }
        printed = v.role_changes;
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Slave tracking
 *
 * Tracks distinct slave clock IDs seen via Delay_Req in the last
 * SLAVE_TIMEOUT_MS, for the OLED's active-slave counter. slave_track_seen
 * records/refreshes a slave; slave_active_count returns the live count.
 * ═════════════════════════════════════════════════════════════════════════ */

/* Record a Delay_Req from this slave clock_id. */
static void slave_track_seen(const uint8_t *clock_id)
{
    int64_t now_ms = esp_timer_get_time() / 1000;

    portENTER_CRITICAL(&g_slaves_spinlock);

    int free_slot = -1;
    int oldest_slot = 0;
    int64_t oldest_time = INT64_MAX;

    for (int i = 0; i < SLAVE_TRACK_MAX; i++)
    {
        if (g_slaves[i].last_seen_ms != 0 &&
            memcmp(g_slaves[i].clock_id, clock_id, 8) == 0)
        {
            /* Existing slave — refresh */
            g_slaves[i].last_seen_ms = now_ms;
            portEXIT_CRITICAL(&g_slaves_spinlock);
            return;
        }
        if (g_slaves[i].last_seen_ms == 0 && free_slot < 0)
        {
            free_slot = i;
        }
        if (g_slaves[i].last_seen_ms < oldest_time)
        {
            oldest_time = g_slaves[i].last_seen_ms;
            oldest_slot = i;
        }
    }

    /* New slave — use free slot, or evict oldest if full */
    int slot = (free_slot >= 0) ? free_slot : oldest_slot;
    memcpy(g_slaves[slot].clock_id, clock_id, 8);
    g_slaves[slot].last_seen_ms = now_ms;

    portEXIT_CRITICAL(&g_slaves_spinlock);
}

/* Count slaves seen in the last SLAVE_TIMEOUT_MS. */
static int slave_active_count(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;
    int count = 0;

    portENTER_CRITICAL(&g_slaves_spinlock);
    for (int i = 0; i < SLAVE_TRACK_MAX; i++)
    {
        if (g_slaves[i].last_seen_ms != 0 &&
            (now_ms - g_slaves[i].last_seen_ms) < SLAVE_TIMEOUT_MS)
        {
            count++;
        }
    }
    portEXIT_CRITICAL(&g_slaves_spinlock);

    return count;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: PTP message handlers
 *
 * enqueue_delay_req copies the incoming Delay_Req's RX hardware timestamp and
 * identity onto g_delay_resp_q (called from the receive thread, does no I/O);
 * delay_resp_task drains the queue and builds + sends the Delay_Resp off the
 * receive thread. send_sync_and_followup emits the two-step Sync + Follow_Up
 * pair (Follow_Up carries the Sync's HW TX timestamp). send_announce emits the
 * Announce with current clock-quality fields.
 * ═════════════════════════════════════════════════════════════════════════ */

/* Receive-thread half: validate, snapshot, enqueue. No logging, no send, no
 * heap — keep the receive loop free to take the next frame (including Sync /
 * Announce, which must not be starved by a Delay_Req flood). */
static void enqueue_delay_req(const uint8_t *ptp_payload, size_t len,
                              const struct timespec *rx_hw_ts)
{
    if (len < sizeof(ptp_delay_req_msg_t))
        return;
    const ptp_delay_req_msg_t *req = (const ptp_delay_req_msg_t *)ptp_payload;

    delay_resp_job_t job;
    job.rx_hw_ts = *rx_hw_ts;
    job.req_port_id = req->hdr.source_port_id; /* packed-struct copy */
    job.req_seq = ntohs(req->hdr.sequence_id);
    job.req_correction = req->hdr.correction_field; /* raw, echoed verbatim */

    /* Timeout 0: under flood, drop rather than block the receive loop. A
     * dropped Delay_Resp costs the slave one delay sample — it simply tries
     * again at its next logMinDelayReqInterval. Count drops for diagnostics. */
    if (g_delay_resp_q == NULL ||
        xQueueSend(g_delay_resp_q, &job, 0) != pdTRUE)
        atomic_fetch_add(&g_delay_resp_drops, 1);
}

/* Worker-thread half: build and transmit the Delay_Resp. Also owns slave
 * tracking now (display-only, so it belongs off the receive hot path). */
static void delay_resp_task(void *arg)
{
    (void)arg;
    delay_resp_job_t job;

#if LOG_LOCAL_LEVEL >= ESP_LOG_INFO
    /* Heartbeat throttle: aggregate Delay_Req counts and emit one INFO line
     * per second. The state and the throttle block are wrapped in
     *   #if LOG_LOCAL_LEVEL >= ESP_LOG_INFO
     * so when INFO is disabled at compile time the function is identical
     * to its pre-heartbeat form — no timer read, no counter, no log call,
     * nothing. ESP_LOGI itself already short-circuits at runtime when the
     * tag's level has been lowered via esp_log_level_set, so we don't need
     * to repeat that check here; we only need to gate the counter+timer
     * work, which happens BEFORE the macro and can't be elided by it. */
    uint32_t since_last_log = 0;
    int64_t last_log_ms = esp_timer_get_time() / 1000;
#endif

    for (;;)
    {
        if (xQueueReceive(g_delay_resp_q, &job, portMAX_DELAY) != pdTRUE)
            continue;

#if LOG_LOCAL_LEVEL >= ESP_LOG_INFO
        since_last_log++;
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (now_ms - last_log_ms >= 1000)
        {
            ESP_LOGI("PTP_RX", "Delay_Req: %lu in last %lld ms",
                     (unsigned long)since_last_log,
                     (long long)(now_ms - last_log_ms));
            since_last_log = 0;
            last_log_ms = now_ms;
        }
#endif

        /* Track this slave for the OLED screen-1 counter */
        slave_track_seen(job.req_port_id.clock_identity);

        ptp_delay_resp_msg_t resp = {0};

        /* CRITICAL: DELAY_RESP must echo the DELAY_REQ sequence ID. */
        build_ptp_header(&resp.hdr, PTP_MSG_DELAY_RESP, sizeof(ptp_delay_resp_msg_t),
                         job.req_seq, 0, 0, 0x03);

        /* IEEE 1588-2008 11.3.2 c): the Delay_Resp carries the Delay_Req's
         * correctionField (TC residence times accumulated on the request
         * path). Both sides are on-wire big-endian and never interpreted
         * here, so the raw copy IS the correct echo. build_ptp_header
         * memset it to 0 just above — set it after. */
        resp.hdr.correction_field = job.req_correction;

        encode_ptp_timestamp(&resp.receive_timestamp, &job.rx_hw_ts);
        resp.requesting_port_id = job.req_port_id;

        int ret = l2tap_send_ptp(&resp, sizeof(resp), NULL);
        if (ret <= 0)
            ESP_LOGW("PTP_TX", "DELAY_RESP send failed: ret=%d errno=%d", ret, errno);
    }
}

static void send_sync_and_followup(void)
{
    uint16_t seq = g_sync_seq++;
    struct timespec t1 = {0};

    ptp_sync_msg_t sync = {0};
    build_ptp_header(&sync.hdr, PTP_MSG_SYNC, sizeof(ptp_sync_msg_t),
                     seq, PTP_FLAG_TWO_STEP, 0, 0x00);

    /* TX timestamp comes back synchronously in t1. ret <= 0 covers both the
     * explicit failure (-1) and the 0-bytes-accepted case: either way the
     * Sync never hit the wire, and a Follow_Up for it would be an orphan
     * carrying a fabricated timestamp. */
    int ret = l2tap_send_ptp(&sync, sizeof(sync), &t1);
    if (ret <= 0)
        return;

    bool used_fallback = false;
    if (t1.tv_sec == 0 && t1.tv_nsec == 0)
    {
        /* HW TX timestamp not delivered by the EMAC. Once PHC is at
         * year-2026 wall-clock this should never legitimately happen;
         * warn so a silent SW-fallback regression is visible. */
        ESP_LOGW("PTP_TX", "Sync seq=%u: HW TX timestamp unavailable, using SW fallback",
                 seq);
        emac_get_time(&t1);
        used_fallback = true;
    }

    /* NOTE: a TX_TS diagnostic log used to live here. Removed: it ran
     * between the Sync TX and the FollowUp TX, delaying the FollowUp by
     * 100s of µs and occasionally 10+ ms when the UART buffer flushed.
     * This widened FU latency seen by slaves and added timing jitter.
     * Use ESP_LOGD if you need it back during bring-up. */
    (void)used_fallback;

    /* Record on-wire Sync spacing for the web /api/health dashboard. */
    ptp_sync_jitter_note(&t1);

    ptp_sync_msg_t fu = {0};
    build_ptp_header(&fu.hdr, PTP_MSG_FOLLOW_UP, sizeof(ptp_sync_msg_t),
                     seq, 0, 0, 0x02);
    encode_ptp_timestamp(&fu.origin_timestamp, &t1);
    l2tap_send_ptp(&fu, sizeof(fu), NULL);
}

static void send_announce(void)
{
    ptp_announce_msg_t ann = {0};

    uint8_t cc = advertised_clock_class();
    bool locked_or_holdover = (cc != 248);

    // ✅ Build proper TimeProperties flags
    uint16_t flags = 0;

    /* Only assert PTP timescale + traceability once we've actually held real
     * time. A never-synchronized clock (clockClass 248) must not claim a
     * traceable PTP timescale — otherwise a slave that ignores clockClass
     * would still adopt the bogus free-running epoch. TIME/FREQ_TRACEABLE are
     * asserted only when fully LOCKED (cc==6), never in holdover (cc==7).
     *
     * currentUtcOffsetValid rides the same gate: announcing "UTC offset
     * valid" alongside an ARB timescale on a free-running epoch is
     * contradictory (IEEE 1588-2008 8.2.4) — the 37 s constant means
     * nothing until the timescale is real. LEAP59/61 are never asserted:
     * leap-second scheduling is not available from this NMEA stream, and
     * g_utc_offset is maintained manually (see its declaration). */
    if (locked_or_holdover)
    {
        flags |= PTP_FLAG_UTC_OFFSET_VALID;
        flags |= PTP_FLAG_PTP_TIMESCALE;
        if (cc == 6)
        {
            flags |= PTP_FLAG_TIME_TRACEABLE;
            flags |= PTP_FLAG_FREQ_TRACEABLE;
        }
    }

    build_ptp_header(&ann.hdr,
                     PTP_MSG_ANNOUNCE,
                     sizeof(ptp_announce_msg_t),
                     g_announce_seq++,
                     flags,
                     1,
                     0x05);

    struct timespec now;
    emac_get_time(&now);

    encode_ptp_timestamp(&ann.origin_timestamp, &now);

    // Clock quality (dynamic): 248 = never synchronized, 7 = holdover, 6 = locked
    ann.grandmaster_clock_class = cc;
    ann.grandmaster_clock_accuracy = (cc == 6) ? 0x21 : (cc == 7) ? 0x23
                                                                  : 0xFE;
    /* time_source: GNSS (0x20) only when fully locked; INTERNAL_OSCILLATOR
     * (0xA0) in holdover and while never-synchronized. */
    ann.time_source = (cc == 6) ? 0x20 : 0xA0;

    // ✅ REQUIRED for ptp4l
    ann.current_utc_offset = htons(g_utc_offset);

    uint8_t p1 = atomic_load(&g_priority1);
    uint8_t p2 = atomic_load(&g_priority2);
    ann.grandmaster_priority1 = p1;
    ann.grandmaster_clock_variance = htons(0xFFFF);
    ann.grandmaster_priority2 = p2;

    memcpy(ann.grandmaster_identity, g_clock_id, 8);

    l2tap_send_ptp(&ann, sizeof(ann), NULL);

    /* Publish the dataset just placed on the wire for bmca.local (MASTER
     * case) — one atomic_store, no seqlock: this is the only writer and the
     * value is a single 32-bit word. variance is always the 0xFFFF constant
     * above, so it isn't packed here. */
    uint32_t packed = (uint32_t)cc | ((uint32_t)ann.grandmaster_clock_accuracy << 8) |
                      ((uint32_t)p1 << 16) | ((uint32_t)p2 << 24);
    atomic_store(&g_local_ds_packed, packed);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: NMEA parsers
 *
 * parse_zda / parse_rmc extract UTC time and fix validity from the
 * time-bearing sentences (RMC owns fix validity; ZDA only updates time when
 * RMC has confirmed a valid fix).
 *
 * GNSS signal-quality monitoring (GSV/GSA):
 * Parses NMEA $xxGSV (satellites in view) and $xxGSA (active satellites)
 * sentences to track signal-to-noise per SV across all constellations
 * (GPS / GLONASS / Galileo / BeiDou). Useful for diagnosing antenna or
 * sky-view problems on a stationary timing receiver.
 *
 * Each constellation emits its own GSV group; a group is N sentences
 * (sentence 1 of N, 2 of N, ...) each containing up to 4 SVs. We accumulate
 * per-talker into a per-talker buffer (g_gsv_acc), then emit a combined
 * snapshot when sentence == total.
 *
 * Data exposed to gnss_signal_task (declared in the global-state section):
 *   g_gnss_snr_avg     — avg C/N₀ across all SVs in view (dB-Hz)
 *   g_gnss_snr_min     — worst SV C/N₀
 *   g_gnss_snr_max     — best SV C/N₀
 *   g_gnss_sv_in_view  — count of SVs reported by any constellation
 *   g_gnss_sv_used     — count of SVs used in the position-fix solution (GSA)
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────────
 * NMEA line hygiene helpers
 * ───────────────────────────────────────────────────────────────────────── */

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

/* Validate the "*XX" NMEA checksum (XOR of every byte between '$' and '*').
 * Rejects lines with no '$', no '*', a malformed hex field, or a mismatch.
 * Without this gate, a UART glitch that lands inside a digit passes every
 * downstream range check and is consumed as truth — worst at COARSE STEP
 * time, where a corrupted seconds value steps the PHC to the wrong second.
 * The TAU1201's binary protocol frames and partial boot-time lines also
 * flow through gnss_task's line assembler; they fail here and are dropped
 * silently on purpose (logging would spam for the whole configurator run). */
static bool nmea_checksum_ok(const char *line)
{
    if (line[0] != '$')
        return false;
    uint8_t sum = 0;
    const char *p = line + 1;
    while (*p && *p != '*')
        sum ^= (uint8_t)*p++;
    if (*p != '*')
        return false;
    int hi = hexval(p[1]);
    int lo = (hi >= 0) ? hexval(p[2]) : -1; /* p[2] only read when p[1] is a hex digit */
    if (lo < 0)
        return false;
    return sum == (uint8_t)((hi << 4) | lo);
}

/* Split a NUL-terminated NMEA sentence on commas, PRESERVING empty fields.
 * strtok_r collapses ",," — silently renumbering every later field — and
 * empty fields are routine in NMEA: unused GSA PRN slots, untracked GSV
 * SNRs, empty RMC course on a stationary receiver. Modifies buf in place
 * (each ',' becomes NUL) and stores up to max_fields pointers. Returns the
 * number of fields stored (clamped to max_fields). */
static int nmea_split(char *buf, char *fields[], int max_fields)
{
    if (max_fields <= 0)
        return 0;
    int n = 0;
    fields[n++] = buf;
    for (char *p = buf; *p && n < max_fields; p++)
    {
        if (*p == ',')
        {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    return n;
}

/* ─────────────────────────────────────────────────────────────────────────
 * gnss_task helpers — protect g_gnss_seconds writes
 * ───────────────────────────────────────────────────────────────────────── */
static bool parse_zda(const char *line)
{
    int hh, mm, ss, day, mon, year;

    if (sscanf(line, "$%*2cZDA,%2d%2d%2d,%2d,%2d,%4d",
               &hh, &mm, &ss, &day, &mon, &year) != 6)
        return false;

    /* Range-check before building tm (mirrors parse_rmc). ZDA carries a
     * 4-digit year. ss==60 permitted for leap second. */
    if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hh > 23 || mm > 59 || ss > 60 || year < 2000 || year > 2100)
        return false;

    struct tm t = {0};
    t.tm_year = year - 1900;
    t.tm_mon = mon - 1;
    t.tm_mday = day;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;

    time_t utc = my_timegm(&t);

    /* Update GNSS time only if RMC has confirmed fix is valid.
     * ZDA alone does not indicate fix status (NMEA spec) — many modules
     * continue emitting ZDA after fix loss with stale or coasting time. */
    taskENTER_CRITICAL(&g_gnss_spinlock);
    bool fix_ok = g_gnss_valid; /* set/cleared by parse_rmc only */
    if (fix_ok)
        g_gnss_seconds = utc;
    taskEXIT_CRITICAL(&g_gnss_spinlock);

    if (fix_ok)
        g_last_gnss_update = time(NULL);

    return true;
}

static bool parse_rmc(const char *line)
{
    char buf[128];
    strncpy(buf, line, sizeof(buf));
    buf[sizeof(buf) - 1] = 0;

    /* strip checksum after '*' (already verified by nmea_checksum_ok) */
    char *star = strchr(buf, '*');
    if (star)
        *star = 0;

    /* Positional split that PRESERVES empty fields (see nmea_split). RMC:
     *   0=$xxRMC 1=time 2=status 3=lat 4=N/S 5=lon 6=E/W 7=speed 8=course
     *   9=date 10=magvar 11=magvar-E/W 12=mode
     * course (8) and magvar (10/11) are routinely empty on a stationary
     * timing receiver; the old strtok_r walk shifted the date into the
     * wrong slot whenever any earlier field was empty. */
    char *f[16];
    int nf = nmea_split(buf, f, 16);

    if (nf < 3)
        return false;

    if (f[2][0] != 'A')
    {
        /* Fix lost — clear validity flag */
        taskENTER_CRITICAL(&g_gnss_spinlock);
        g_gnss_valid = false;
        taskEXIT_CRITICAL(&g_gnss_spinlock);
        return false;
    }

    if (nf < 10)
        return false;

    int hh = 0, mm = 0, ss = 0;
    int day = 0, mon = 0, year = 0;
    if (sscanf(f[1], "%2d%2d%2d", &hh, &mm, &ss) != 3 ||
        sscanf(f[9], "%2d%2d%2d", &day, &mon, &year) != 3)
    {
        /* Malformed time/date on a sentence claiming a valid fix: keep the
         * previous valid time rather than poisoning g_gnss_seconds. */
        return false;
    }

    if (year < 80)
        year += 100;

    /* Range-check before building tm. Feeding mon=0 (-> tm_mon=-1) to
     * my_timegm would index days_in_month[-1]. On a range failure, keep the
     * previous valid time rather than poisoning g_gnss_seconds.
     * (ss==60 permitted for leap second.) */
    if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hh > 23 || mm > 59 || ss > 60 || year < 80 || year > 200)
    {
        return false;
    }

    struct tm t = {0};
    t.tm_year = year;
    t.tm_mon = mon - 1;
    t.tm_mday = day;
    t.tm_hour = hh;
    t.tm_min = mm;
    t.tm_sec = ss;

    time_t utc = my_timegm(&t);

    taskENTER_CRITICAL(&g_gnss_spinlock);
    g_gnss_seconds = utc;
    g_gnss_valid = true;
    g_last_rmc_ms = esp_timer_get_time() / 1000;
    taskEXIT_CRITICAL(&g_gnss_spinlock);

    g_last_gnss_update = time(NULL);
    return true;
}

/* GGA contains the total number of satellites used by the complete
 * multi-constellation navigation solution.
 *
 *   0  message ID
 *   1  UTC
 *   2  latitude
 *   3  N/S
 *   4  longitude
 *   5  E/W
 *   6  fix quality
 *   7  total satellites used
 */
static bool parse_gga(const char *line)
{
    if (line == NULL || line[0] != '$' || strlen(line) < 10)
        return false;

    char buf[160];
    strncpy(buf, line, sizeof(buf));
    buf[sizeof(buf) - 1] = 0;

    /* Checksum has already been verified by gnss_task(). */
    char *star = strchr(buf, '*');
    if (star)
        *star = 0;

    char *f[20];
    int nf = nmea_split(buf, f, 20);

    if (nf < 8)
        return false;

    int fix_quality = atoi(f[6]);
    int used = atoi(f[7]);

    if (used < 0)
        used = 0;
    if (used > 255)
        used = 255;

    /* Do not retain a stale count when GGA says no solution. Allystar may
     * report fix quality 3 for a PPS/timing fix, so every non-zero quality
     * is accepted here. */
    if (fix_quality == 0)
        used = 0;

    int64_t now_ms = esp_timer_get_time() / 1000;

    portENTER_CRITICAL(&g_gnss_sig_lock);
    g_gnss_sv_used = (uint8_t)used;
    g_gga_last_update_ms = now_ms;
    portEXIT_CRITICAL(&g_gnss_sig_lock);

    return true;
}

/* Talker index mapping:
 *   0 = GP (GPS), 1 = GL (GLONASS), 2 = GA (Galileo),
 *   3 = GB/BD (BeiDou), 4 = GN (multi-constellation summary), 5 = other
 * Indexes both g_gsv_acc[] and g_sv_active_per_talker[]. */
static int talker_index(const char *talker /* 2 chars, e.g. "GP" */)
{
    if (talker[0] == 'G' && talker[1] == 'P')
        return 0;
    if (talker[0] == 'G' && talker[1] == 'L')
        return 1;
    if (talker[0] == 'G' && talker[1] == 'A')
        return 2;
    if (talker[0] == 'G' && talker[1] == 'B')
        return 3;
    if (talker[0] == 'B' && talker[1] == 'D')
        return 3;
    if (talker[0] == 'G' && talker[1] == 'N')
        return 4;
    return 5;
}

static uint8_t gsv_compute_total_visible(int64_t now_ms)
{
    /* Index 4 is GN: a combined multi-constellation report. If it is fresh,
     * use it alone; adding GP/GL/GA/GB would double-count the satellites. */
    if (g_gsv_view_seen_ms[4] != 0 &&
        now_ms - g_gsv_view_seen_ms[4] <= GSV_COUNT_FRESH_MS)
    {
        return g_gsv_view_by_talker[4];
    }

    /* No fresh combined GN report: sum the latest fresh constellation-
     * specific totals. The total is repeated in every sentence belonging
     * to a GSV group, so each talker is stored once rather than added for
     * every sentence. */
    uint16_t total = 0;

    for (int t = 0; t < GSV_TALKER_COUNT; t++)
    {
        if (t == 4)
            continue; /* skip GN combined slot */

        if (g_gsv_view_seen_ms[t] != 0 &&
            now_ms - g_gsv_view_seen_ms[t] <= GSV_COUNT_FRESH_MS)
        {
            total += g_gsv_view_by_talker[t];
        }
    }

    return (total > 255) ? 255 : (uint8_t)total;
}

/* Publish signal-strength statistics.
 *
 * Important: this function no longer derives "satellites in view" from
 * non-zero SNR entries. A visible satellite can legitimately have an empty
 * SNR field. The visible count is now obtained from the GSV header inside
 * parse_gsv().
 */
static void gsv_publish_snapshot(void)
{
    uint16_t sum = 0;
    uint8_t cnt = 0;
    uint8_t mn = 255;
    uint8_t mx = 0;

    for (int t = 0; t < GSV_TALKER_COUNT; t++)
    {
        for (int i = 0; i < GSV_MAX_SVS_PER_TALKER; i++)
        {
            uint8_t s = g_gsv_acc[t].snr[i];

            if (s > 0)
            {
                sum += s;
                cnt++;

                if (s < mn)
                    mn = s;
                if (s > mx)
                    mx = s;
            }
        }
    }

    uint8_t avg = (cnt > 0) ? (uint8_t)(sum / cnt) : 0;

    if (cnt == 0)
    {
        mn = 0;
        mx = 0;
    }

    portENTER_CRITICAL(&g_gnss_sig_lock);
    g_gnss_snr_avg = avg;
    g_gnss_snr_min = mn;
    g_gnss_snr_max = mx;

    /* Do not assign g_gnss_sv_in_view here. An SNR count is not the same
     * thing as the receiver-reported number of satellites in view. */

    g_gnss_sig_last_update_ms = esp_timer_get_time() / 1000;
    portEXIT_CRITICAL(&g_gnss_sig_lock);
}

static bool parse_gsv(const char *line)
{
    /* GSV:
     *   field 0 = $xxGSV
     *   field 1 = total number of sentences in this group
     *   field 2 = current sentence number
     *   field 3 = total satellites in view for this talker/group
     *   fields 4 onward = PRN,elevation,azimuth,SNR groups
     */
    if (line == NULL || line[0] != '$' || strlen(line) < 8)
        return false;

    char talker[3] = {line[1], line[2], 0};
    int t = talker_index(talker);

    if (t < 0 || t >= GSV_TALKER_COUNT)
        return false;

    char buf[160];
    strncpy(buf, line, sizeof(buf));
    buf[sizeof(buf) - 1] = 0;

    char *star = strchr(buf, '*');
    if (star)
        *star = 0;

    char *f[24];
    int nf = nmea_split(buf, f, 24);

    if (nf < 4)
        return false;

    int total_sentences = atoi(f[1]);
    int sentence_number = atoi(f[2]);
    int satellites_in_view = atoi(f[3]);

    if (total_sentences <= 0 ||
        sentence_number <= 0 ||
        sentence_number > total_sentences)
    {
        return false;
    }

    if (satellites_in_view < 0)
        satellites_in_view = 0;
    if (satellites_in_view > 255)
        satellites_in_view = 255;

    /* Sentence 1 begins a new GSV group for this talker. */
    if (sentence_number == 1)
        memset(&g_gsv_acc[t], 0, sizeof(g_gsv_acc[t]));

    g_gsv_acc[t].total_expected = total_sentences;
    g_gsv_acc[t].count = satellites_in_view;

    /* Collect SNR values for the existing min/avg/max diagnostics.
     * Empty SNR fields remain zero and are excluded from SNR statistics,
     * but they do not reduce the visible count taken from field 3. */
    for (int k = 0; 7 + 4 * k < nf; k++)
    {
        const char *snr_field = f[7 + 4 * k];

        int snr_value = (snr_field[0] == 0)
                            ? 0
                            : atoi(snr_field);

        uint8_t snr = (snr_value >= 10 && snr_value <= 55)
                          ? (uint8_t)snr_value
                          : 0;

        int slot = (sentence_number - 1) * 4 + k;

        if (slot >= 0 && slot < GSV_MAX_SVS_PER_TALKER)
            g_gsv_acc[t].snr[slot] = snr;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;

    /* Store this talker's reported total. Do not add it once per GSV
     * sentence because field 3 is repeated throughout the group. */
    g_gsv_view_by_talker[t] = (uint8_t)satellites_in_view;
    g_gsv_view_seen_ms[t] = now_ms;

    uint8_t total_visible = gsv_compute_total_visible(now_ms);

    portENTER_CRITICAL(&g_gnss_sig_lock);
    g_gnss_sv_in_view = total_visible;
    g_gnss_sig_last_update_ms = now_ms;

    /* If GGA has disappeared, do not indefinitely display its last known
     * satellites-used count while GSV continues updating. */
    if (g_gga_last_update_ms == 0 ||
        now_ms - g_gga_last_update_ms > GSV_COUNT_FRESH_MS)
    {
        g_gnss_sv_used = 0;
    }

    portEXIT_CRITICAL(&g_gnss_sig_lock);

    /* Publish SNR statistics after receiving the last sentence in the group. */
    if (sentence_number == total_sentences)
        gsv_publish_snapshot();

    return true;
}

static inline bool prn_bit(const uint8_t *bitmap, int prn)
{
    if (prn < 0 || prn >= SV_CHURN_PRN_MAX)
        return false;
    return (bitmap[prn / 8] >> (prn % 8)) & 1;
}
static inline void prn_set(uint8_t *bitmap, int prn)
{
    if (prn < 0 || prn >= SV_CHURN_PRN_MAX)
        return;
    bitmap[prn / 8] |= (uint8_t)(1u << (prn % 8));
}

/* GSA: lists PRNs used in the current position fix.
 *   Format: $xxGSA,M,3,prn1,prn2,...prn12,pdop,hdop,vdop*xx
 *   Mode M = M/A (manual/auto), fix-mode 1/2/3 (no/2D/3D),
 *   then up to 12 PRN fields (empty for unused slots).
 * We just want the count of non-empty PRN slots = SVs contributing to fix. */
static bool parse_gsa(const char *line)
{
    if (line[0] != '$' || strlen(line) < 8)
        return false;

    char talker[3] = {line[1], line[2], 0};
    int t = talker_index(talker);

    char buf[160];
    strncpy(buf, line, sizeof(buf));
    buf[sizeof(buf) - 1] = 0;
    char *star = strchr(buf, '*');
    if (star)
        *star = 0;

    /* Build the new active-set for this talker from fields 3..14. Positional
     * split (nmea_split): unused PRN slots are EMPTY fields, and the old
     * strtok_r walk collapsed them — sliding PDOP/HDOP/VDOP left into the
     * PRN window, where atoi("2.5") == 2 minted phantom SVs and corrupted
     * the churn statistics and the sv_used count. */
    uint8_t new_set[SV_CHURN_PRN_MAX / 8] = {0};
    int used = 0;

    char *f[20]; /* $xxGSA has 18 fields; NMEA 4.10 appends a system ID */
    int nf = nmea_split(buf, f, 20);
    for (int i = 3; i <= 14 && i < nf; i++)
    {
        if (f[i][0] == 0)
            continue;
        int prn = atoi(f[i]);
        if (prn > 0 && prn < SV_CHURN_PRN_MAX)
        {
            prn_set(new_set, prn);
            used++;
        }
    }

    /* Diff new_set against this talker's previous set; tally entries/exits.
     *
     * IMPORTANT: GNSS receivers can emit GSA in two patterns:
     *
     *   Pattern A: one $GPGSA + one $GLGSA + ... per cycle, each containing
     *              only that constellation's PRNs. $GNGSA (if emitted)
     *              contains the union of all.
     *
     *   Pattern B: multiple $GNGSA per cycle, each containing one
     *              constellation's PRNs. No per-constellation talkers.
     *
     * In pattern B, treating each $GNGSA arrival as one "set update" of
     * the GN slot makes successive $GNGSA emissions look like enormous
     * churn — each constellation's PRN set appears to "replace" the
     * previous one, so a steady 4-constellation receiver would report
     * ~4× |SVs| entries and exits per second. */
    uint32_t entries = 0, exits = 0;

    if (t == 4) /* GN: special handling — coalesce within a cycle */
    {
        static int64_t gn_window_start_ms = 0;
        static uint8_t gn_window_accum[SV_CHURN_PRN_MAX / 8];
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (now_ms - gn_window_start_ms > 1200)
        {
            /* New cycle began — diff the just-finished accumulation
             * against the previous one, then start fresh. */
            for (int i = 0; i < SV_CHURN_PRN_MAX; i++)
            {
                bool was = prn_bit(g_sv_active_per_talker[t], i);
                bool now = prn_bit(gn_window_accum, i);
                if (now && !was)
                    entries++;
                else if (was && !now)
                    exits++;
            }
            memcpy(g_sv_active_per_talker[t], gn_window_accum, sizeof(gn_window_accum));
            memset(gn_window_accum, 0, sizeof(gn_window_accum));
            gn_window_start_ms = now_ms;
        }
        /* Add this sentence's PRNs to the in-progress accumulation. */
        for (int i = 0; i < SV_CHURN_PRN_MAX / 8; i++)
            gn_window_accum[i] |= new_set[i];
    }
    else /* per-constellation talker: straightforward diff */
    {
        for (int i = 0; i < SV_CHURN_PRN_MAX; i++)
        {
            bool was = prn_bit(g_sv_active_per_talker[t], i);
            bool now = prn_bit(new_set, i);
            if (now && !was)
                entries++;
            else if (was && !now)
                exits++;
        }
        memcpy(g_sv_active_per_talker[t], new_set, sizeof(new_set));
    }

    portENTER_CRITICAL(&g_sv_churn_lock);
    g_sv_entries += entries;
    g_sv_exits += exits;
    g_sv_gsa_seen++;
    portEXIT_CRITICAL(&g_sv_churn_lock);

    /* GSA remains responsible for PRN-set/churn diagnostics. Do not update
    * g_gnss_sv_used here: each GSA may describe only one constellation.
    * GGA provides the total used by the complete solution. */
    return true;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: PPS ISR + servo (the timing core)
 *
 * pps_isr_handler captures the PHC timestamp inside the GPIO ISR — as close
 * to the PPS edge as possible — to eliminate scheduling jitter from the
 * servo offset measurement. pps_task turns each captured edge into a GNSS
 * time target (handling sentence_offset bootstrapping, holdover coasting,
 * and PPS edge-to-edge jitter accounting). servo_task is the PI(D) control
 * loop that disciplines the PHC frequency to GNSS, including lock acquisition,
 * holdover, and the self-healing reject-wedge watchdog.
 * ═════════════════════════════════════════════════════════════════════════ */

static void IRAM_ATTR pps_isr_handler(void *arg)
{
    /*
     * Capture the PHC time AS CLOSE AS POSSIBLE to the PPS edge.
     * Doing this inside the ISR (rather than in pps_task) eliminates
     * scheduling jitter (~10–50 µs) that would otherwise be injected
     * directly into the servo offset measurement.
     */

    /* Drop edges while emac_restart_preserve_phc() is rebuilding the MAC:
     * esp_eth_stop/start re-initializes the PHC, so a capture landing in
     * the ~220 ms dead window reads a just-reset counter (~0). Publishing
     * that would hand pps_task a garbage offset sample (LOCKED absorbs it
     * via the reject gate, but ACQUIRING would burn a sanity-rail strike
     * on it). Losing the edge entirely is the clean outcome: pps_task sees
     * no new capture and the servo simply skips that second. atomic_load
     * on RV32 is a plain inlined load — ISR/IRAM-safe. */
    if (atomic_load(&g_emac_restarting))
        return;

    struct timespec ts;
    emac_get_time(&ts);

    /* Publish ns+seq atomically w.r.t. pps_task's paired read. _ISR variant:
     * we are in interrupt context. Critical section is a few stores long. */
    portENTER_CRITICAL_ISR(&g_pps_capture_lock);
    g_pps_capture_ns =
        (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    g_pps_capture_seq++;
    portEXIT_CRITICAL_ISR(&g_pps_capture_lock);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_pps_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * pps_task — waits on the PPS ISR, updates jitter tracking, computes the
 * GNSS-time target for the servo, and notifies servo_task.
 * ───────────────────────────────────────────────────────────────────────── */
static void pps_task(void *arg)
{
    /* Gate on timing-core readiness (PHC started + L2TAP bound), NOT on link.
     * GNSS disciplining must begin at power-on even with no peer connected. */
    while (!atomic_load(&g_phc_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Drain any stale PPS notifications captured before PHC was up */
    while (xSemaphoreTake(g_pps_sem, 0) == pdTRUE)
    { /* discard */
    }

    static uint32_t last_capture_seq = 0;

    while (1)
    {
        // ESP_LOGW("PPS", "Start of PPS Running: integ=%.0f adj=%.1f ppb g_pps_cap_ns=%llu",
        //          g_integral,
        //          g_freq_ppb, g_pps_capture_ns);
        if (xSemaphoreTake(g_pps_sem, portMAX_DELAY))
        {
            /*
             * Use the PHC value captured INSIDE the ISR, not a fresh
             * read from this task. This is what gives us sub-µs
             * accuracy: the value is sampled within ~1 µs of the
             * actual PPS edge, before any FreeRTOS scheduling delay.
             */
            /* Read seq+ns together under the same lock the ISR uses, so the
             * 64-bit ns value cannot tear against a concurrent ISR on the
             * other core. */
            portENTER_CRITICAL(&g_pps_capture_lock);
            uint32_t seq = g_pps_capture_seq;
            uint64_t phc_ns = g_pps_capture_ns;
            portEXIT_CRITICAL(&g_pps_capture_lock);

            if (seq == last_capture_seq)
                continue; /* spurious wake — no new edge */
            last_capture_seq = seq;

            /* Edge regularity: ~1 s after the previous edge (monotonic time). */
            static int64_t prev_edge_mono_us = 0;
            int64_t edge_mono_us = esp_timer_get_time();
            bool edge_regular = false;
            if (prev_edge_mono_us != 0)
            {
                int64_t d_us = edge_mono_us - prev_edge_mono_us;
                edge_regular = (d_us > 1000000LL - PPS_REGULAR_TOL_US) &&
                               (d_us < 1000000LL + PPS_REGULAR_TOL_US);
            }
            prev_edge_mono_us = edge_mono_us;
            uint64_t phc_s = phc_ns / 1000000000ULL;

            /* ── PPS edge-to-edge jitter (raw receiver timing noise) ──
             * Skip the first edge (no prev) and any edge after a long gap
             * (e.g. holdover or initial bring-up); only accumulate when
             * the previous edge is from the immediately previous second.
             * "Plausible" = interval within ±100 ms of 1 s; anything wider
             * is a gap, not jitter. */
            static uint64_t prev_phc_ns_for_jitter = 0;
            if (prev_phc_ns_for_jitter != 0)
            {
                int64_t interval = (int64_t)phc_ns - (int64_t)prev_phc_ns_for_jitter;
                int64_t jitter = interval - 1000000000LL;
                if (llabs(jitter) < 100000000LL)
                {
                    portENTER_CRITICAL(&g_pps_jit_lock);
                    if (jitter < g_pps_jit_min_ns)
                        g_pps_jit_min_ns = jitter;
                    if (jitter > g_pps_jit_max_ns)
                        g_pps_jit_max_ns = jitter;
                    g_pps_jit_sum_abs_ns += llabs(jitter);
                    g_pps_jit_count++;
                    if (llabs(jitter) > 100000LL) /* >100 µs = clearly bad edge */
                        g_pps_jit_outliers++;
                    portEXIT_CRITICAL(&g_pps_jit_lock);
                }
                /* else: gap event — don't account, but still update prev */
            }
            prev_phc_ns_for_jitter = phc_ns;
            /* ── end jitter accumulation ── */

            taskENTER_CRITICAL(&g_gnss_spinlock);
            time_t gnss_sec = g_gnss_seconds;
            bool gnss_valid = g_gnss_valid;
            int64_t rmc_ms = g_last_rmc_ms;
            taskEXIT_CRITICAL(&g_gnss_spinlock);
            /* A fix is only as good as its freshness (see GNSS_FRESH_MS). */
            if (gnss_valid && (esp_timer_get_time() / 1000 - rmc_ms) > GNSS_FRESH_MS)
                gnss_valid = false;

            /* Trusted edge: regular spacing AND GNSS time advanced by exactly 1 s
             * since the previous edge (a lone bad-time sentence fails this on its
             * own edge and on the next one). */
            static bool prev_edge_gnss_valid = false;
            static time_t prev_edge_gnss_sec = 0;
            bool edge_trusted = edge_regular && gnss_valid && prev_edge_gnss_valid &&
                                (gnss_sec == prev_edge_gnss_sec + 1);
            prev_edge_gnss_valid = gnss_valid;
            prev_edge_gnss_sec = gnss_sec;

            ESP_LOGD("PPS_DIAG",
                     "phc_s=%llu gnss_sec=%lld",
                     (unsigned long long)phc_s,
                     (long long)gnss_sec);

            /* Phase and second of this edge, as the PHC sees them. */
            int64_t nearest_ns = (((int64_t)phc_ns + 500000000LL) / 1000000000LL) * 1000000000LL;
            int64_t phase_ns = (int64_t)phc_ns - nearest_ns; /* within +-0.5 s */
            int64_t phc_label_s = nearest_ns / 1000000000LL;
            static uint32_t label_gen = UINT32_MAX;
            static int64_t prev_label_s = 0;
            bool log_fix = false;
            int32_t log_diff = 0;
            uint32_t log_n = 0;

            taskENTER_CRITICAL(&g_servo_spinlock);
            g_last_pps_phc_ns = phc_ns;
            g_last_pps_trusted = edge_trusted;
            g_last_pps_regular = edge_regular;
            g_pps_edge_count++;
            bool plausible = true;
            if (g_phc_stepped)
            {
                /* Second continuity: a real edge advances the PHC second by 1, or by
                 * more after missing edges, or by 0 for an extra (glitch) edge. A
                 * second going backwards or jumping absurdly far is a garbage PHC
                 * capture (e.g. read during an EMAC restart): mark it implausible so
                 * the servo skips it, and keep the history unchanged. */
                if (g_phc_step_gen == label_gen)
                {
                    int64_t step_s = phc_label_s - prev_label_s;
                    if (step_s < 0 || step_s > 86400)
                        plausible = false;
                    else if (step_s == 0)
                        g_pps_coast_extra++;
                    else if (step_s > 1)
                        g_pps_coast_missing += (uint32_t)(step_s - 1);
                }
                if (plausible)
                {
                    label_gen = g_phc_step_gen;
                    prev_label_s = phc_label_s;
                    if (gnss_valid)
                        g_phc_offset_ns = phase_ns;
                    /* Seconds checker: does the receiver agree with the PHC? */
                    if (gnss_valid && edge_trusted)
                    {
                        int64_t nmea_s = (int64_t)gnss_sec + (int64_t)g_utc_offset + GNSS_SENTENCE_OFFSET;
                        int32_t diff = (int32_t)(nmea_s - phc_label_s);
                        if (diff == 0)
                        {
                            g_seconds_agree = true;
                            g_seconds_diff = 0;
                            g_seconds_disagree_n = 0;
                        }
                        else
                        {
                            if (diff == g_seconds_diff)
                                g_seconds_disagree_n++;
                            else
                            {
                                g_seconds_diff = diff;
                                g_seconds_disagree_n = 1;
                            }
                            g_seconds_agree = false;
                            uint32_t confirm = g_ever_locked ? SECONDS_CONFIRM_LOCKED_S : SECONDS_CONFIRM_BOOT_S;
                            if (g_seconds_disagree_n >= confirm && g_seconds_fix_req == 0)
                            {
                                g_seconds_fix_req = diff;
                                log_fix = true;
                                log_diff = diff;
                                log_n = g_seconds_disagree_n;
                            }
                        }
                    }
                }
            }
            g_last_pps_plausible = plausible;
            taskEXIT_CRITICAL(&g_servo_spinlock);
            if (!plausible)
                ESP_LOGW("PPS", "implausible PHC second at this edge (%lld after %lld) -- edge skipped",
                         (long long)phc_label_s, (long long)prev_label_s);
            if (log_fix)
                ESP_LOGW("PPS", "receiver and PHC disagree by %ld s for %lu checks -- PHC to be stepped by whole seconds",
                         (long)log_diff, (unsigned long)log_n);

            if (servo_task_handle != NULL)
                xTaskNotifyGive(servo_task_handle);
        }
        // ESP_LOGW("PPS", "PPS Running: so=%d",
        //          sentence_offset);
    }
}

/* All servo PHC writes go through this so they can be suppressed atomically
 * while watchdog_task is restarting the MAC on the other core. Returns false
 * if the write was skipped due to an in-progress restart. */
static inline bool servo_phc_ioctl(uint32_t cmd, void *arg)
{
    if (atomic_load(&g_emac_restarting))
        return false;
    esp_eth_ioctl(g_eth_hndl, cmd, arg);
    return true;
}

static void servo_task(void *arg)
{
    /* Wait for the timing core to be ready: EMAC driver started, PHC running,
     * L2TAP bound. This is set at init right after esp_eth_start — it does NOT
     * wait for a peer/link. The servo therefore acquires GNSS lock at power-on
     * whether or not a slave is connected. (Previously this gated on
     * g_l2tap_ready set inside on_link_up, which stalled the entire timing core
     * until a link appeared — the root cause of the "master before slave"
     * cold-acquire failure.) */
    while (!atomic_load(&g_phc_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    uint64_t last_pps = 0;
    bool prev_holdover = false;

    static servo_state_t state = SERVO_UNLOCKED;
    static int stable_count = 0;
    static int measure_count = 0;
    static int64_t offset_acc = 0;
    static int64_t acq_prev = 0, acq_jump = 0; /* ACQUIRE-A: previous sample, largest sample-to-sample change */
    static int64_t prev_acq_offset = INT64_MIN;
    static int acq_sample_count = 0; /* accepted ACQUIRING samples since last transition */
    static int64_t acq_win[LOCK_WIN_N]; /* last ACQUIRING offsets (ring) */
    static double acq_win_f[LOCK_WIN_N]; /* frequency applied after each of those samples */
    static int acq_win_n = 0;           /* samples in the ring */
    static int acq_win_i = 0;           /* next write index */
    static int acq_win_seen = 0;        /* samples since acquisition (re)started */
    static bool realign_pending = false;  /* holdover exit: phase to be realigned */
    static int realign_n = 0;
    static int64_t realign_buf[REALIGN_SAMPLES];
    static int64_t realign_start_ms = 0;
    static int realign_slew_left = 0;     /* servo cycles of frequency slew still to run */
    int64_t lock_start_at_holdover_entry = 0; /* preserve lock duration across sentence-only outages */

#define MEASURE_TICKS 5
#define SERVO_INTEGRAL_LIMIT 2000000000.0

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        taskENTER_CRITICAL(&g_gnss_spinlock);
        time_t cur_gnss_sec = g_gnss_seconds;
        bool cur_gnss_valid = g_gnss_valid;
        taskEXIT_CRITICAL(&g_gnss_spinlock);

        taskENTER_CRITICAL(&g_servo_spinlock);
        uint64_t pps = g_last_pps_phc_ns;
        bool pps_trusted_snap = g_last_pps_trusted;
        bool pps_plausible_snap = g_last_pps_plausible;
        bool pps_regular_snap = g_last_pps_regular;
        int32_t sec_fix = g_seconds_fix_req;
        g_seconds_fix_req = 0;
        taskEXIT_CRITICAL(&g_servo_spinlock);

        /* Seconds checker asked for a whole-second correction (the receiver
         * disagreed with the PHC long enough): step the PHC by whole seconds,
         * relative to its current value. The write latency leaves a few tens of
         * us of phase error, which the realign below (LOCKED) or the acquisition
         * (ACQUIRING) removes; class 7 until then. */
        if (sec_fix != 0)
        {
            struct timespec ts_fix;
            emac_get_time(&ts_fix);
            int64_t fix_ns = (int64_t)ts_fix.tv_sec * 1000000000LL + ts_fix.tv_nsec + (int64_t)sec_fix * 1000000000LL;
            eth_mac_time_t t_fix = {
                .seconds = (uint32_t)(fix_ns / 1000000000LL),
                .nanoseconds = (uint32_t)(fix_ns % 1000000000LL),
            };
            if (servo_phc_ioctl(ETH_MAC_ESP_CMD_S_PTP_TIME, &t_fix))
            {
                taskENTER_CRITICAL(&g_servo_spinlock);
                g_phc_step_gen++;
                g_seconds_agree = false; /* the next check confirms the new second */
                g_seconds_diff = 0;
                g_seconds_disagree_n = 0;
                taskEXIT_CRITICAL(&g_servo_spinlock);
                ESP_LOGW("SERVO", "PHC stepped by %ld s to the receiver's second", (long)sec_fix);
                g_settled_after_holdover = false; /* class 7 until realigned and settled again */
                if (state == SERVO_LOCKED && !g_holdover)
                {
                    avg_reset();
                    median_filter_reset();
                    g_last_offset = 0;
                    g_consecutive_rejects = 0;
                    memset(g_lock_qual_ring, 0, sizeof(g_lock_qual_ring));
                    g_lock_qual_idx = 0;
                    g_lock_qual_count = 0;
                    g_lock_acquired_ms = 0;
                    realign_pending = true;
                    realign_slew_left = 0;
                    realign_n = 0;
                    realign_start_ms = esp_timer_get_time() / 1000;
                }
            }
            else
            {
                taskENTER_CRITICAL(&g_servo_spinlock);
                g_seconds_fix_req = sec_fix; /* EMAC restarting: try again next cycle */
                taskEXIT_CRITICAL(&g_servo_spinlock);
            }
        }

        bool new_pps = (pps != last_pps);
        if (new_pps)
            last_pps = pps;

        g_holdover = (time(NULL) - g_last_gnss_update > HOLDOVER_TIMEOUT_SEC) || !cur_gnss_valid;

        if (g_holdover && !prev_holdover)
        {
            /* Before the first GNSS lock there is nothing to hold over — the PHC
             * has never held real time. Show ACQUIRING (link up, waiting for
             * GNSS), not HOLDOVER, so the LED's "degrading" heartbeat doesn't
             * imply a once-good clock is decaying. Only a clock that has locked
             * at least once (g_ever_locked) can legitimately be in holdover. */
            if (g_ever_locked)
            {
                ESP_LOGW("SERVO", "Entering HOLDOVER mode");
                led_set_state(LED_STATE_HOLDOVER);
            }
            else
            {
                ESP_LOGW("SERVO", "GNSS not yet acquired — ACQUIRING (no holdover before first fix)");
                led_set_state(LED_STATE_ACQUIRING);
            }
            lock_start_at_holdover_entry = g_lock_start_ms; /* stash for possible restore */
            g_lock_start_ms = 0;

            g_settled_after_holdover = false;
            g_lock_acquired_ms = 0;
            realign_pending = false;
            realign_slew_left = 0;
        }

        if (!g_holdover && prev_holdover)
        {
            /* GNSS is back. The PHC kept its second through the outage (with or
             * without PPS: it free-ran on the held frequency), so there is no
             * reason to start over -- only the phase may have drifted. Re-seat the
             * phase working buffers, keep the frequency, and (if LOCKED) realign the
             * phase before resuming the loop. If the receiver now reports a
             * different second, the seconds checker holds class 7 meanwhile. */
            ESP_LOGW("SERVO", "GNSS lock restored -- preserving freq, re-seating phase averager");
            avg_reset(); /* clears g_avg_* AND the paired g_d_* */
            median_filter_reset();
            g_last_offset = 0;
            g_consecutive_rejects = 0;
            memset(g_lock_qual_ring, 0, sizeof(g_lock_qual_ring));
            g_lock_qual_idx = 0;
            g_lock_qual_count = 0;
            g_qual_raw_min = INT64_MAX;
            g_qual_raw_max = INT64_MIN;
            g_qual_raw_samples = 0;
            if (state == SERVO_LOCKED)
            {
                led_set_state(LED_STATE_LOCKED);
                /* The PHC kept time through the outage: count the lock duration
                 * through it. g_lock_acquired_ms re-arms when the realign is done. */
                g_lock_start_ms = (lock_start_at_holdover_entry > 0)
                                      ? lock_start_at_holdover_entry
                                      : esp_timer_get_time() / 1000;
                g_lock_acquired_ms = 0;
                realign_pending = true;
                realign_n = 0;
                realign_start_ms = esp_timer_get_time() / 1000;
            }
            else
            {
                led_set_state(LED_STATE_ACQUIRING);
            }
        }
        prev_holdover = g_holdover;

        if (!new_pps)
            continue;

        switch (state)
        {
        case SERVO_UNLOCKED:
        {
            if (!cur_gnss_valid || cur_gnss_sec < 1577836800LL)
            {
                ESP_LOGW("SERVO", "Waiting for valid GNSS time (gnss_sec=%lld)",
                         (long long)cur_gnss_sec);
                continue;
            }

            /* Coarse-step only on a trusted edge (see g_last_pps_trusted): on a
             * glitch edge, or with a bad-time sentence, the PHC would be set on
             * the wrong second and the sentence offset derived from it would be
             * 1 s off for the rest of the uptime. */
            if (!pps_trusted_snap)
            {
                ESP_LOGW("SERVO", "UNLOCKED: PPS edge not trusted (spacing or GNSS second) -- waiting for the next edge");
                continue;
            }

            ESP_LOGW("SERVO", "UNLOCKED → ACQUIRING");
            led_set_state(LED_STATE_ACQUIRING);

            state = SERVO_ACQUIRING;
            stable_count = 0;
            measure_count = 0;
            offset_acc = 0;
            prev_acq_offset = INT64_MIN;
            acq_win_n = 0;
            acq_win_seen = 0;
            acq_sample_count = 0;
            g_integral = 0;

            g_lock_acquired_ms = 0;
            g_settled_after_holdover = false;

            uint32_t tai_sec =
                (uint32_t)((uint64_t)cur_gnss_sec + (uint64_t)g_utc_offset + 1ULL);

            ESP_LOGW("SERVO", "COARSE STEP → TAI=%" PRIu32 " s", tai_sec);

            eth_mac_time_t t = {
                .seconds = tai_sec,
                .nanoseconds = 0,
            };

            servo_phc_ioctl(ETH_MAC_ESP_CMD_S_PTP_TIME, &t);

            /* The PHC now holds real time. Published under the spinlock that
             * pps_task (other core) uses; the seconds history starts over. */
            taskENTER_CRITICAL(&g_servo_spinlock);
            g_phc_stepped = true;
            g_phc_step_gen++;
            g_seconds_agree = true;
            g_seconds_diff = 0;
            g_seconds_disagree_n = 0;
            taskEXIT_CRITICAL(&g_servo_spinlock);

            continue;
        }

        case SERVO_ACQUIRING:
        {
            if (!pps_plausible_snap)
                continue; /* garbage PHC capture (see pps_task) */
            /* A glitch edge (and the real edge right after it) is irregularly
             * spaced: never let it into the acquisition average or the PI. */
            if (!pps_regular_snap)
                continue;
            int64_t offset = (int64_t)pps - (((int64_t)pps + 500000000LL) / 1000000000LL) * 1000000000LL;

            /* ACQUIRING deliberately has NO convergence timeout. On noisy
             * PPS the lock gates (5 µs phase / 2 µs freq delta, 9
             * consecutive samples) can legitimately take many minutes, and
             * during all of it the fast-gain servo is actively disciplining
             * — a forced re-step would only throw that progress away and
             * step the PHC under any slave already following. The inputs
             * that used to wedge this state are all gated upstream now
             * (PHC-plausibility gate above, NMEA checksum, ISR restart
             * gate), so a long acquisition means signal
             * quality is the limit. Surface that instead of "fixing" it:
             * warn every ~5 minutes of continuous acquisition. */
            if (++acq_sample_count % 300 == 0)
                ESP_LOGW("SERVO",
                         "ACQUIRING for ~%d min without lock — PPS jitter or "
                         "antenna/sky-view quality is the likely limit",
                         acq_sample_count / 60);

            if (!g_phc_residual_measured)
            {
                if (measure_count == 0)
                {
                    measure_count++;
                    ESP_LOGW("SERVO",
                             "ACQUIRE-A: discard first (%lld ns)",
                             (long long)offset);
                    continue;
                }

                if (measure_count == 1)
                    acq_jump = 0; /* first sample of a new set */
                else if (llabs(offset - acq_prev) > acq_jump)
                    acq_jump = llabs(offset - acq_prev);
                acq_prev = offset;
                offset_acc += offset;
                measure_count++;

                if (measure_count - 1 >= MEASURE_TICKS)
                {
                    /* Step only on a consistent set: no sample-to-sample change beyond
                     * ACQ_A_JUMP_NS. A free crystal drifting steadily changes the offset by
                     * a constant small amount per second (~46 us here; anything up to
                     * 1000 ppm passes), but a jump of the reference inside the set -- which
                     * averaged to a fraction of the jump and left the PHC far off -- is
                     * caught, and a fresh set is collected. */
                    if (acq_jump > ACQ_A_JUMP_NS)
                    {
                        ESP_LOGW("SERVO",
                                 "ACQUIRE-A: reference jumped %lld ns inside the sample set -- collecting again",
                                 (long long)acq_jump);
                        measure_count = 1; /* first-sample discard already done */
                        offset_acc = 0;
                        continue;
                    }
                    int64_t avg_offset = offset_acc / MEASURE_TICKS;

                    ESP_LOGW("SERVO",
                             "ACQUIRE-A: avg offset=%lld ns → corrective step",
                             (long long)avg_offset);

                    int64_t corrected_ns = (int64_t)pps - avg_offset;
                    if (corrected_ns < 0)
                        corrected_ns = 0;

                    eth_mac_time_t t = {
                        .seconds = (uint32_t)(corrected_ns / 1000000000ULL),
                        .nanoseconds = (uint32_t)(corrected_ns % 1000000000ULL),
                    };

                    servo_phc_ioctl(ETH_MAC_ESP_CMD_S_PTP_TIME, &t);
                    taskENTER_CRITICAL(&g_servo_spinlock);
                    g_phc_step_gen++;
                    taskEXIT_CRITICAL(&g_servo_spinlock);

                    g_phc_residual_measured = true;
                    measure_count = 0;
                    offset_acc = 0;
                }
                continue;
            }

            /* ── Integrate with anti-windup ──────────────────────────────────
             * Tentatively integrate, compute the unsaturated correction, then
             * clamp the output. If the output saturated, BACK-CALCULATE the
             * integral to exactly the value that produces the clamped output,
             * so the integral never stores energy the actuator cannot express.
             * This prevents the rail-to-rail limit cycle seen on noisy PPS,
             * while being a no-op whenever the output is not saturated (normal
             * acquisition on a clean signal behaves exactly as before). */

            /* 1. Tentative integration (unchanged condition: freeze in holdover). */
            double integral_tentative = g_integral;
            if (!g_holdover)
                integral_tentative += (double)offset;

            /* Keep the hard integral clamp as a final backstop. */
            if (integral_tentative > SERVO_INTEGRAL_LIMIT)
                integral_tentative = SERVO_INTEGRAL_LIMIT;
            if (integral_tentative < -SERVO_INTEGRAL_LIMIT)
                integral_tentative = -SERVO_INTEGRAL_LIMIT;

            /* 2. Unsaturated correction and the resulting (clamped) output. */
            double correction_acq =
                SERVO_KP * (double)offset + SERVO_KI * integral_tentative;
            double freq_unsat = -correction_acq;
            double freq_sat = freq_unsat;
            bool saturated = false;
            if (freq_sat > SERVO_FREQ_LIMIT_PPB)
            {
                freq_sat = SERVO_FREQ_LIMIT_PPB;
                saturated = true;
            }
            else if (freq_sat < -SERVO_FREQ_LIMIT_PPB)
            {
                freq_sat = -SERVO_FREQ_LIMIT_PPB;
                saturated = true;
            }

            if (saturated && !g_holdover)
            {
                /* Back-calculate the integral so that
                 *   -(Kp*offset + Ki*integral) == freq_sat
                 * i.e. integral = (-freq_sat - Kp*offset) / Ki
                 * This holds the integral at exactly the boundary that keeps the
                 * output pinned at the rail, with no excess accumulation. When
                 * the error later reverses, the loop comes off the rail
                 * immediately instead of having to unwind a saturated integral.
                 * SERVO_KI is a compile-time positive constant (0.3) so the
                 * division is always safe. */
                g_integral = (-freq_sat - SERVO_KP * (double)offset) / SERVO_KI;

                /* The back-calculated value is, by construction, within the
                 * range the actuator can express, but clamp defensively so a
                 * pathological offset can never push the stored integral past
                 * its hard limit. */
                if (g_integral > SERVO_INTEGRAL_LIMIT)
                    g_integral = SERVO_INTEGRAL_LIMIT;
                if (g_integral < -SERVO_INTEGRAL_LIMIT)
                    g_integral = -SERVO_INTEGRAL_LIMIT;
            }
            else
            {
                /* Not saturated (or in holdover, where the integral is frozen):
                 * accept the tentative integration as-is. This is identical to
                 * the original behavior in the unsaturated case. */
                g_integral = integral_tentative;
            }

            g_freq_ppb = freq_sat;

            int32_t adj_acq_ppb = (int32_t)g_freq_ppb;
            if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_acq_ppb))
                atomic_store(&g_freq_ppb_hw, adj_acq_ppb);

            acq_win[acq_win_i] = offset;
            acq_win_f[acq_win_i] = g_freq_ppb;
            acq_win_i = (acq_win_i + 1) % LOCK_WIN_N;
            if (acq_win_n < LOCK_WIN_N)
                acq_win_n++;
            if (acq_win_seen < LOCK_WIN_GRACE)
                acq_win_seen++;
            bool win_ok = false;
            double win_mean = 0.0, win_slope = 0.0, win_freq = 0.0;
            if (acq_win_n == LOCK_WIN_N && acq_win_seen >= LOCK_WIN_GRACE && !g_holdover)
            {
                double sy = 0.0, sxy = 0.0, sf = 0.0;
                bool in_band = true;
                for (int k = 0; k < LOCK_WIN_N; k++)
                {
                    int64_t yi = acq_win[(acq_win_i + k) % LOCK_WIN_N]; /* oldest first */
                    if (llabs(yi) >= LOCK_WIN_MAX_NS)
                        in_band = false;
                    double y = (double)yi;
                    sf += acq_win_f[(acq_win_i + k) % LOCK_WIN_N];
                    sy += y;
                    sxy += (double)k * y;
                }
                const double wn = (double)LOCK_WIN_N;
                const double sx = wn * (wn - 1.0) / 2.0;
                const double sxx = (wn - 1.0) * wn * (2.0 * wn - 1.0) / 6.0;
                win_mean = sy / wn;
                win_slope = (wn * sxy - sx * sy) / (wn * sxx - sx * sx); /* ns per 1 s sample = ppb */
                /* Frequency the PHC needs = average applied over the window minus
                 * the residual drift measured under it (PHC fast => slope > 0). */
                win_freq = sf / wn - win_slope;
                double am = (win_mean < 0.0) ? -win_mean : win_mean;
                double as = (win_slope < 0.0) ? -win_slope : win_slope;
                win_ok = in_band && (am < (double)LOCK_OFFSET_NS) && (as < LOCK_WIN_SLOPE_PPB);
            }
            bool phase_ok = (llabs(offset) < LOCK_OFFSET_NS);
            bool freq_ok = (prev_acq_offset == INT64_MIN) ||
                           (llabs(offset - prev_acq_offset) < LOCK_FREQ_DELTA_NS);
            prev_acq_offset = offset;

            bool rule_step = phase_ok && freq_ok;
            bool rule_lock = rule_step && (++stable_count > LOCK_THRESHOLD);
            if (rule_lock || win_ok)
            {
                {
                    if (!rule_lock)
                    {
                        /* Hand the slow loop the window-averaged frequency, not the
                         * instantaneous value (which carries the last noisy sample's
                         * proportional kick). The 9-sample path is unchanged. */
                        ESP_LOGW("SERVO",
                                 "LOCK by %d-sample window: mean=%.0f ns slope=%.0f ppb freq=%.0f ppb (was %.0f)",
                                 LOCK_WIN_N, win_mean, win_slope, win_freq, g_freq_ppb);
                        g_freq_ppb = win_freq;
                        if (g_freq_ppb > SERVO_FREQ_LIMIT_PPB)
                            g_freq_ppb = SERVO_FREQ_LIMIT_PPB;
                        if (g_freq_ppb < -SERVO_FREQ_LIMIT_PPB)
                            g_freq_ppb = -SERVO_FREQ_LIMIT_PPB;
                        adj_acq_ppb = (int32_t)g_freq_ppb;
                        if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_acq_ppb))
                            atomic_store(&g_freq_ppb_hw, adj_acq_ppb);
                    }
                    ESP_LOGW("SERVO",
                             "LOCKED (offset=%lld ns, phase+freq converged %d samples, adj=%ld ppb)",
                             (long long)offset, stable_count, (long)adj_acq_ppb);
                    state = SERVO_LOCKED;
                    acq_sample_count = 0;

                    g_integral = -(double)g_freq_ppb / SERVO_KI_SLOW;
                    if (g_integral > SERVO_INTEGRAL_LIMIT)
                        g_integral = SERVO_INTEGRAL_LIMIT;
                    if (g_integral < -SERVO_INTEGRAL_LIMIT)
                        g_integral = -SERVO_INTEGRAL_LIMIT;
                    ESP_LOGI("SERVO",
                             "integrator rescaled for slow servo: integ=%.0f (target adj=%ld ppb)",
                             g_integral, (long)(int32_t)g_freq_ppb);

                    g_last_offset = offset;
                    median_filter_reset();
                    avg_reset();
                    led_set_state(LED_STATE_LOCKED);
                    g_lock_start_ms = esp_timer_get_time() / 1000;

                    memset(g_lock_qual_ring, 0, sizeof(g_lock_qual_ring));
                    g_lock_qual_idx = 0;
                    g_lock_qual_count = 0;

                    g_lock_acquired_ms = esp_timer_get_time() / 1000;
                    g_settled_after_holdover = false;

                    /* The COARSE STEP (SERVO_UNLOCKED) already set the PHC to
                     * GNSS-derived TAI before we could reach this state, so the
                     * PHC genuinely holds real time now. Latch it: from here on
                     * holdover (clockClass 7) is a legitimate claim. */
                    g_ever_locked = true;
                }
            }
            else if (!rule_step)
            {
                if ((stable_count & 0x03) == 0)
                {
                    ESP_LOGI("SERVO",
                             "ACQUIRE-B: not locked (offset=%lld phase=%s freq=%s adj=%ld ppb integ=%.0f)",
                             (long long)offset,
                             phase_ok ? "OK" : "FAIL",
                             freq_ok ? "OK" : "FAIL",
                             (long)adj_acq_ppb,
                             g_integral);
                }
                stable_count = 0;
            }

            continue;
        }

        case SERVO_LOCKED:
        {
            if (!pps_plausible_snap)
                continue; /* garbage PHC capture (see pps_task) */
            int64_t raw_offset = (int64_t)pps - (((int64_t)pps + 500000000LL) / 1000000000LL) * 1000000000LL;

#define HARD_REJECT_NS 100000LL /* 100 µs */
#if HOLDOVER_FREEZE_FREQ
            if (g_holdover)
            {
                double hold_ppb = -(SERVO_KI_SLOW * g_integral);
                if (hold_ppb > SERVO_FREQ_LIMIT_PPB)
                    hold_ppb = SERVO_FREQ_LIMIT_PPB;
                if (hold_ppb < -SERVO_FREQ_LIMIT_PPB)
                    hold_ppb = -SERVO_FREQ_LIMIT_PPB;
                if (g_freq_ppb != hold_ppb)
                {
                    g_freq_ppb = hold_ppb;
                    int32_t adj_h = (int32_t)g_freq_ppb;
                    if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_h))
                        atomic_store(&g_freq_ppb_hw, adj_h);
                }
                continue; /* free-run on the learned frequency; no phase steering */
            }
#endif
            if (realign_pending && realign_slew_left > 0)
            {
                if (--realign_slew_left == 0)
                {
                    /* slew done: back to the learned crystal correction, resume the loop */
                    g_freq_ppb = -(SERVO_KI_SLOW * g_integral);
                    if (g_freq_ppb > SERVO_FREQ_LIMIT_PPB)
                        g_freq_ppb = SERVO_FREQ_LIMIT_PPB;
                    if (g_freq_ppb < -SERVO_FREQ_LIMIT_PPB)
                        g_freq_ppb = -SERVO_FREQ_LIMIT_PPB;
                    int32_t adj_end = (int32_t)g_freq_ppb;
                    if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_end))
                        atomic_store(&g_freq_ppb_hw, adj_end);
                    avg_reset();
                    median_filter_reset();
                    g_last_offset = 0;
                    g_consecutive_rejects = 0;
                    memset(g_lock_qual_ring, 0, sizeof(g_lock_qual_ring));
                    g_lock_qual_idx = 0;
                    g_lock_qual_count = 0;
                    g_lock_acquired_ms = esp_timer_get_time() / 1000; /* settle window starts now */
                    realign_pending = false;
                    ESP_LOGI("SERVO", "REALIGN: slew complete (residual now %lld ns)", (long long)raw_offset);
                }
                continue;
            }
            if (realign_pending)
            {
                int64_t now_ms = esp_timer_get_time() / 1000;
                if (now_ms - realign_start_ms > REALIGN_TIMEOUT_MS)
                {
                    ESP_LOGE("SERVO", "REALIGN: no consistent phase within %d s after holdover -- full re-acquire",
                             REALIGN_TIMEOUT_MS / 1000);
                    realign_pending = false;
                    state = SERVO_UNLOCKED;
                    g_phc_residual_measured = false;
                    g_integral = 0;
                    g_freq_ppb = 0;
                    stable_count = 0;
                    prev_acq_offset = INT64_MIN;
                    acq_win_n = 0;
                    acq_win_seen = 0;
                    measure_count = 0;
                    offset_acc = 0;
                    median_filter_reset();
                    avg_reset();
                    g_last_offset = 0;
                    g_consecutive_rejects = 0;
                    led_set_state(LED_STATE_ACQUIRING);
                    g_lock_start_ms = 0;
                    g_lock_acquired_ms = 0;
                    g_settled_after_holdover = false;
                    continue;
                }
                if (llabs(raw_offset) >= REALIGN_MAX_NS)
                {
                    realign_n = 0; /* phase far off: not a realign case (very long outage) */
                    continue;
                }
                realign_buf[realign_n++] = raw_offset;
                int64_t mn = realign_buf[0], mx = realign_buf[0];
                for (int i = 1; i < realign_n; i++)
                {
                    if (realign_buf[i] < mn)
                        mn = realign_buf[i];
                    if (realign_buf[i] > mx)
                        mx = realign_buf[i];
                }
                if (mx - mn > REALIGN_SPREAD_NS)
                {
                    realign_buf[0] = raw_offset; /* inconsistent: restart from this sample */
                    realign_n = 1;
                    continue;
                }
                if (realign_n < REALIGN_SAMPLES)
                    continue;
                int64_t srt[REALIGN_SAMPLES];
                memcpy(srt, realign_buf, sizeof(srt));
                for (int i = 1; i < REALIGN_SAMPLES; i++)
                    for (int j = i; j > 0 && srt[j] < srt[j - 1]; j--)
                    {
                        int64_t tmp = srt[j];
                        srt[j] = srt[j - 1];
                        srt[j - 1] = tmp;
                    }
                int64_t err = (srt[REALIGN_SAMPLES / 2 - 1] + srt[REALIGN_SAMPLES / 2]) / 2;
                bool stepped = false;
                if (llabs(err) > REALIGN_MIN_NS && llabs(err) <= REALIGN_STEP_NS)
                {
                    /* Slew the error out: hold learned + slew for n whole servo cycles. */
                    int n = (int)((llabs(err) + REALIGN_SLEW_MAX_PPB - 1) / REALIGN_SLEW_MAX_PPB);
                    if (n < 1)
                        n = 1;
                    double learned = -(SERVO_KI_SLOW * g_integral);
                    double slew = -(double)err / (double)n; /* ppb for n seconds = -err ns */
                    g_freq_ppb = learned + slew;
                    if (g_freq_ppb > SERVO_FREQ_LIMIT_PPB)
                        g_freq_ppb = SERVO_FREQ_LIMIT_PPB;
                    if (g_freq_ppb < -SERVO_FREQ_LIMIT_PPB)
                        g_freq_ppb = -SERVO_FREQ_LIMIT_PPB;
                    int32_t adj_sl = (int32_t)g_freq_ppb;
                    if (!servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_sl))
                    {
                        realign_n = 0; /* EMAC restarting: try again */
                        continue;
                    }
                    atomic_store(&g_freq_ppb_hw, adj_sl);
                    realign_slew_left = n;
                    ESP_LOGW("SERVO", "REALIGN after holdover: phase error %lld ns (spread %lld ns) -> slewing %d s at %+.0f ppb",
                             (long long)err, (long long)(mx - mn), n, slew);
                    continue; /* stays pending (class 7) until the slew is done */
                }
                if (llabs(err) > REALIGN_STEP_NS)
                {
                    /* Step relative to the PHC's CURRENT value (read, then set), like the
                     * EMAC-restart restore: the error is then the ioctl latency, not the
                     * servo's wake-up delay after the PPS edge. */
                    struct timespec ts_now;
                    emac_get_time(&ts_now);
                    int64_t now_ns = (int64_t)ts_now.tv_sec * 1000000000LL + ts_now.tv_nsec - err;
                    eth_mac_time_t t = {
                        .seconds = (uint32_t)(now_ns / 1000000000LL),
                        .nanoseconds = (uint32_t)(now_ns % 1000000000LL),
                    };
                    if (!servo_phc_ioctl(ETH_MAC_ESP_CMD_S_PTP_TIME, &t))
                    {
                        realign_n = 0; /* EMAC restarting: try again */
                        continue;
                    }
                    taskENTER_CRITICAL(&g_servo_spinlock);
                    g_phc_step_gen++;
                    taskEXIT_CRITICAL(&g_servo_spinlock);
                    /* A step lands a few tens of us off (PHC write latency): measure
                     * the residual again and slew it out before resuming the loop --
                     * resuming at once made the slow loop overshoot it (to ~+30 us)
                     * while the 32-sample average already looked converged. */
                    ESP_LOGW("SERVO", "REALIGN after holdover: phase error %lld ns -> PHC stepped; measuring the residual",
                             (long long)err);
                    realign_n = 0;
                    continue;
                }
                /* keep only the learned crystal correction (integral term) */
                g_freq_ppb = -(SERVO_KI_SLOW * g_integral);
                if (g_freq_ppb > SERVO_FREQ_LIMIT_PPB)
                    g_freq_ppb = SERVO_FREQ_LIMIT_PPB;
                if (g_freq_ppb < -SERVO_FREQ_LIMIT_PPB)
                    g_freq_ppb = -SERVO_FREQ_LIMIT_PPB;
                int32_t adj_rl = (int32_t)g_freq_ppb;
                if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_rl))
                    atomic_store(&g_freq_ppb_hw, adj_rl);
                ESP_LOGW("SERVO", "REALIGN after holdover: phase error %lld ns (spread %lld ns) -> %s, freq %.0f ppb",
                         (long long)err, (long long)(mx - mn), stepped ? "PHC stepped" : "no step", g_freq_ppb);
                avg_reset();
                median_filter_reset();
                g_last_offset = 0;
                g_consecutive_rejects = 0;
                memset(g_lock_qual_ring, 0, sizeof(g_lock_qual_ring));
                g_lock_qual_idx = 0;
                g_lock_qual_count = 0;
                g_lock_acquired_ms = now_ms; /* settle window starts now */
                realign_pending = false;
                continue;
            }
            /* While the averager warms up (< 8 samples) gate against 0: every way
             * into SERVO_LOCKED (9-sample rule, window fallback, holdover realign)
             * guarantees |phase| < 20 us, so a warm-up sample 100 us away is a glitch
             * edge. Ungated, one such sample poisoned the average and wedged the
             * servo 45 s later. */
            int64_t reference = (g_avg_count >= 8)
                                    ? g_avg_sum / (int64_t)g_avg_count
                                    : 0;

            /* Only apply the reject gate once the averager has enough samples
             * for a stable reference. 8 is roughly a quarter of AVG_WINDOW (32);
             * below that the running mean is dominated by whichever handful of
             * early samples we happen to have, and rejecting against it would
             * throw away good samples during warm-up. Once we cross 8, the
             * reference is statistically representative enough to distinguish
             * a real 100 µs outlier from ordinary PPS noise. */
            if (llabs(raw_offset - reference) > HARD_REJECT_NS)
            {
                g_total_rejects++;
                g_consecutive_rejects++; /* visible on OLED screen 2 */
                g_phase_converged = false;
                if (g_consecutive_rejects >= CLASS6_DEMOTE_REJECTS)
                    g_phase_degraded = true;
                if ((g_total_rejects & 0x07) == 1)
                {
                    ESP_LOGW("SERVO",
                             "Rejected impossible jump: raw=%lld vs avg=%lld "
                             "(delta=%lld), total rejects=%lu streak=%lu",
                             (long long)raw_offset,
                             (long long)reference,
                             (long long)(raw_offset - reference),
                             (unsigned long)g_total_rejects,
                             (unsigned long)g_consecutive_rejects);
                }

                /* Self-heal watchdog. A rejection happens BEFORE the averager
                 * is updated, so a stale reference can never correct itself by
                 * normal operation: every sample is rejected, the average stays
                 * frozen, the gate never moves, adj/integrator freeze, the PHC
                 * free-runs, and raw_offset diverges further every second —
                 * forever. The LOCK_LOSS_AVG detector below is also bypassed by
                 * the early continue, so it cannot recover on its own. If we
                 * reject REJECT_STREAK_REACQUIRE samples in a row (~45 s), treat
                 * it as a wedge and force a full re-acquire. A few rejects in a
                 * row is normal PPS noise; tens in a row is a stuck reference. */
                if (g_consecutive_rejects >= REJECT_STREAK_REACQUIRE)
                {
                    ESP_LOGE("SERVO",
                             "REJECT WEDGE: %lu consecutive rejects (avg stuck at "
                             "%lld, raw now %lld). Forcing full re-acquire.",
                             (unsigned long)g_consecutive_rejects,
                             (long long)reference,
                             (long long)raw_offset);
                    state = SERVO_UNLOCKED;
                    g_phc_residual_measured = false;
                    g_integral = 0;
                    g_freq_ppb = 0;
                    stable_count = 0;
                    prev_acq_offset = INT64_MIN;
                    acq_win_n = 0;
                    acq_win_seen = 0;
                    measure_count = 0;
                    offset_acc = 0;
                    median_filter_reset();
                    avg_reset();
                    g_last_offset = 0;
                    /* g_total_rejects is lifetime — NOT reset on re-acquire. */
                    g_consecutive_rejects = 0;
                    led_set_state(LED_STATE_ACQUIRING);
                    g_lock_start_ms = 0;
                    g_lock_acquired_ms = 0;
                    g_settled_after_holdover = false;
                }
                continue;
            }

            g_consecutive_rejects = 0; /* accepted sample breaks the streak */

            int64_t evicted = (g_avg_count == AVG_WINDOW)
                                  ? g_avg_ring[g_avg_idx]
                                  : 0;
            g_avg_ring[g_avg_idx] = raw_offset;
            g_avg_idx = (g_avg_idx + 1) % AVG_WINDOW;
            if (g_avg_count < AVG_WINDOW)
            {
                g_avg_sum += raw_offset;
                g_avg_count++;
            }
            else
            {
                g_avg_sum += raw_offset - evicted;
            }
            int64_t offset = g_avg_sum / (int64_t)g_avg_count;

            int64_t d_evicted = (g_d_count == SERVO_D_WINDOW)
                                    ? g_d_ring[g_d_idx]
                                    : 0;
            g_d_ring[g_d_idx] = offset;
            g_d_idx = (g_d_idx + 1) % SERVO_D_WINDOW;
            if (g_d_count < SERVO_D_WINDOW)
                g_d_count++;

            double d_term = 0.0;
            if (g_d_count >= SERVO_D_WINDOW)
            {
                d_term = ((double)offset - (double)d_evicted) / (double)SERVO_D_WINDOW;
            }

            if (!g_holdover)
                g_integral += (double)offset;

            if (g_integral > SERVO_INTEGRAL_LIMIT)
                g_integral = SERVO_INTEGRAL_LIMIT;
            if (g_integral < -SERVO_INTEGRAL_LIMIT)
                g_integral = -SERVO_INTEGRAL_LIMIT;

            double desired_freq_ppb = -(SERVO_KP_SLOW * (double)offset + SERVO_KI_SLOW * g_integral + (SERVO_KD_ENABLE ? SERVO_KD_SLOW : 0.0) * d_term);

            double delta_ppb = desired_freq_ppb - g_freq_ppb;
            if (delta_ppb > ADJ_SLEW_PPB_PER_S)
                delta_ppb = ADJ_SLEW_PPB_PER_S;
            if (delta_ppb < -ADJ_SLEW_PPB_PER_S)
                delta_ppb = -ADJ_SLEW_PPB_PER_S;
            g_freq_ppb += delta_ppb;

            if (g_freq_ppb > SERVO_FREQ_LIMIT_PPB)
                g_freq_ppb = SERVO_FREQ_LIMIT_PPB;
            if (g_freq_ppb < -SERVO_FREQ_LIMIT_PPB)
                g_freq_ppb = -SERVO_FREQ_LIMIT_PPB;

            int32_t adj_ppb = (int32_t)g_freq_ppb;
            if (servo_phc_ioctl(ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_ppb))
                atomic_store(&g_freq_ppb_hw, adj_ppb);

            g_last_offset = offset;
            g_phase_converged = (g_avg_count >= AVG_WINDOW) && (llabs(offset) < CLASS6_MAX_OFFSET_NS);
            g_phase_degraded = (llabs(offset) > CLASS6_DEMOTE_NS);
#define LOCK_LOSS_AVG_NS 1000000LL
#define LOCK_LOSS_AVG_COUNT 60
            static int avg_bad_count = 0;

            if (g_avg_count >= AVG_WINDOW && llabs(offset) > LOCK_LOSS_AVG_NS)
            {
                avg_bad_count++;
                if (avg_bad_count >= LOCK_LOSS_AVG_COUNT)
                {
                    ESP_LOGE("SERVO",
                             "LOCK LOST: averaged offset >%lld ns for %d s. "
                             "Full re-acquire.",
                             (long long)LOCK_LOSS_AVG_NS, avg_bad_count);
                    state = SERVO_UNLOCKED;
                    g_phc_residual_measured = false;
                    g_integral = 0;
                    g_freq_ppb = 0;
                    stable_count = 0;
                    prev_acq_offset = INT64_MIN;
                    acq_win_n = 0;
                    acq_win_seen = 0;
                    measure_count = 0;
                    offset_acc = 0;
                    median_filter_reset();
                    avg_reset();
                    g_last_offset = 0;
                    avg_bad_count = 0;
                    /* g_total_rejects is lifetime — NOT reset on re-acquire. */
                    g_consecutive_rejects = 0;
                    led_set_state(LED_STATE_ACQUIRING);
                    g_lock_start_ms = 0;
                    g_lock_acquired_ms = 0;
                    g_settled_after_holdover = false;
                    continue;
                }
            }
            else
            {
                avg_bad_count = 0;
            }

            /* Update lock quality ring buffer (screen 2 OLED display).
             * Uses RAW offset relative to the averaged baseline — this is
             * "how noisy is the receiver right now", not "how well are we
             * tracking the average" (which is trivially always ~0 because
             * we're averaging across 32 samples). */
            int64_t raw_dev = llabs(raw_offset - offset);
            g_lock_qual_ring[g_lock_qual_idx] = (raw_dev < LOCK_QUAL_GOOD_NS) ? 1 : 0;
            g_lock_qual_idx = (g_lock_qual_idx + 1) % LOCK_QUAL_WINDOW;
            if (g_lock_qual_count < LOCK_QUAL_WINDOW)
                g_lock_qual_count++;

            /* Track recent raw spread for display. Reset every full window
             * so the displayed min/max reflects "recent" behavior, not the
             * lifetime extreme of one cosmic-ray outlier. */
            if (raw_offset < g_qual_raw_min)
                g_qual_raw_min = raw_offset;
            if (raw_offset > g_qual_raw_max)
                g_qual_raw_max = raw_offset;
            if (++g_qual_raw_samples >= LOCK_QUAL_WINDOW)
            {
                g_qual_raw_samples = 0;
                /* Decay bounds toward current sample rather than hard reset —
                 * keeps display from flickering between extremes. */
                g_qual_raw_min = raw_offset;
                g_qual_raw_max = raw_offset;
            }

            ESP_LOGI("SERVO",
                     "state=%d raw=%lld avg=%lld d=%.0f integ=%.0f adj=%ld ppb",
                     state,
                     (long long)raw_offset,
                     (long long)offset,
                     d_term,
                     g_integral,
                     (long)adj_ppb);

            continue;
        }
        }
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Background tasks
 *
 * gnss_task           — reads the GNSS UART, feeds the TAU1201 configurator
 *                      while it runs, and dispatches NMEA lines to parsers.
 * gnss_signal_task    — periodic (30 s) signal-quality / PPS-jitter / SV-churn
 *                      diagnostics report.
 * ptp_rx_task        — L2TAP receive loop; dispatches Delay_Req and Announce,
 *                      and runs the BMCA announce-timeout reclaim.
 * ptp_tx_task        — periodic Sync/Follow_Up (1 Hz) and Announce (0.5 Hz)
 *                      transmit loop while we are the active master.
 * ═════════════════════════════════════════════════════════════════════════ */

static void gnss_task(void *arg)
{
    static char line[128];
    int idx = 0;

    /* Chunked UART reads: one driver call per up-to-64-byte burst instead
     * of one per byte (~1-2k mutex/ringbuffer round-trips per second at
     * full NMEA rate). The timeout is deliberately SHORT (1 tick = 10 ms):
     * it only matters for the tail bytes of each 1 Hz burst, and it bounds
     * the added parse latency of the burst's final sentence to one tick.
     * That bound is load-bearing — the pps_task sentence_offset heuristic
     * depends on time-bearing sentences landing consistently on the same
     * side of the PPS edge, and a longer timeout (e.g. 100 ms) would eat
     * visibly into that sentence-to-edge margin. Idle cost of the short
     * timeout is ~100 empty wakeups/s at priority 5 — negligible. */
    static uint8_t chunk[64];

    while (1)
    {
        int len = uart_read_bytes(GNSS_UART_NUM, chunk, sizeof(chunk),
                                  pdMS_TO_TICKS(10));

        if (len <= 0)
            continue;

        /* Fork the whole chunk into the TAU1201 timing-mode configurator
         * while it's still running — this batch call is the interface
         * feed_data was designed for. Once the sequence has finished
         * (success OR failure), skip the call entirely; the direct
         * flag-check avoids the function call across the translation-unit
         * boundary. */
        if (!initialization_sequence_done)
        {
            allystar_timing_feed_data(chunk, (size_t)len);
        }

        for (int ci = 0; ci < len; ci++)
        {
            uint8_t c = chunk[ci];

            // NMEA line termination
            if (c == '\n')
            {
                line[idx] = 0;

                /* Checksum gate: only sentences whose "*XX" verifies get
                 * parsed. Binary Allystar frames and boot-time partial
                 * lines land here too and are dropped without logging
                 * (see nmea_checksum_ok). */
                if (nmea_checksum_ok(line))
                {
                    if (strstr(line, "ZDA"))
                    {
                        parse_zda(line);
                    }
                    else if (strstr(line, "RMC"))
                    {
                        parse_rmc(line);
                    }
                    else if (strstr(line, "GGA"))
                    {
                        parse_gga(line);
                    }
                    else if (strstr(line, "GSV"))
                    {
                        parse_gsv(line);
                    }
                    else if (strstr(line, "GSA"))
                    {
                        parse_gsa(line);
                    }
                }

                idx = 0;
            }
            else if (c != '\r')
            {
                if (idx < sizeof(line) - 1)
                {
                    line[idx++] = (char)c;
                }
                else
                {
                    idx = 0;
                }
            }
        }
    }
}

/* Periodic signal-quality report — runs once every 30 seconds. */
static void gnss_signal_task(void *arg)
{
    /* Let things stabilize before first report */
    vTaskDelay(pdMS_TO_TICKS(15000));

    while (1)
    {
        uint8_t avg, mn, mx, sv_view, sv_used;
        int64_t last_ms;

        portENTER_CRITICAL(&g_gnss_sig_lock);
        avg = g_gnss_snr_avg;
        mn = g_gnss_snr_min;
        mx = g_gnss_snr_max;
        sv_view = g_gnss_sv_in_view;
        sv_used = g_gnss_sv_used;
        last_ms = g_gnss_sig_last_update_ms;
        portEXIT_CRITICAL(&g_gnss_sig_lock);

        int64_t age_ms = (esp_timer_get_time() / 1000) - last_ms;

        if (last_ms == 0)
        {
            ESP_LOGW("GNSS_SIG", "no GSV data received yet — receiver module may not be emitting it");
        }
        else if (age_ms > 5000)
        {
            ESP_LOGW("GNSS_SIG", "GSV stale (last update %lld ms ago)", (long long)age_ms);
        }
        else
        {
            const char *verdict;
            if (avg >= 40)
                verdict = "excellent";
            else if (avg >= 35)
                verdict = "good";
            else if (avg >= 30)
                verdict = "fair";
            else if (avg >= 25)
                verdict = "weak";
            else
                verdict = "noisy";

            ESP_LOGI("GNSS_SIG",
                     "SNR: avg=%u min=%u max=%u dB-Hz | SVs: view=%u used=%u | %s",
                     avg, mn, mx, sv_view, sv_used, verdict);

            if (sv_used < 4 && sv_used > 0)
                ESP_LOGW("GNSS_SIG",
                         "Only %u SVs in fix — receiver may degrade to time-only solution",
                         sv_used);

            /* ── PPS edge-to-edge jitter (snapshot + reset) ──
             * This is the *raw* receiver timing noise, independent of the
             * servo. If this is small (< few µs) but offset still jumps,
             * the problem is the receiver's time *solution* changing
             * (constellation churn). If this is large, the receiver's
             * PPS edge itself is jittering. The two diagnoses point at
             * different fixes. */
            int64_t jit_min, jit_max, jit_sum_abs;
            uint32_t jit_count, jit_outliers;
            portENTER_CRITICAL(&g_pps_jit_lock);
            jit_min = g_pps_jit_min_ns;
            jit_max = g_pps_jit_max_ns;
            jit_sum_abs = g_pps_jit_sum_abs_ns;
            jit_count = g_pps_jit_count;
            jit_outliers = g_pps_jit_outliers;
            g_pps_jit_min_ns = INT64_MAX;
            g_pps_jit_max_ns = INT64_MIN;
            g_pps_jit_sum_abs_ns = 0;
            g_pps_jit_count = 0;
            g_pps_jit_outliers = 0;
            portEXIT_CRITICAL(&g_pps_jit_lock);

            if (jit_count > 0)
            {
                int64_t mean_abs = jit_sum_abs / (int64_t)jit_count;
                ESP_LOGI("GNSS_SIG",
                         "PPS jitter (last %lus): mean|j|=%lld ns min=%+lld max=%+lld "
                         "outliers>100us=%lu / %lu samples",
                         30UL, (long long)mean_abs,
                         (long long)jit_min, (long long)jit_max,
                         (unsigned long)jit_outliers, (unsigned long)jit_count);
                /* Warn only when the *measured* jitter is actually high — not on
                 * SNR alone. A weak constellation with a clean PPS edge (e.g.
                 * mean|j| < 1 us) is still perfectly usable for the servo. */
                if (mean_abs > 5000)
                {
                    if (avg < 30)
                        ESP_LOGW("GNSS_SIG",
                                 "Weak signal (SNR avg=%u dB-Hz): PPS jitter is high. "
                                 "Antenna placement or sky view is the limit.",
                                 avg);
                    ESP_LOGW("GNSS_SIG",
                             "→ PPS edge jitter exceeds 5 µs mean — receiver PPS quality is the bottleneck "
                             "(not constellation churn)");
                }
            }
            else
            {
                if (avg < 30)
                    ESP_LOGW("GNSS_SIG",
                             "Weak signal (SNR avg=%u dB-Hz) and no PPS edges measured this window.", avg);
                else
                    ESP_LOGW("GNSS_SIG", "PPS jitter: no edges measured (PHC not stepped yet, or PPS disconnected?)");
            }

            /* ── SV churn (snapshot + reset) ──
             * Per-talker entries+exits over the window. A churn rate of
             * a few SVs per minute is normal as low-elevation satellites
             * cross the horizon. Dozens per minute = the receiver is
             * struggling to maintain lock and constantly cycling which
             * SVs are in the position-fix solution. That's exactly the
             * pattern that produces time-solution jumps even at constant
             * SV count. */
            uint32_t entries, exits, gsa_seen;
            portENTER_CRITICAL(&g_sv_churn_lock);
            entries = g_sv_entries;
            exits = g_sv_exits;
            gsa_seen = g_sv_gsa_seen;
            g_sv_entries = 0;
            g_sv_exits = 0;
            g_sv_gsa_seen = 0;
            portEXIT_CRITICAL(&g_sv_churn_lock);

            ESP_LOGI("GNSS_SIG",
                     "SV churn (last %lus): %lu entries, %lu exits across %lu GSA sentences",
                     30UL,
                     (unsigned long)entries,
                     (unsigned long)exits,
                     (unsigned long)gsa_seen);
            if (entries + exits > 30)
                ESP_LOGW("GNSS_SIG",
                         "→ High SV churn (%lu changes in 30 s) — constellation in fix is unstable, "
                         "expect time-solution jumps",
                         (unsigned long)(entries + exits));
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * Tasks
 * ───────────────────────────────────────────────────────────────────────── */
/* BMCA announce-timeout reclaim (moved out of ptp_rx_task unchanged, plus the
 * listening rule): while not master, take the master role when no Announce
 * has been heard for ANNOUNCE_TIMEOUT_SEC. While the link is down nothing can
 * be heard, so the window restarts: after boot or a link drop the box listens
 * for a full timeout on a live link before it may transmit. Only ptp_rx_task
 * calls this (single writer of g_i_am_master). */
static void bmca_timeout_check(void)
{
    if (g_i_am_master)
        return;
    time_t now = time(NULL);
    if (!atomic_load(&g_link_up))
    {
        g_last_announce_rx = now; /* listening restarts at link-up */
        return;
    }
    if ((now - g_last_announce_rx) > ANNOUNCE_TIMEOUT_SEC)
    {
        bmca_view_write_begin();
        g_bmca_view.peer_valid = false; /* the peer is gone */
        g_bmca_view.peer_better = false;
        g_bmca_view.decided_by = BMCA_FIELD_NONE;
        bmca_view_write_end();
        bmca_set_role(true, BMCA_REASON_ANNOUNCE_TIMEOUT, NULL, BMCA_FIELD_NONE);
    }
}

static void ptp_rx_task(void *arg)
{
    /* Wait for L2TAP to be ready (set by timing_core_bringup at init, before
     * any link is required — see the timing-core decoupling). */
    while (!atomic_load(&g_l2tap_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    g_last_announce_rx = time(NULL); /* LISTENING starts now (see g_i_am_master) */

    // ESP_LOGI(TAG, "ptp_rx_task started");

    size_t irec_space = L2TAP_IREC_SPACE(sizeof(struct timespec));

    union
    {
        uint8_t buf[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } irec_u;

    /* Allocate RX scratch buffer once for the lifetime of this task.
     * Never freed because this task never exits. Explicit check rather
     * than assert(): under a production profile with assertions silenced
     * or disabled, assert() compiles away and a boot-time allocation
     * failure would surface later as a NULL dereference inside read().
     * PTP receive cannot run without this buffer — flag FAULT and park. */
    uint8_t *frame_buf = calloc(1, 1536);
    if (frame_buf == NULL)
    {
        ESP_LOGE(TAG, "ptp_rx: RX buffer allocation failed — PTP receive disabled");
        led_set_state(LED_STATE_FAULT);
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        struct timeval tv = {.tv_sec = 0, .tv_usec = 5000};

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(g_l2tap_fd, &rfds);

        int sel = select(g_l2tap_fd + 1, &rfds, NULL, NULL, &tv);
        if (sel < 0)
        {
            // ESP_LOGE(TAG, "select error: %s", strerror(errno));
            vTaskDelay(1);
            continue;
        }

        // Timeout -> still need to check Announce timeout
        if (sel == 0)
        {
            goto timeout_check;
        }

        memset(&irec_u, 0, sizeof(irec_u));

        l2tap_irec_hdr_t *irec = (l2tap_irec_hdr_t *)irec_u.buf;
        irec->type = L2TAP_IREC_TIME_STAMP;
        irec->len = L2TAP_IREC_LEN(sizeof(struct timespec));

        l2tap_extended_buff_t ext = {
            .info_recs_len = irec_space,
            .info_recs_buff = irec_u.buf,
            .buff_len = 1536,
            .buff = frame_buf,
        };

        ssize_t n = read(g_l2tap_fd, &ext, 0);
        if (n <= 0)
        {
            vTaskDelay(1);
            continue;
        }

        // Extract RX hardware timestamp
        struct timespec hw_ts = {0};
        if (irec->type == L2TAP_IREC_TIME_STAMP)
        {
            memcpy(&hw_ts, irec->data, sizeof(struct timespec));
        }

        uint8_t *eth_frame = ext.buff;
        size_t eth_len = ext.buff_len;

        if (eth_len < 14 + sizeof(ptp_header_t))
            goto timeout_check;

        const ptp_header_t *rx_hdr = (const ptp_header_t *)(eth_frame + 14);

        /* Accept only PTPv2 frames in OUR domain. versionPTP is the low
         * nibble of its byte (majorVersion); the high nibble is v2.1's
         * minorVersion and is deliberately ignored for compatibility.
         * Without this gate, a Delay_Req from a neighboring domain was
         * answered with a domain-0 response (cross-domain corruption), and
         * foreign-domain or v1 Announces fed our BMCA. */
        if ((rx_hdr->version_ptp & 0x0F) != 2 ||
            rx_hdr->domain_number != PTP_DOMAIN)
            goto timeout_check;

        uint8_t msg_type = eth_frame[14] & 0x0F;

        // ESP_LOGD("PTP_RX", "Type 0x%X @ %lld.%09ld",
        //          msg_type,
        //          (long long)hw_ts.tv_sec,
        //          (long)hw_ts.tv_nsec);

        switch (msg_type)
        {
        case PTP_MSG_DELAY_REQ:
            /* Only the ACTING master answers Delay_Req. When BMCA has
             * ceded mastership (g_i_am_master == false) we must stay
             * silent: the multicast request is addressed to the winning
             * master, and a second Delay_Resp carrying OUR (different)
             * receiveTimestamp would corrupt every slave's path-delay
             * math. Mirrors the g_i_am_master gate ptp_tx_task applies to
             * Sync/Announce. (g_i_am_master is only ever written by this
             * task — handle_announce and the reclaim below — so the read
             * is race-free.)
             * Snapshot + enqueue only; delay_resp_task builds and sends.
             * No logging on the receive hot path. */
            if (g_i_am_master)
                enqueue_delay_req(eth_frame + 14, eth_len - 14, &hw_ts);
            break;

        case PTP_MSG_ANNOUNCE:
            handle_announce(eth_frame + 14, eth_len - 14);
            break;

        default:
            break;
        }

    timeout_check:
        /* BMCA announce-timeout reclaim: if we're currently acting as a slave
         * and haven't heard an Announce from our best master for longer than
         * ANNOUNCE_TIMEOUT_SEC, reclaim the master role. */
        bmca_timeout_check();

        // ESP_LOGW("PTP_RX_TASK", "PTP RX task is working");
    } /* end of while(1) */
} /* end of ptp_rx_task */

static void ptp_tx_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();

    while (1)
    {
        if (atomic_load(&g_link_up) &&
            atomic_load(&g_l2tap_ready) &&
            g_i_am_master)
        {
            /* No Sync/Follow_Up until the PHC holds GNSS time (first lock this
             * boot; the clock stops advertising class 248). Announce still goes
             * out so BMCA keeps working; slaves just get no 1970 timestamps. */
            if (g_ever_locked)
                send_sync_and_followup();

            static int announce_div = 0;
            if (++announce_div >= PTP_ANNOUNCE_DIV)
            {
                send_announce();
                announce_div = 0;
            }
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PTP_SYNC_INTERVAL_MS));
        // ESP_LOGW("PTP_TX_TASK", "PTP TX task is working");
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Health monitoring
 *
 * heap_monitor_task — logs free/min/largest internal heap every 60 s.
 *                     Watch for monotonically decreasing free heap → leak.
 *
 * emac_restart_preserve_phc — restarts the EMAC driver without losing PTP
 *                     time sync, by snapshotting and extrapolating the PHC
 *                     across the dead window.
 *
 * watchdog_task     — checks every 30 s that the PHC seconds counter is
 *                     advancing. Logs a warning after >30 s of freeze and
 *                     attempts an EMAC restart after >60 s of freeze (bounded
 *                     by g_emac_restart_count < 10). Provides self-healing for
 *                     slow RX descriptor exhaustion. (Progress-tracking globals
 *                     are in the global-state section: g_last_progress_phc_s,
 *                     g_last_progress_check_ms, g_emac_restart_count.)
 * ═════════════════════════════════════════════════════════════════════════ */

static void heap_monitor_task(void *arg)
{
    /* Wait a bit for system to settle before first measurement */
    vTaskDelay(pdMS_TO_TICKS(10000));

    size_t baseline_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI("HEAP", "Baseline free internal heap: %u bytes",
             (unsigned)baseline_free);
    /* Published for status_handler's heap_leak field. baseline_free is never
     * 0 in practice (there's always some internal heap free), so 0 stays a
     * safe "not yet measured" sentinel for the reader side. */
    atomic_store(&g_heap_baseline_free, (uint32_t)baseline_free);

    int sample_count = 0;

    while (1)
    {
        size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t min_internal = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        size_t lfb_internal = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

        /* Compute drift from baseline (negative = leak) */
        int32_t drift = (int32_t)free_internal - (int32_t)baseline_free;

        ESP_LOGI("HEAP",
                 "free=%u min=%u largest=%u drift=%+ld restart_cnt=%lu",
                 (unsigned)free_internal,
                 (unsigned)min_internal,
                 (unsigned)lfb_internal,
                 (long)drift,
                 (unsigned long)g_emac_restart_count);

        /* If we've lost more than 32 KB since boot, warn loudly */
        if (drift < -32768)
        {
            ESP_LOGW("HEAP", "Possible leak: %ld bytes lost since boot",
                     (long)-drift);
        }

        /* Every 5 samples (5 minutes), dump detailed heap stats.
         * This tells us whether the leak manifests as growing block count
         * (real leak) or just fragmentation. */
        if (++sample_count >= 5)
        {
            sample_count = 0;
            ESP_LOGI("HEAP", "===== Detailed heap dump (5-min interval) =====");
            heap_caps_print_heap_info(MALLOC_CAP_INTERNAL);
            ESP_LOGI("HEAP", "==============================================");
        }

        vTaskDelay(pdMS_TO_TICKS(HEAP_MON_INTERVAL_MS));
        // ESP_LOGW("HEAP", "HEAP MONITOR WORKING - free=%u bytes", (unsigned)free_internal);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * emac_restart_preserve_phc
 *
 * Restart the EMAC driver without losing PTP time synchronization.
 *
 * The naive esp_eth_stop()/esp_eth_start() cycle resets the PHC counter
 * to zero, which destroys time sync. Instead we:
 *   1. Snapshot PHC + esp_timer simultaneously
 *   2. Stop driver
 *   3. Start driver
 *   4. Re-enable PTP
 *   5. Compute extrapolated PHC time using esp_timer + servo's freq
 *      adjustment, write to PHC immediately
 *   6. Re-apply frequency adjustment
 *
 * Expected residual offset after restart: < 1µs (dominated by the time
 * between sampling esp_timer and the EMAC actually starting to count
 * after ETH_MAC_ESP_CMD_S_PTP_TIME). The servo will pull this back in
 * a few seconds.
 *
 * NOTE: This still loses PTP packet flow during the ~120ms dead window.
 * Slaves locked to this master will see one missed Sync interval.
 * ───────────────────────────────────────────────────────────────────────── */
static void emac_restart_preserve_phc(void)
{
    ESP_LOGW("WDT", "Attempting EMAC restart with PHC preservation (count=%lu)",
             (unsigned long)(g_emac_restart_count + 1));

    /* Two-stage gating against in-flight L2TAP TX:
     *   (1) Set g_emac_restarting so new l2tap_send_ptp / servo_phc_ioctl
     *       callers fast-reject (return -1 / false) instead of touching the
     *       EMAC while we're stopping it.
     *   (2) Take g_l2tap_tx_mtx so any caller already past the atomic check
     *       on the other core — i.e. mid-write() right now — completes
     *       before we call esp_eth_stop(). Released in the cleanup block
     *       below; every fail path inside the do/while(0) uses `break` so
     *       the mutex never leaks. */
    atomic_store(&g_emac_restarting, true);
    if (g_l2tap_tx_mtx)
        xSemaphoreTake(g_l2tap_tx_mtx, portMAX_DELAY);

    /* Single-iteration "break loop" so every fail path exits to the
     * cleanup below without goto and without repeating cleanup code. */
    do
    {
        /* ── Step 1: Snapshot PHC and esp_timer simultaneously ──
         * Read PHC first, then esp_timer immediately. Keep these two
         * statements adjacent — any code between them adds capture error.
         * Disable interrupts to ensure no preemption between samples. */
        struct timespec ts_before;
        int64_t mono_before_us;

        /* The critical section's real job is LOCAL: it pins the PHC read
         * and the esp_timer read back-to-back with no preemption or
         * interrupt between them on this core — that adjacency is what
         * bounds the snapshot's capture error. It does NOT serialize
         * servo_task's PHC ioctls on the other core (servo never takes
         * this mux around them); cross-core exclusion comes from
         * g_emac_restarting, set above, which fast-rejects every servo
         * PHC write and every PPS-ISR capture for the whole restart
         * window. */
        taskENTER_CRITICAL(&g_servo_spinlock);
        emac_get_time(&ts_before);
        mono_before_us = esp_timer_get_time();
        taskEXIT_CRITICAL(&g_servo_spinlock);

        int64_t phc_ns_before = (int64_t)ts_before.tv_sec * 1000000000LL +
                                (int64_t)ts_before.tv_nsec;

        /* Snapshot the frequency adjustment to re-apply after the restart.
         * Read the lock-free g_freq_ppb_hw mirror — the int32 the hardware
         * last received — NOT g_freq_ppb: that double is written by
         * servo_task without any lock (a spinlocked read here never
         * serialized anything), and a torn cross-core double read would
         * extrapolate the dead window with garbage and re-step the PHC
         * arbitrarily wrong. The mirror is stable for the whole window:
         * g_emac_restarting (set above) blocks new ADJ writes, and a write
         * in flight at flag-set time was applied to hardware either way. */
        int32_t saved_freq_ppb = atomic_load(&g_freq_ppb_hw);

        ESP_LOGI("WDT", "PHC snapshot: %lld.%09ld (mono=%lld us, freq_ppb=%ld)",
                 (long long)ts_before.tv_sec,
                 (long)ts_before.tv_nsec,
                 (long long)mono_before_us,
                 (long)saved_freq_ppb);

        /* ── Step 2: Stop driver ── */
        esp_err_t err = esp_eth_stop(g_eth_hndl);
        if (err != ESP_OK)
        {
            ESP_LOGE("WDT", "esp_eth_stop failed: %s", esp_err_to_name(err));
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        /* ── Step 3: Restart driver ── */
        err = esp_eth_start(g_eth_hndl);
        if (err != ESP_OK)
        {
            ESP_LOGE("WDT", "esp_eth_start failed: %s", esp_err_to_name(err));
            break;
        }

        /* ── Step 4: Re-enable PTP ── */
        bool ptp_en = true;
        esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &ptp_en);

        /* Brief delay to let PHC start ticking */
        vTaskDelay(pdMS_TO_TICKS(20));

        /* ── Step 5: Compute extrapolated time and write to PHC ──
         * We want to set PHC to "what it would have been if it never stopped."
         *
         * Elapsed monotonic time across the dead window:
         *   dt_us = mono_now - mono_before_us
         *
         * Apply the servo's frequency adjustment so PHC advances at the
         * same rate it WOULD have if it had been running:
         *   PHC_advance_ns = dt_us × 1000 × (1 + freq_ppb × 1e-9)
         *
         * For a 120ms dead window with freq_ppb = -47000:
         *   freq correction = -47000 × 1e-9 × 120e6 ns = -5640 ns
         * Without this correction, we'd be off by ~5.6µs.
         *
         * Read esp_timer as late as possible — right before the ioctl
         * write — to minimize gap between our extrapolation and the write
         * taking effect. */
        int64_t mono_after_us = esp_timer_get_time();
        int64_t dt_us = mono_after_us - mono_before_us;
        int64_t dt_ns = dt_us * 1000LL;

        /* Apply frequency correction. saved_freq_ppb is in ppb (1e-9 units). */
        int64_t freq_correction_ns = (int64_t)((double)dt_ns * (double)saved_freq_ppb * 1e-9);
        int64_t target_ns = phc_ns_before + dt_ns + freq_correction_ns;

        if (target_ns < 0)
        {
            ESP_LOGE("WDT", "Computed negative target time, aborting");
            break;
        }

        eth_mac_time_t t = {
            .seconds = (uint32_t)(target_ns / 1000000000LL),
            .nanoseconds = (uint32_t)(target_ns % 1000000000LL),
        };

        esp_err_t set_err = esp_eth_ioctl(g_eth_hndl,
                                          ETH_MAC_ESP_CMD_S_PTP_TIME,
                                          &t);
        if (set_err != ESP_OK)
        {
            ESP_LOGE("WDT", "S_PTP_TIME failed: %s", esp_err_to_name(set_err));
            break;
        }

        /* ── Step 6: Re-apply frequency adjustment ── */
        int32_t adj_ppb = saved_freq_ppb;
        esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_ppb);

        g_emac_restart_count++;

        ESP_LOGW("WDT",
                 "EMAC restart complete: dt=%lld us, freq_corr=%lld ns, target=%u.%09u",
                 (long long)dt_us,
                 (long long)freq_correction_ns,
                 t.seconds, t.nanoseconds);
    } while (0);

    /* Cleanup (runs on every exit, success or failure).
     * Reverse of the entry order: release the TX mutex first (any callers
     * blocked on it can now wake), then clear the atomic flag so new
     * callers can pass the fast-reject check. Both orderings are race-free;
     * this one matches the conventional acquire/release nesting. */
    if (g_l2tap_tx_mtx)
        xSemaphoreGive(g_l2tap_tx_mtx);
    atomic_store(&g_emac_restarting, false);
}

/* Read the PHC's live seconds counter — the watchdog's progress signal.
 * The PHC ticks in hardware whenever the EMAC PTP block is alive, through
 * holdover, link loss and GNSS removal alike, so "not advancing" really
 * does mean the EMAC needs help (unlike the PPS capture, which merely
 * means the GNSS stopped pulsing — see the Check 1 comment below). */
static uint64_t watchdog_phc_seconds(void)
{
    struct timespec ts = {0};
    emac_get_time(&ts);
    return (uint64_t)ts.tv_sec;
}

static void watchdog_task(void *arg)
{
    /* Wait 60 s for system to fully come up before arming */
    vTaskDelay(pdMS_TO_TICKS(60000));

    g_last_progress_phc_s = watchdog_phc_seconds();
    g_last_progress_check_ms = esp_timer_get_time() / 1000;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(30000));

        uint64_t now_ms = esp_timer_get_time() / 1000;

        /* For the JSON "wdt" object: exactly one alert increment per pass
         * that found a problem (not per condition), decided after both
         * checks below have run. */
        bool wdt_problem = false;
        wdt_state_t wdt_new_state = WDT_OK;

        /* ───── Check 1: PHC freeze ─────
         * Monitor the PHC ITSELF (hardware seconds counter, read via
         * emac_get_time), NOT g_pps_capture_ns. The PPS capture only
         * advances when GNSS PPS edges arrive, so sampling it here
         * conflated "PPS stopped" (holdover, antenna unplugged, no GNSS
         * module fitted — exactly when clock stability matters most, and
         * nothing an EMAC restart can fix) with "EMAC PTP block wedged"
         * (the one condition the restart IS for). The old form restarted
         * the EMAC up to 10 times — bouncing the link, re-running DHCP
         * and re-stepping the PHC each time — on every PPS outage longer
         * than a minute.
         *
         * Gated on g_phc_ready: before timing_core_bringup() succeeds the
         * PHC is legitimately not counting, and if bring-up FAILED the box
         * is already in LED FAULT, where 10 EMAC restarts would not help.
         * g_phc_ready reaches its final value during app_main, long before
         * this task arms, so the gate is effectively static: either the
         * arming baseline above was a live PHC read, or this branch never
         * runs. */
        if (atomic_load(&g_phc_ready))
        {
            uint64_t cur_phc_s = watchdog_phc_seconds();
            bool phc_frozen = (cur_phc_s == g_last_progress_phc_s);
            uint64_t elapsed = now_ms - g_last_progress_check_ms;

            if (phc_frozen && elapsed > 30000)
            {
                wdt_problem = true;
                wdt_new_state = WDT_PHC_FROZEN;

                ESP_LOGW("WDT",
                         "PHC frozen for %llu ms (phc_s=%llu) — health degraded",
                         (unsigned long long)elapsed,
                         (unsigned long long)cur_phc_s);

                /* Only attempt recovery after >60s freeze AND <10 prior restarts */
                if (elapsed > 60000 && g_emac_restart_count < 10)
                {
                    emac_restart_preserve_phc();

                    /* After restart, give system a moment to recover before next check */
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    g_last_progress_phc_s = watchdog_phc_seconds();
                    g_last_progress_check_ms = esp_timer_get_time() / 1000;
                }
                else if (g_emac_restart_count >= 10)
                {
                    wdt_new_state = WDT_RESTART_LIMIT;
                    ESP_LOGE("WDT",
                             "EMAC restart limit reached — system needs reboot");
                }
            }
            else if (!phc_frozen)
            {
                g_last_progress_phc_s = cur_phc_s;
                g_last_progress_check_ms = now_ms;
            }
        }

        /* ───── Check 2: Heap pressure ─────
         * Now that the my_timegm leak is fixed, this should never trigger
         * under normal operation. If it does, there's a secondary leak we
         * missed — try a restart anyway in case it clears lwIP buffers.
         * This is a hail-mary: pressure caused by something other than the
         * EMAC/lwIP pools (httpd, OTA) won't be helped by it, so the
         * attempt is rate-limited to one per 5 minutes — sustained low
         * heap must not burn the whole 10-restart budget (shared with
         * Check 1) in five consecutive 30 s polls. The ESP_LOGE fires on
         * every poll regardless, so the condition stays loudly visible. */
        size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

        if (free_now < 50000)
        {
            static uint64_t last_heap_restart_ms = 0;

            wdt_problem = true;
            if (wdt_new_state == WDT_OK)
                wdt_new_state = WDT_HEAP_CRITICAL;

            ESP_LOGE("WDT",
                     "Critical heap pressure (%u bytes free) — attempting recovery",
                     (unsigned)free_now);

            if (g_emac_restart_count < 10 &&
                (last_heap_restart_ms == 0 ||
                 now_ms - last_heap_restart_ms >= 300000))
            {
                last_heap_restart_ms = now_ms;
                emac_restart_preserve_phc();
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
        // ESP_LOGW("WDT", "Watchdog check complete: PHC %s, free heap %u bytes, restart count %lu",
        //          phc_frozen ? "FROZEN" : "OK",
        //          (unsigned)free_now,
        //          (unsigned long)g_emac_restart_count);

        atomic_store(&g_wdt_state, (uint8_t)wdt_new_state);
        if (wdt_problem)
        {
            atomic_fetch_add(&g_wdt_alerts, 1);
            atomic_store(&g_wdt_last_alert_s,
                        (uint32_t)((now_ms - (uint64_t)g_boot_time_ms) / 1000));
        }
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: LED status indicator
 *
 * Single GPIO status LED, low-priority task, drives blink patterns from the
 * servo state. A second GPIO LED pulses once per PPS edge.
 *
 * Status-LED patterns:
 *   Solid off  : no Ethernet link (cable unplugged, PHY down)
 *   Slow blink : link up, but not yet locked (no GNSS, or servo acquiring)
 *   Solid on   : LOCKED — time output is trustworthy
 *   Heartbeat  : HOLDOVER — was locked, lost GNSS, still serving (degrading)
 *   Fast blink : FAULT — something broken, check logs
 *
 * The task reads a shared atomic state variable. It never blocks the servo,
 * PPS handler, or RX/TX paths. A 10 ms tick gives clean pattern timing
 * without measurable CPU cost.
 * ═════════════════════════════════════════════════════════════════════════ */

// Public API: any task/ISR-safe context can call this.
void led_set_state(led_state_t new_state)
{
    atomic_store(&s_led_state, new_state);
}

static void led_write(bool on)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_HIGH ? on : !on);
}

static void led_task(void *arg)
{
    // Configure pin
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    led_write(false);

    ESP_LOGI(LED_TAG, "LED task started on GPIO %d", LED_GPIO);

    // Tick counter wraps every 100 ticks = 1 second at 10 ms tick
    uint32_t tick = 0;
    const TickType_t period = pdMS_TO_TICKS(LED_TICK_MS);
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        led_state_t state = atomic_load(&s_led_state);
        bool on = false;

        switch (state)
        {
        case LED_STATE_NO_LINK:
            // Always off
            on = false;
            break;

        case LED_STATE_ACQUIRING:
            // 1 Hz, 50% duty: 500 ms on, 500 ms off
            on = (tick % 100) < 50;
            break;

        case LED_STATE_LOCKED:
            // Solid on
            on = true;
            break;

        case LED_STATE_HOLDOVER:
            // Heartbeat: two short flashes per second
            // Pattern over 100 ticks (1 s):
            //   on  for ticks  0..7   (80 ms)
            //   off for ticks  8..19  (120 ms)
            //   on  for ticks 20..27  (80 ms)
            //   off for ticks 28..99  (720 ms)
            {
                uint32_t t = tick % 100;
                on = (t < 8) || (t >= 20 && t < 28);
            }
            break;

        case LED_STATE_FAULT:
            // 5 Hz, 50% duty: 100 ms on, 100 ms off
            on = (tick % 20) < 10;
            break;

        default:
            on = false;
            break;
        }

        led_write(on);
        tick++;
        vTaskDelayUntil(&last_wake, period);
    }
}

void led_start(void)
{
    task_create_tracked(led_task, "led", LED_TASK_STACK, NULL, LED_TASK_PRIO, NULL);
}

static void pps_led_task(void *arg)
{
    /* Configure pin */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PPS_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PPS_LED_GPIO, PPS_LED_ACTIVE_HIGH ? 0 : 1); /* off */

    ESP_LOGI("PPS_LED", "task started on GPIO %d (pulse=%d ms)",
             PPS_LED_GPIO, PPS_LED_ON_MS);

    uint32_t last_seen_seq = 0;

    while (1)
    {
        uint32_t now_seq = g_pps_capture_seq;

        if (now_seq != last_seen_seq)
        {
            // ESP_LOGI("PPS_IRS", "Current g_pps_capture_seq=%lu", (unsigned long)g_pps_capture_seq);
            last_seen_seq = now_seq;

            /* Pulse on */
            gpio_set_level(PPS_LED_GPIO, PPS_LED_ACTIVE_HIGH ? 1 : 0);
            vTaskDelay(pdMS_TO_TICKS(PPS_LED_ON_MS));
            gpio_set_level(PPS_LED_GPIO, PPS_LED_ACTIVE_HIGH ? 0 : 1);
        }

        /* Poll at 10 ms — fast enough to make the pulse appear synchronous
         * with the edge to a human eye (10 ms is below perception threshold). */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: OLED display + button
 *
 * 128x64 monochrome OLED driven via u8g2, with five screens the operator
 * cycles using the front-panel button:
 *
 *   Screen 1 (main)        — lock state, UTC, offset, GNSS, slave count
 *   Screen 2 (servo)       — servo internals, lock duration, ppb, integrator
 *   Screen 3 (gnss)         — GNSS signal quality and SV counts
 *   Screen 4 (network)     — link, IP, uptime, heap
 *   Screen 5 (timing cfg)  — TAU1201 timing-mode configuration status
 *
 * button_task debounces the screen-change button; display_task is the render
 * loop; the render_screen_* helpers each draw one screen. The display
 * auto-jumps to the timing-config screen on first failure so the operator
 * sees it without scrolling.
 * ═════════════════════════════════════════════════════════════════════════ */

static void button_task(void *arg)
{
    /* Configure button pin */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << DISPLAY_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = DISPLAY_BUTTON_ACTIVE_LOW ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = DISPLAY_BUTTON_ACTIVE_LOW ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    ESP_LOGI("BTN", "Display button task on GPIO %d", DISPLAY_BUTTON_GPIO);

    int stable_count = 0;
    bool last_stable_state = false; // false = released
    bool prev_raw_state = false;

    while (1)
    {
        bool raw = (gpio_get_level(DISPLAY_BUTTON_GPIO) ==
                    (DISPLAY_BUTTON_ACTIVE_LOW ? 0 : 1));

        if (raw == prev_raw_state)
        {
            /* Reading is stable — count up to debounce threshold */
            if (stable_count < BUTTON_DEBOUNCE_TICKS)
            {
                stable_count++;
                if (stable_count == BUTTON_DEBOUNCE_TICKS && raw != last_stable_state)
                {
                    /* New stable state confirmed */
                    last_stable_state = raw;
                    /* Trigger only on press edge (released → pressed) */
                    if (raw == true)
                    {
                        uint8_t cur = atomic_load(&g_display_screen);
                        cur = (cur + 1) % DISPLAY_NUM_SCREENS;
                        atomic_store(&g_display_screen, cur);
                        ESP_LOGI("BTN", "Screen → %u", (unsigned)cur);
                    }
                }
            }
        }
        else
        {
            /* Bouncing or change — reset counter */
            stable_count = 0;
            prev_raw_state = raw;
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void display_task(void *arg)
{
    /* Wait a bit for everything to come up */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Wake display from sleep — InitDisplay leaves it sleeping */
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 12, "PTP Grandmaster");

    /* Device name on two rows (worst case 20 + 12 chars, always fits
     * the 21-column budget).  Falls back to "ESP32-P4" only if the OLED
     * task ever beats ethernet_init to the buffer (it can't in the
     * current app_main ordering, but the guard costs nothing). */
    u8g2_DrawStr(&u8g2, 0, 24, g_host_line1[0] ? g_host_line1 : "ESP32-P4");
    if (g_host_line2[0])
        u8g2_DrawStr(&u8g2, 0, 36, g_host_line2);

    /* Firmware version from version.txt (via the app descriptor). Shown on the
     * boot splash so the running image is identifiable before any network or
     * GNSS is up. Guard against a NULL descriptor just in case. */
    {
        const esp_app_desc_t *app = esp_app_get_description();
        char vbuf[36]; /* "fw " + up to 31-char version + NUL */
        snprintf(vbuf, sizeof(vbuf), "fw %s",
                 (app && app->version[0]) ? app->version : "?");
        u8g2_DrawStr(&u8g2, 0, 48, vbuf);
    }

    u8g2_DrawStr(&u8g2, 0, 60, "Booting...");
    u8g2_SendBuffer(&u8g2);

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI("OLED", "Display task running, %d screens", DISPLAY_NUM_SCREENS);

    /* Auto-jump to timing-config screen the first time a failure is reported,
     * so the operator sees it without having to scroll. After auto-jump we
     * leave navigation to the user. */
    bool auto_jumped = false;

    while (1)
    {
        if (!auto_jumped && g_timing_error_step != NULL)
        {
            atomic_store(&g_display_screen, 4);
            auto_jumped = true;
        }

        uint8_t screen = atomic_load(&g_display_screen);

        u8g2_ClearBuffer(&u8g2);

        switch (screen)
        {
        case 0:
            render_screen_main();
            break;
        case 1:
            render_screen_servo();
            break;
        case 2:
            render_screen_gnss();
            break;
        case 3:
            render_screen_network();
            break;
        case 4:
            render_screen_timing_cfg();
            break;
        default:
            render_screen_main();
            break;
        }

        /* Tiny screen-number indicator in bottom-right corner */
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
        char idx[8];
        snprintf(idx, sizeof(idx), "%u/%u", (unsigned)(screen + 1),
                 (unsigned)DISPLAY_NUM_SCREENS);
        u8g2_DrawStr(&u8g2, 128 - 16, 64, idx);

        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
    }
}

/* ──────────────────────────────────────────────────────────────────────────────────
 * Screen 1: Main status — what users glance at most
 *   - Lock state (big)
 *   - UTC time
 *   - Current offset
 *   - GNSS lock indicator
 *   - TAI offset + active slave count
 * ─────────────────────────────────────────────────────────────────────────────────── */
static void render_screen_main(void)
{
    /* Snapshot state */
    led_state_t state = atomic_load(&s_led_state);
    int64_t offset = g_last_offset;
    bool gnss_valid = g_gnss_valid;
    time_t gnss_sec = g_gnss_seconds;

    /* Read-only mirror of advertise_as_holdover()'s settle condition. We do NOT
     * call that function from the display task: it has the side effect of
     * setting g_settled_after_holdover (see the assignment inside
     * advertise_as_holdover), and racing the servo task to write that flag
     * would make PTP holdover-advertisement depend on screen refresh timing.
     * So we recompute the same predicate here without writing anything.
     * "Settling" = genuinely locked (not in holdover) but still inside the
     * POST_HOLDOVER_SETTLE_MS window after the phase reference was last
     * re-seated, which is exactly the window where PTP still advertises
     * holdover while the box is operationally LOCKED. */
    bool settling = (!g_holdover && g_ever_locked && holdover_advert_ro(NULL));

    char buf[32];

    /* Line 1: Lock state, big font */
    u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
    const char *state_str = "?";
    switch (state)
    {
    case LED_STATE_NO_LINK:
        state_str = "NO LINK";
        break;
    case LED_STATE_ACQUIRING:
        /* Distinguish "never had a fix since boot" from "lost lock, working
         * back". Before the first lock the PHC holds no real time at all, so
         * "WAIT GNSS" is the honest label; after that, a drop back to ACQUIRING
         * is a genuine re-acquisition of a clock that was once correct. */
        state_str = g_ever_locked ? "ACQUIRING" : "WAIT GNSS";
        break;
    case LED_STATE_LOCKED:
        state_str = settling ? "LOCKED (settling)" : "LOCKED";
        break;
    case LED_STATE_HOLDOVER:
        state_str = "HOLDOVER";
        break;
    case LED_STATE_FAULT:
        state_str = "FAULT";
        break;
    }
    u8g2_DrawStr(&u8g2, 0, 12, state_str);

    /* Separator line */
    u8g2_DrawHLine(&u8g2, 0, 16, 128);

    /* UTC time (line 2) */
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    if (gnss_valid && gnss_sec > 1577836800LL)
    {
        struct tm tm_utc;
        time_t t = gnss_sec;
        gmtime_r(&t, &tm_utc);
        snprintf(buf, sizeof(buf), "UTC %02d:%02d:%02d",
                 tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);
    }
    else
    {
        snprintf(buf, sizeof(buf), "UTC --:--:--");
    }
    u8g2_DrawStr(&u8g2, 0, 28, buf);

    /* Offset (line 3) */
    if (state == LED_STATE_LOCKED || state == LED_STATE_HOLDOVER)
    {
        snprintf(buf, sizeof(buf), "Offset: %+lld ns", (long long)offset);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Offset: ---");
    }
    u8g2_DrawStr(&u8g2, 0, 40, buf);

    /* GNSS sat indicator (line 4) — sentinel: GNSS recently updated? */
    time_t now = time(NULL);
    bool gnss_fresh = (now - g_last_gnss_update) < 3;
    snprintf(buf, sizeof(buf), "GNSS: %s", gnss_fresh ? "OK" : "no fix");
    u8g2_DrawStr(&u8g2, 0, 52, buf);

    /* Bottom line: TAI offset + active slave count */
    int slaves = slave_active_count();
    snprintf(buf, sizeof(buf), "TAI+%ds   Sla:%d", g_utc_offset, slaves);
    u8g2_DrawStr(&u8g2, 0, 63, buf);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Screen 2: Servo internals — engineer's view of timing health
 *   - Advertised PTP clockClass (cc:6/7/248), right-aligned on title row
 *   - Lock duration / holdover age
 *   - Quality % + recent raw-offset peak span + lifetime reject total,
 *     OR reject-streak warning "REJ n/45 !" when a streak is active
 *   - Frequency adjustment (ppb)
 *   - Integrator value + PPS edge counter (compacted with k/M suffixes)
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_servo(void)
{
    led_state_t state = atomic_load(&s_led_state);
    double freq_ppb = g_freq_ppb;
    double integ = g_integral;
    uint32_t pps_count = g_pps_capture_seq;
    int64_t lock_start = g_lock_start_ms;
    int64_t now_ms = esp_timer_get_time() / 1000;
    uint32_t rej_streak = g_consecutive_rejects; /* reject-streak watchdog */
    uint32_t rej_total = g_total_rejects;        /* lifetime reject count */

    char buf[32];

    /* Live advertised PTP clockClass, recomputed READ-ONLY for display. We do
     * NOT call advertised_clock_class() here: it routes through
     * advertise_as_holdover(), which has the side effect of latching
     * g_settled_after_holdover. Racing servo_task to write that flag would make
     * PTP holdover-advertisement depend on screen-refresh timing (same reason
     * screen 1 recomputes 'settling' inline). So we mirror the predicate here
     * with no writes. This is exactly what's going out on the Announce wire:
     *   248 = never synchronized since boot (PHC holds no real time)
     *     7 = holdover (was locked, coasting)
     *     6 = locked to GNSS
     * Lets you confirm wire state and the screen-1 label agree at a glance. */
    uint8_t disp_cc;
    if (!g_ever_locked)
        disp_cc = 248;
    else if (g_holdover)
        disp_cc = 7;
    else if (holdover_advert_ro(NULL))
        disp_cc = 7; /* settling, not converged, or currently degraded */
    else
        disp_cc = 6;

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "SERVO");

    /* Right-align "cc:NNN" on the title row. This font (u8g2_font_6x10_tf) is
     * a fixed 6px/glyph, so position arithmetically — consistent with the rest
     * of this file, which hard-codes pixel offsets rather than measuring. */
    char ccbuf[12];
    int cc_len = snprintf(ccbuf, sizeof(ccbuf), "cc:%u", (unsigned)disp_cc);
    int cc_x = 128 - (cc_len * 6);
    if (cc_x < 0)
        cc_x = 0;
    u8g2_DrawStr(&u8g2, cc_x, 10, ccbuf);

    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    /* Lock duration / holdover age */
    if (state == LED_STATE_LOCKED && lock_start > 0)
    {
        int64_t dur_s = (now_ms - lock_start) / 1000;
        if (dur_s < 60)
            snprintf(buf, sizeof(buf), "Lock: %llds", (long long)dur_s);
        else if (dur_s < 3600)
            snprintf(buf, sizeof(buf), "Lock: %lldm%llds",
                     (long long)(dur_s / 60), (long long)(dur_s % 60));
        else
            snprintf(buf, sizeof(buf), "Lock: %lldh%lldm",
                     (long long)(dur_s / 3600), (long long)((dur_s % 3600) / 60));
    }
    else if (state == LED_STATE_HOLDOVER)
    {
        time_t age = time(NULL) - g_last_gnss_update;
        snprintf(buf, sizeof(buf), "Holdover: %llds", (long long)age);
    }
    else
    {
        snprintf(buf, sizeof(buf), "(not locked)");
    }
    u8g2_DrawStr(&u8g2, 0, 22, buf);

    /* Line: quality + recent raw spread + lifetime reject total — OR, when a
     * streak is active, a prominent reject-streak warning.
     *
     * When the servo is rejecting samples (raw offset >100µs from the running
     * average), the streak is non-zero. A persistent non-zero streak is the
     * early signature of the stale-reference wedge: the averager has frozen
     * and every sample is being thrown away. servo_task forces a full
     * re-acquire at REJECT_STREAK_REACQUIRE. Surfacing the live count here
     * means you see "REJ n/45" climbing on the OLED *before* it trips, instead
     * of discovering a frozen Adj/Offset hours later.
     *
     * The streak takes priority over the quality readout because if we're
     * rejecting, the Q/peak numbers are stale (the averager isn't updating).
     *
     * When NOT actively rejecting, the quality line carries a persistent
     * lifetime reject total "R:n" (count since boot, never reset on re-acquire).
     * R:0 means the board has not rejected a single sample since power-up; any
     * non-zero value tells you — at a glance, after an unattended period — that
     * a reject event occurred and self-healed, even though the streak is back
     * to zero now. */
    if (rej_streak > 0)
    {
        snprintf(buf, sizeof(buf), "REJ %lu/%d !",
                 (unsigned long)rej_streak, REJECT_STREAK_REACQUIRE);
    }
    else if (state == LED_STATE_LOCKED && g_lock_qual_count > 0)
    {
        int good = 0;
        for (int i = 0; i < g_lock_qual_count; i++)
            good += g_lock_qual_ring[i];
        int pct = (good * 100) / g_lock_qual_count;

        int64_t span_us = 0;
        if (g_qual_raw_min != INT64_MAX && g_qual_raw_max != INT64_MIN)
        {
            int64_t a = llabs(g_qual_raw_min);
            int64_t b = llabs(g_qual_raw_max);
            span_us = ((a > b) ? a : b) / 1000;
        }
        /* Cap displayed peak at 3 digits (>=999u shows "999+") and bound the
         * reject total to a fixed-width field so the line never exceeds the
         * ~21-char OLED width: <10000 shown as-is, larger shown as "Nk", and
         * anything implausibly large clamped to "99k+". */
        char rstr[8];
        if (rej_total <= 9999)
            snprintf(rstr, sizeof(rstr), "%lu", (unsigned long)rej_total);
        else if (rej_total <= 99999)
            snprintf(rstr, sizeof(rstr), "%luk", (unsigned long)(rej_total / 1000));
        else
            snprintf(rstr, sizeof(rstr), "99k+");

        if (span_us > 999)
            snprintf(buf, sizeof(buf), "Q:%d%% pk999+ R:%s", pct, rstr);
        else
            snprintf(buf, sizeof(buf), "Q:%d%% pk%lldu R:%s",
                     pct, (long long)span_us, rstr);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Q: -- R:%lu", (unsigned long)rej_total);
    }
    u8g2_DrawStr(&u8g2, 0, 33, buf);

    /* Frequency adjustment */
    snprintf(buf, sizeof(buf), "Adj: %+d ppb", (int)freq_ppb);
    u8g2_DrawStr(&u8g2, 0, 44, buf);

    /* Integrator + PPS count, both compacted to <=6 chars to stay within
     * screen-2's 21-char budget (6px font). g_integral is clamped to +/-2e9,
     * so its magnitude fits in uint32 — same width the compiler can prove for
     * pps_count, which is what keeps -Wformat-truncation happy. */
    char istr[12], pstr[12];
    int64_t iv = (int64_t)integ;
    uint32_t im = (uint32_t)llabs(iv);
    char isign = (iv < 0) ? '-' : '+';
    if (im < 100000UL)
        snprintf(istr, sizeof(istr), "%c%lu", isign, (unsigned long)im);
    else if (im < 10000000UL)
        snprintf(istr, sizeof(istr), "%c%luk", isign, (unsigned long)(im / 1000UL));
    else
        snprintf(istr, sizeof(istr), "%c%luM", isign, (unsigned long)(im / 1000000UL));

    if (pps_count < 100000UL)
        snprintf(pstr, sizeof(pstr), "%lu", (unsigned long)pps_count);
    else if (pps_count < 10000000UL)
        snprintf(pstr, sizeof(pstr), "%luk", (unsigned long)(pps_count / 1000UL));
    else
        snprintf(pstr, sizeof(pstr), "%luM", (unsigned long)(pps_count / 1000000UL));

    snprintf(buf, sizeof(buf), "I:%s PPS:%s", istr, pstr);
    u8g2_DrawStr(&u8g2, 0, 55, buf);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Screen 3: GNSS signal quality — antenna/sky-view diagnosis
 *   - Verdict (EXCELLENT / GOOD / FAIR / WEAK / NOISY)
 *   - Average SNR with min/max range
 *   - Satellites in view vs used in fix
 *   - Fix quality hint
 *
 * Updated by parse_gsv / parse_gsa from NMEA sentences. Refresh rate is
 * driven by the receiver module (typically 1 Hz). If "No GSV data" appears,
 * the module isn't emitting GSV; check NMEA sentence configuration.
 *
 * Constellations covered: GPS, GLONASS, Galileo, BeiDou, multi-GNSS (GN),
 * plus a catch-all "other" slot for any additional talkers (see talker_index).
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_gnss(void)
{
    portENTER_CRITICAL(&g_gnss_sig_lock);
    uint8_t avg = g_gnss_snr_avg;
    uint8_t mn = g_gnss_snr_min;
    uint8_t mx = g_gnss_snr_max;
    uint8_t sv_view = g_gnss_sv_in_view;
    uint8_t sv_used = g_gnss_sv_used;
    int64_t last_ms = g_gnss_sig_last_update_ms;
    portEXIT_CRITICAL(&g_gnss_sig_lock);

    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t age_ms = now_ms - last_ms;

    char buf[32];

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "GNSS SIGNAL");
    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    if (last_ms == 0)
    {
        u8g2_DrawStr(&u8g2, 0, 28, "No GSV data");
        u8g2_DrawStr(&u8g2, 0, 40, "(check NMEA cfg)");
        return;
    }

    if (age_ms > 5000)
    {
        u8g2_DrawStr(&u8g2, 0, 28, "GSV stale");
        snprintf(buf, sizeof(buf), "Last: %llds ago", (long long)(age_ms / 1000));
        u8g2_DrawStr(&u8g2, 0, 40, buf);
        return;
    }

    /* Verdict describes signal *quality*, which maps roughly to expected
     * PPS jitter — not whether the device "works". A locked servo is
     * possible even with NOISY signal; the verdict just tells you whether
     * the antenna/sky-view is the limiting factor. */
    const char *verdict;
    if (avg >= 40)
        verdict = "EXCELLENT";
    else if (avg >= 35)
        verdict = "GOOD";
    else if (avg >= 30)
        verdict = "FAIR";
    else if (avg >= 25)
        verdict = "WEAK";
    else
        verdict = "NOISY";

    u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
    u8g2_DrawStr(&u8g2, 0, 26, verdict);

    /* Small subtitle clarifying what the verdict means */
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    u8g2_DrawStr(&u8g2, 0, 33, "(signal/antenna quality)");

    /* SNR detail */
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    snprintf(buf, sizeof(buf), "SNR:%u (%u-%u) dB", avg, mn, mx);
    u8g2_DrawStr(&u8g2, 0, 45, buf);

    /* SV counts */
    snprintf(buf, sizeof(buf), "SVs:%u view /%u used", sv_view, sv_used);
    u8g2_DrawStr(&u8g2, 0, 55, buf);

    /* Fix-quality hint */
    const char *fix_hint;
    if (sv_used >= 5)
        fix_hint = "Fix: solid";
    else if (sv_used == 4)
        fix_hint = "Fix: minimal 3D";
    else if (sv_used == 3)
        fix_hint = "Fix: 2D only";
    else if (sv_used > 0)
        fix_hint = "Fix: degraded";
    else
        fix_hint = "Fix: none";
    u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
    u8g2_DrawStr(&u8g2, 0, 63, fix_hint);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Screen 4: Network / system — connectivity and uptime
 *   - Link state (+ L2TAP-ready indicator)
 *   - IP address
 *   - Uptime
 *   - Free heap
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_network(void)
{
    bool link_up = atomic_load(&g_link_up);
    bool l2tap_ready = atomic_load(&g_l2tap_ready);
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t boot_ms = g_boot_time_ms;
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    char buf[32];

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "NETWORK");
    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    /* Row 1: link + IP together.  "UP 192.168.100.243" is <= 18 chars.
     * A '!' after UP flags link-up-but-L2TAP-not-ready.  Freed rows go
     * to the hostname below -- the identity this page exists to answer. */
    if (link_up && g_eth_netif)
    {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(g_eth_netif, &ip) == ESP_OK && ip.ip.addr != 0)
            snprintf(buf, sizeof(buf), "UP%s " IPSTR,
                     l2tap_ready ? "" : "!", IP2STR(&ip.ip));
        else
            snprintf(buf, sizeof(buf), "UP%s (no IP)", l2tap_ready ? "" : "!");
    }
    else
    {
        snprintf(buf, sizeof(buf), "DOWN (no link)");
    }
    u8g2_DrawStr(&u8g2, 0, 24, buf);

    /* Rows 2-3: hostname the DHCP server sees.  Reverse-lookup with
     * hostname_to_mac.py, or mac_hostname_to_mac() on-device. */
    u8g2_DrawStr(&u8g2, 0, 35, g_host_line1[0] ? g_host_line1 : "(no hostname)");
    if (g_host_line2[0])
        u8g2_DrawStr(&u8g2, 0, 46, g_host_line2);

    /* Row 4: uptime + free heap combined.  "Up 12d03h H:412K". */
    int64_t up_s = (now_ms - boot_ms) / 1000;
    int days = (int)(up_s / 86400);
    int hrs = (int)((up_s % 86400) / 3600);
    int mins = (int)((up_s % 3600) / 60);
    if (days > 0)
        snprintf(buf, sizeof(buf), "Up %dd%02dh H:%uK",
                 days, hrs, (unsigned)(free_heap / 1024));
    else
        snprintf(buf, sizeof(buf), "Up %02d:%02d H:%uK",
                 hrs, mins, (unsigned)(free_heap / 1024));
    u8g2_DrawStr(&u8g2, 0, 57, buf);
}

/* ───────────────────────────────────────────────────────────────────────────────────
 * Screen 5: TAU1201 timing-mode configuration status
 *   - Whether the configurator is still running, succeeded, or failed
 *   - Which step failed (if any)
 *
 * Auto-jumped to on first failure (see display_task) so it cannot be missed.
 * ─────────────────────────────────────────────────────────────────────────────────── */
static void render_screen_timing_cfg(void)
{
    char buf[32];

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "TIMING CFG");
    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    const char *err = g_timing_error_step;
    bool done = initialization_sequence_done;
    bool ok = g_timing_config_ok;

    if (err != NULL)
    {
        u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
        u8g2_DrawStr(&u8g2, 0, 30, "FAILED");

        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        snprintf(buf, sizeof(buf), "Step: %s", err);
        u8g2_DrawStr(&u8g2, 0, 46, buf);
        u8g2_DrawStr(&u8g2, 0, 58, "(see log)");
    }
    else if (ok && done)
    {
        u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
        u8g2_DrawStr(&u8g2, 0, 30, "READY");
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        u8g2_DrawStr(&u8g2, 0, 46, "Survey complete");
        u8g2_DrawStr(&u8g2, 0, 58, "Module in RAM mode");
    }
    else
    {
        u8g2_SetFont(&u8g2, u8g2_font_7x14B_tf);
        u8g2_DrawStr(&u8g2, 0, 30, "RUNNING");
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        u8g2_DrawStr(&u8g2, 0, 46, "Configuring TAU1201");

        /* Format the configured survey duration sensibly: minutes if it's a
         * whole number of minutes, otherwise mm:ss. */
        unsigned secs = ALLYSTAR_SURVEY_MAX_SECONDS;
        if (secs % 60 == 0)
        {
            snprintf(buf, sizeof(buf), "Survey ~%u min", secs / 60);
        }
        else if (secs >= 60)
        {
            snprintf(buf, sizeof(buf), "Survey ~%u:%02u", secs / 60, secs % 60);
        }
        else
        {
            snprintf(buf, sizeof(buf), "Survey ~%u sec", secs);
        }
        u8g2_DrawStr(&u8g2, 0, 58, buf);
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Network initialization
 *
 * Ethernet/IP bring-up, DHCP-with-static-fallback, L2TAP setup, and the
 * link up/down handlers. Since the timing-core decoupling, on_link_up only
 * handles link/PTP-serving concerns (multicast filter refresh, atomics,
 * LED update) — the one-shot L2TAP + PHC start is done at init in
 * timing_core_bringup() and does NOT depend on a link.
 * ═════════════════════════════════════════════════════════════════════════ */

static void ip_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    if (id == IP_EVENT_ETH_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        const char *source = g_static_applied ? "Static" : "DHCP";
        ESP_LOGI("IP", "%s got IP: " IPSTR ", GW: " IPSTR ", Mask: " IPSTR,
                 source,
                 IP2STR(&event->ip_info.ip),
                 IP2STR(&event->ip_info.gw),
                 IP2STR(&event->ip_info.netmask));
        if (g_got_ip_sem && !g_static_applied)
            xSemaphoreGive(g_got_ip_sem);
    }
    else if (id == IP_EVENT_ETH_LOST_IP)
    {
        ESP_LOGW("IP", "Lost IP");
    }
}

static void apply_static_ip(esp_netif_t *netif)
{
    ESP_LOGW("IP", "DHCP timeout — applying static IP fallback");
    g_static_applied = true; /* mark before triggering event */

    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
    {
        ESP_LOGW("IP", "dhcpc_stop returned %s", esp_err_to_name(err));
    }

    esp_netif_ip_info_t ip = {
        .ip.addr = STATIC_IP_ADDR,
        .netmask.addr = STATIC_IP_NETMASK,
        .gw.addr = STATIC_IP_GATEWAY,
    };

    err = esp_netif_set_ip_info(netif, &ip);
    if (err != ESP_OK)
    {
        ESP_LOGE("IP", "set_ip_info failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI("IP", "Static IP applied: " IPSTR " / " IPSTR " GW " IPSTR,
             IP2STR(&ip.ip), IP2STR(&ip.netmask), IP2STR(&ip.gw));
}

/* ─────────────────────────────────────────────────────────────────────────
 * Initialization
 * ───────────────────────────────────────────────────────────────────────── */
static void dhcp_fallback_task(void *arg)
{
    /* Runs for the lifetime of the device. On EVERY link-up edge it retries
     * DHCP from scratch and only falls back to the static IP if DHCP does not
     * answer within the timeout THIS cycle. This fixes the latch bug where a
     * unit that fell back to static (because no DHCP server was present at
     * first boot) kept the static IP forever, even after being replugged into
     * a network that does have DHCP.
     *
     * Per cycle:
     *   0. Arm for the cycle: clear the static latch, drain stale GOT_IP.
     *   1. Wait for the link to be up.
     *   2. (Re)start the DHCP client. apply_static_ip() stops it on fallback,
     *      so it must be restarted explicitly here, not just on first boot.
     *   3. Wait up to DHCP_FALLBACK_TIMEOUT_MS for IP_EVENT_ETH_GOT_IP.
     *   4. On timeout, apply the static fallback for this cycle.
     *   5. Block until the link drops, then loop to handle the next plug-in.
     */
    for (;;)
    {
        /* Prepare for the next link cycle BEFORE the link can come up, so the
         * GOT_IP handler is fully armed before any IP for this cycle arrives:
         *   - clear the static-applied latch so ip_event_handler will signal
         *     GOT_IP again (it suppresses the give while g_static_applied);
         *   - drain any stale GOT_IP give from a *previous* cycle. Draining here
         *     (not after link-up) can't discard a current-cycle give, because a
         *     current-cycle GOT_IP can only arrive after the link is up. */
        g_static_applied = false;
        xSemaphoreTake(g_got_ip_sem, 0);

        /* 1. Wait for link up */
        while (!atomic_load(&g_link_up))
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 2. (Re)start DHCP. If it was never stopped this returns ALREADY_STARTED,
         *    which is fine. If a previous cycle fell back to static, this is what
         *    actually re-enables DHCP — the whole point of the fix. */
        esp_err_t derr = esp_netif_dhcpc_start(g_eth_netif);
        if (derr != ESP_OK && derr != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED)
        {
            ESP_LOGW("IP", "dhcpc_start returned %s", esp_err_to_name(derr));
        }
        else
        {
            ESP_LOGI("IP", "Link up — requesting IP via DHCP");
        }

        /* 3. Wait for DHCP, 4. fall back to static if it doesn't reply */
        if (xSemaphoreTake(g_got_ip_sem,
                           pdMS_TO_TICKS(DHCP_FALLBACK_TIMEOUT_MS)) == pdTRUE)
        {
            ESP_LOGI("IP", "Using DHCP-assigned IP");
        }
        else
        {
            apply_static_ip(g_eth_netif);
        }

        /* 5. Block until the link drops — zero CPU between link events (no
         *    polling). on_link_down gives g_link_down_sem. If the link already
         *    dropped during the DHCP wait above, a give is pending and this
         *    returns immediately. A spurious/early wake is harmless: we loop and
         *    step 1 re-checks g_link_up before doing anything. */
        xSemaphoreTake(g_link_down_sem, portMAX_DELAY);
        ESP_LOGI("IP", "Link down — will retry DHCP on next link-up");
    }
}

/* Bring up the timing core. Depends ONLY on the EMAC driver being started
 * (esp_eth_start), not on a link/peer. Called once from app_main after
 * esp_eth_start and before the timing tasks are launched.
 *
 * Steps, all link-independent:
 *   1. Read the MAC address and derive the PTP clock identity. ETH_CMD_G_MAC_ADDR
 *      works as soon as the driver is started; no peer required. Doing it here
 *      means the BMCA dataset is correct from the first servo tick, not only
 *      after first link-up.
 *   2. Initialize + bind L2TAP to the driver handle. Binding (and timestamp
 *      enable) only needs the driver; frames simply won't arrive until a link
 *      exists, which is fine.
 *   3. Start the PHC (PTP clock). phc_start_blocking only toggles PTP_ENABLE and
 *      reads the PHC counter — no link needed.
 *
 * Returns true on success. On failure the LED is set to FAULT and the timing
 * tasks remain parked on g_phc_ready (never set), which is the safe outcome.
 */
static bool timing_core_bringup(void)
{
    /* 1. MAC + clock identity (driver-only, no link) */
    esp_eth_ioctl(g_eth_hndl, ETH_CMD_G_MAC_ADDR, g_src_mac);
    memcpy(g_clock_id, g_src_mac, 3);
    g_clock_id[3] = 0xFF;
    g_clock_id[4] = 0xFE;
    memcpy(g_clock_id + 5, g_src_mac + 3, 3);
    ESP_LOGI("INIT", "Clock identity set, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             g_src_mac[0], g_src_mac[1], g_src_mac[2],
             g_src_mac[3], g_src_mac[4], g_src_mac[5]);

    /* 2. L2TAP init + bind (driver-only, no link) */
    if (l2tap_init() != ESP_OK)
    {
        ESP_LOGE("INIT", "L2TAP init failed");
        led_set_state(LED_STATE_FAULT);
        return false;
    }
    atomic_store(&g_l2tap_ready, true);

    /* 3. Start the PHC (driver-only, no link) */
    if (!phc_start_blocking())
    {
        ESP_LOGE("INIT", "PHC start failed");
        led_set_state(LED_STATE_FAULT);
        return false;
    }

    /* Timing core is live: servo + pps may now run and discipline to GNSS,
     * independent of any peer. */
    atomic_store(&g_phc_ready, true);
    ESP_LOGI("INIT", "Timing core ready (PHC running, L2TAP bound) — "
                     "GNSS disciplining may begin without a link");
    return true;
}

/* Called from eth_event_handler on every link-up event.
 * Idempotent — safe to call multiple times.
 *
 * This is now strictly about LINK / PTP-serving concerns. The one-shot timing
 * core bring-up (L2TAP + PHC) has moved to timing_core_bringup(), run at init.
 * A link going up no longer has any bearing on whether the clock disciplines
 * to GNSS — only on whether PTP packets flow. */
static void on_link_up(void)
{
    /* Refresh the PTP multicast filter on every link-up. The driver de-dupes
     * internally, so this is idempotent. (MAC/clock_id were already set at init
     * in timing_core_bringup and don't change across link cycles on this HW.) */
    ESP_LOGI("ETH", "Link is UP, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             g_src_mac[0], g_src_mac[1], g_src_mac[2],
             g_src_mac[3], g_src_mac[4], g_src_mac[5]);

    uint8_t ptp_mcast_all[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
    esp_err_t ferr = esp_eth_ioctl(g_eth_hndl, ETH_CMD_ADD_MAC_FILTER, ptp_mcast_all);
    if (ferr == ESP_OK)
        ESP_LOGI(TAG, "PTP multicast filter added");
    else
        ESP_LOGW(TAG, "PTP multicast filter add: %s", esp_err_to_name(ferr));

    atomic_store(&g_link_up, true);

    /* Only update LED if it was reflecting "no link" — don't overwrite a real
     * timing state (LOCKED, HOLDOVER, ACQUIRING) just because a peer connected.
     * Note: with the decoupling, the servo may already be LOCKED before any
     * link ever comes up, so this branch will usually NOT fire. */
    if (atomic_load(&s_led_state) == LED_STATE_NO_LINK)
    {
        led_set_state(LED_STATE_ACQUIRING);
    }

    xSemaphoreGive(g_eth_up_sem); /* unblocks any one-shot waiters */
}

static void on_link_down(void)
{
    ESP_LOGW("ETH", "Link is DOWN");
    atomic_store(&g_link_up, false);
    /* Wake dhcp_fallback_task so it can re-arm DHCP on the next link-up.
     * Binary semaphore: if the task hasn't consumed a prior give yet, this is
     * a harmless no-op (give on an already-full binary sem just fails). */
    if (g_link_down_sem)
        xSemaphoreGive(g_link_down_sem);
    /* Don't touch LED state — Ethernet link != GNSS lock.
     * If servo was LOCKED, it stays LOCKED. The OLED's screen 3 reflects
     * link state separately. */
}

static void eth_event_handler(void *arg, esp_event_base_t base,
                              int32_t id, void *data)
{
    switch (id)
    {
    case ETHERNET_EVENT_CONNECTED:
        on_link_up();
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        on_link_down();
        break;
    default:
        break;
    }
}

static esp_err_t ethernet_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                               ip_event_handler, NULL));

    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num = ETH_MDIO_GPIO;

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_stack_size = 4096; // increase from default 2048
    // Note: descriptor counts are typically Kconfig-only

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &g_eth_hndl));

    bool ptp_en = true;
    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &ptp_en);

    /* Netif glue is required so L2TAP RX path is wired up. */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    g_eth_netif = esp_netif_new(&netif_cfg);
    esp_netif_attach(g_eth_netif, esp_eth_new_netif_glue(g_eth_hndl));
    /* DHCP client is started automatically by ESP_NETIF_DEFAULT_ETH. */

    /* Derive the hostname string and OLED lines from the eFuse MAC now
     * (pure CPU work, no netif state needed).  The actual set_hostname
     * call has to wait until after esp_eth_start() in app_main -- see
     * mac_hostname_apply() there.  Deriving here means the OLED boot
     * splash can self-identify from the very first frame. */
    esp_err_t hostname_err =
        mac_hostname_get(g_hostname,
                         sizeof g_hostname,
                         MAC_HOSTNAME_DASHED);

    if (hostname_err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Failed to derive hostname from MAC: %s",
                 esp_err_to_name(hostname_err));
        return hostname_err;
    }

    if (mac_hostname_display_lines(g_hostname,
                                   g_host_line1,
                                   sizeof g_host_line1,
                                   g_host_line2,
                                   sizeof g_host_line2) != 0)
    {
        ESP_LOGE(TAG,
                 "Failed to format hostname for OLED: %s",
                 g_hostname);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Device hostname: %s", g_hostname);
    ESP_LOGI(TAG, "OLED hostname line 1: %s", g_host_line1);
    ESP_LOGI(TAG, "OLED hostname line 2: %s", g_host_line2);

    return ESP_OK;
}

static esp_err_t l2tap_init(void)
{
    ESP_ERROR_CHECK(esp_vfs_l2tap_intf_register(NULL));

    g_l2tap_fd = open("/dev/net/tap", O_RDWR);
    if (g_l2tap_fd < 0)
    {
        ESP_LOGE(TAG, "Failed to open L2TAP: %s", strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "L2TAP fd=%d", g_l2tap_fd);

    // Set non-blocking via fcntl
    int flags = fcntl(g_l2tap_fd, F_GETFL, 0);
    fcntl(g_l2tap_fd, F_SETFL, flags | O_NONBLOCK);

    if (ioctl(g_l2tap_fd, L2TAP_S_DEVICE_DRV_HNDL, g_eth_hndl) < 0)
    {
        ESP_LOGE(TAG, "L2TAP_S_DEVICE_DRV_HNDL failed: %s", strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Bound to eth handle: %p", g_eth_hndl);

    bool ts_en = true;
    if (ioctl(g_l2tap_fd, L2TAP_S_TIMESTAMP_EN, &ts_en) < 0)
    {
        ESP_LOGW(TAG, "L2TAP_S_TIMESTAMP_EN failed: %s", strerror(errno));
    }
    else
    {
        ESP_LOGI(TAG, "Timestamp EN: OK");
    }

    uint16_t eth_type = ETH_TYPE_PTP;
    if (ioctl(g_l2tap_fd, L2TAP_S_RCV_FILTER, &eth_type) < 0)
    {
        ESP_LOGE(TAG, "L2TAP_S_RCV_FILTER failed: %s", strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "RCV filter set to 0x%04X", eth_type);

    return ESP_OK;
}

static bool phc_start_blocking(void)
{
    for (int i = 0; i < 5; i++)
    {
        ESP_LOGW("INIT", "PHC start attempt %d", i);

        bool en = false;
        esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &en);
        esp_rom_delay_us(10);

        en = true;
        esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_PTP_ENABLE, &en);
        vTaskDelay(pdMS_TO_TICKS(20));

        struct timespec t1, t2;
        emac_get_time(&t1);
        vTaskDelay(pdMS_TO_TICKS(50));
        emac_get_time(&t2);

        int64_t delta =
            ((int64_t)t2.tv_sec * 1000000000LL + t2.tv_nsec) -
            ((int64_t)t1.tv_sec * 1000000000LL + t1.tv_nsec);

        if (delta > 1000000)
        {
            ESP_LOGI("INIT", "PHC started (delta=%lld ns)", (long long)delta);
            // REMOVED: don't set time here. Servo will COARSE STEP it.
            return true;
        }

        ESP_LOGW("INIT", "PHC start retry (%d), delta=%lld", i, (long long)delta);
    }

    ESP_LOGE("INIT", "PHC FAILED TO START after retries");
    return false;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Hardware initialization
 *
 * GNSS UART + PPS GPIO setup, and the TAU1201 timing-mode configuration
 * error callback (surfaced on OLED screen 5).
 * ═════════════════════════════════════════════════════════════════════════ */

/* -----------------------------------------------------------------------
 * GNSS Grandmaster Code
 * ----------------------------------------------------------------------- */
static void gnss_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = GNSS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(GNSS_UART_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config(GNSS_UART_NUM, &cfg);
    uart_set_pin(GNSS_UART_NUM, GNSS_TX_PIN, GNSS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void pps_init(void)
{
    /* The PPS pin is edge-triggered. If the GNSS module is absent or the
     * cable is disconnected, this pin floats and a POSEDGE interrupt will
     * fire continuously on ambient noise — a textbook interrupt storm. The
     * handler then ran against an uninitialized g_eth_hndl/g_pps_sem (we
     * were called before ethernet_init and before the semaphores exist, fixed on app_main now also),
     * which aborted and crashed the panic handler on whichever task happened
     * to be current. Anchor the pin LOW with the internal pull-down so a
     * disconnected/missing GNSS leaves the pin defined; PPS is idle-low /
     * active-high, so this matches the signal's normal resting state and
     * does not interfere with a real PPS output. */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PPS_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&io_conf);

    /* DELIBERATE: the ISR service is registered WITHOUT ESP_INTR_FLAG_IRAM.
     * pps_isr_handler is IRAM_ATTR, but it calls esp_eth_ioctl(), which is
     * flash-resident — flagging the interrupt IRAM would let it fire during
     * flash-cache-off windows (OTA erase/write) and crash on the first
     * flash-resident call. Without the flag the interrupt is simply
     * DEFERRED across those windows: during an OTA upload, the occasional
     * PPS capture lands milliseconds late and shows up as HARD_REJECT
     * streak entries on OLED screen 2 — expected, bounded, self-clearing.
     * Making the capture path truly IRAM-safe would mean bypassing
     * esp_eth_ioctl for raw PHC register reads, which is more fragile
     * across IDF upgrades than the ms-scale OTA-only artifact it removes. */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PPS_GPIO, pps_isr_handler, NULL);
}

static void timing_config_error_cb(const char *failed_step_name)
{
    ESP_LOGE("MAIN", "Timing-mode config failed at step: [%s]",
             failed_step_name ? failed_step_name : "?");
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: Boot-time static facts + config JSON
 *
 * Everything here is either a hardware constant (flash size/mode, heap
 * capability totals, CPU MHz) or a compile-time #define. Both classes are
 * read/formatted exactly ONCE, at boot, from app_main — never on the httpd
 * hot path (rule 1.2.2: no flash access at runtime). g_config_json holds the
 * pre-rendered "config":{...} JSON fragment; status_handler appends it
 * verbatim.
 * ═════════════════════════════════════════════════════════════════════════ */
typedef struct
{
    uint32_t cpu_mhz;
    uint32_t flash_bytes;
    char flash_mode[8]; /* e.g. "dio" */
    uint32_t flash_mhz;
    size_t heap_internal_total;
    long rtc_heap_total; /* -1 => emit null */
    long tcm_heap_total; /* -1 => emit null */
} boot_facts_t;
static boot_facts_t g_boot_facts;

static void boot_facts_collect(void)
{
    g_boot_facts.cpu_mhz = esp_rom_get_cpu_ticks_per_us();

    uint32_t flash_sz = 0;
    esp_flash_get_size(NULL, &flash_sz);
    g_boot_facts.flash_bytes = flash_sz;

    strlcpy(g_boot_facts.flash_mode, CONFIG_ESPTOOLPY_FLASHMODE,
           sizeof(g_boot_facts.flash_mode));
    g_boot_facts.flash_mhz = CONFIG_ESPTOOLPY_FLASHFREQ_VAL;

    g_boot_facts.heap_internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);

    /* Heap-MANAGED totals (what the allocator owns for this capability), not
     * physical region sizes — the dashboard's Memory-card labels say so. */
#ifdef MALLOC_CAP_RTCRAM
    {
        size_t t = heap_caps_get_total_size(MALLOC_CAP_RTCRAM);
        g_boot_facts.rtc_heap_total = (t > 0) ? (long)t : -1;
    }
#else
    g_boot_facts.rtc_heap_total = -1;
#endif

#ifdef MALLOC_CAP_TCM
    {
        size_t t = heap_caps_get_total_size(MALLOC_CAP_TCM);
        g_boot_facts.tcm_heap_total = (t > 0) ? (long)t : -1;
    }
#else
    g_boot_facts.tcm_heap_total = -1;
#endif
}

/* Pre-formatted "config":{...} JSON fragment, built once at boot. Every
 * value comes from the #define the code actually uses at its real call
 * site — never retyped. transport/delay_mechanism/two_step are fixed by
 * design in this firmware (L2 Annex F only, E2E only; every Sync is sent
 * with PTP_FLAG_TWO_STEP in send_sync_and_followup(), so two_step is always
 * true — there is no on/off macro to derive it from, just that literal
 * flag). Priorities are runtime state, not config — they're reported in
 * bmca.local instead (see web_append_bmca in web_server.h). */
static char g_config_json[1024];

static void config_json_build(void)
{
    snprintf(g_config_json, sizeof(g_config_json),
        "\"config\":{"
        "\"ptp\":{\"domain\":%d,\"transport\":\"L2\",\"delay_mechanism\":\"E2E\",\"two_step\":true,"
                 "\"sync_interval_ms\":%d,\"announce_interval_ms\":%d,"
                 "\"announce_timeout_s\":%d,\"bmca_stability_threshold\":%d},"
        "\"servo\":{\"acq_kp\":%g,\"acq_ki\":%g,"
                   "\"lock_kp\":%g,\"lock_ki\":%g,\"lock_kd\":%g,\"lock_kd_enabled\":%s,"
                   "\"avg_window\":%d,\"lock_qual_window\":%d,\"lock_qual_good_ns\":%d,"
                   "\"reject_reacquire_at\":%d,\"holdover_timeout_s\":%d,\"post_holdover_settle_ms\":%d},"
        "\"hw\":{\"pps_gpio\":%d,\"pps_led_gpio\":%d,\"pps_led_on_ms\":%d,\"button_gpio\":%d,"
               "\"gnss_uart\":%d,\"gnss_baud\":%d,\"display_screens\":%d},"
        "\"net\":{\"dhcp_fallback_timeout_ms\":%d,\"slaves_max\":%d,\"slave_timeout_ms\":%d},"
        "\"mon\":{\"heap_mon_interval_ms\":%d}"
        "}",
        (int)PTP_DOMAIN,
        (int)PTP_SYNC_INTERVAL_MS, (int)PTP_ANNOUNCE_INTERVAL_MS,
        (int)ANNOUNCE_TIMEOUT_SEC, (int)BMCA_STABILITY_THRESHOLD,
        (double)SERVO_KP, (double)SERVO_KI,
        (double)SERVO_KP_SLOW, (double)SERVO_KI_SLOW, (double)SERVO_KD_SLOW,
        SERVO_KD_ENABLE ? "true" : "false",
        (int)AVG_WINDOW, (int)LOCK_QUAL_WINDOW, (int)LOCK_QUAL_GOOD_NS,
        (int)REJECT_STREAK_REACQUIRE, (int)HOLDOVER_TIMEOUT_SEC, (int)POST_HOLDOVER_SETTLE_MS,
        (int)PPS_GPIO, (int)PPS_LED_GPIO, (int)PPS_LED_ON_MS, (int)DISPLAY_BUTTON_GPIO,
        (int)GNSS_UART_NUM, (int)GNSS_UART_BAUD, (int)DISPLAY_NUM_SCREENS,
        (int)DHCP_FALLBACK_TIMEOUT_MS, (int)SLAVE_TRACK_MAX, (int)SLAVE_TIMEOUT_MS,
        (int)HEAP_MON_INTERVAL_MS);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION: app_main
 *
 * Brings up the OLED, records boot time, launches the button/display tasks,
 * starts Ethernet + LED + PPS, initializes the GNSS UART and the TAU1201
 * timing-mode configurator, then launches all worker tasks (they self-gate
 * on g_phc_ready for the timing core, and on g_link_up / g_l2tap_ready for
 * PTP packet I/O) and finally the web server.
 *
 * NOTE: web_server.h is included here, immediately before app_main, exactly
 * as in the original — keep this placement.
 * ═════════════════════════════════════════════════════════════════════════ */
#include "web_server.h"

void app_main(void)
{
    // If your loggin is not working (nothing is being produced at serial output), look at sdkconfig.defaults to know why
    // and re-enable it, if desired
    esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("*", ESP_LOG_WARN);
    //  esp_log_level_set("PTP_SERVER", ESP_LOG_INFO);

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                          // u8x8_byte_sw_i2c,
                                          u8g2_esp32_i2c_byte_cb,
                                          u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
    // u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);

    ESP_LOGI(TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in
                             // sleep mode after this,

    /* Record boot time for uptime display */
    g_boot_time_ms = esp_timer_get_time() / 1000;

    /* Launch button + display tasks (low priority — never block timing-critical work) */
    task_create_tracked(button_task, "btn", BUTTON_TASK_STACK, NULL, BUTTON_TASK_PRIO, NULL);
    task_create_tracked(display_task, "display", DISPLAY_TASK_STACK, NULL, DISPLAY_TASK_PRIO, NULL);

    /* ─────────────────────────────────────────────────────────────────────
     * Order matters: every dependency of pps_isr_handler must exist BEFORE
     * pps_init() arms the GPIO interrupt. The handler does:
     *     emac_get_time(&ts)       → esp_eth_ioctl(g_eth_hndl, ...)
     *     xSemaphoreGiveFromISR(g_pps_sem, ...)
     * so g_pps_sem must be created and g_eth_hndl must be populated (i.e.
     * ethernet_init + esp_eth_start completed) before the first edge can
     * possibly fire. Otherwise a PPS pulse landing in this window — or a
     * floating PPS pin firing on noise when no GNSS module is attached —
     * dereferences NULL inside the ISR and panics the box at boot.
     * ───────────────────────────────────────────────────────────────────── */

    g_eth_up_sem = xSemaphoreCreateBinary();
    g_pps_sem = xSemaphoreCreateBinary();

    /* Serializes the shared L2TAP fd across delay_resp_task + ptp_tx_task.
     * Must exist before any l2tap_send_ptp call (the PTP tasks below). */
    g_l2tap_tx_mtx = xSemaphoreCreateMutex();

    /* DHCP-fallback machinery — created BEFORE esp_eth_start so IP_EVENT_
     * ETH_GOT_IP can never fire while g_got_ip_sem is still NULL (the
     * handler NULL-guards, but a silently dropped give would look like
     * "DHCP never answered" and push a perfectly good lease onto the
     * static fallback 30 s later). The task starts now too, so its
     * first-cycle drain runs before any link can possibly come up and a
     * genuinely early GOT_IP can't be mistaken for a stale one from a
     * previous cycle. Until link-up the task only polls g_link_up at
     * 500 ms — it touches no netif state before ethernet_init sets it. */
    g_got_ip_sem = xSemaphoreCreateBinary();
    g_link_down_sem = xSemaphoreCreateBinary();
    task_create_tracked(dhcp_fallback_task, "dhcp_fb", 3072, NULL, 2, NULL);

    /* Ethernet driver init + start. Will not block waiting for link.
     * Fatal driver/netif failures abort inside ethernet_init via
     * ESP_ERROR_CHECK; a non-OK return here means only the OPTIONAL
     * hostname derivation failed — the device remains a fully functional
     * grandmaster, it just appears under the default lwIP name (and
     * g_hostname stays empty, which every consumer already guards). */
    esp_err_t eth_init_err = ethernet_init();
    if (eth_init_err != ESP_OK)
        ESP_LOGW("MAIN",
                 "ethernet_init: hostname derivation failed (%s) — "
                 "continuing with default hostname",
                 esp_err_to_name(eth_init_err));
    ESP_ERROR_CHECK(esp_eth_start(g_eth_hndl));
    ESP_LOGI("ETH", "Ethernet started (link state will be reported on connect)");

    /* Install the hostname NOW -- after esp_eth_start() so lwIP has
     * fully wired the netif, and before the first DHCP DISCOVER so the
     * unit appears under its stable name in leases from the first
     * transaction.  The hostname persists on the netif for the whole
     * app lifetime -- surviving cable unplug/replug and DHCP restarts
     * without any per-event re-application.  Logging (success and the
     * specific CONFIG_LWIP_MAX_HOSTNAME_LEN hint on failure) is owned
     * by the component. */
    if (g_hostname[0])
        mac_hostname_apply(g_eth_netif, g_hostname);

    /* Now safe to arm the PPS GPIO interrupt: g_pps_sem and g_eth_hndl exist. */
    pps_init();

    /* Start the LED subsystem BEFORE timing_core_bringup, so that if bring-up
     * fails it can surface LED_STATE_FAULT to a running LED task. Initial state
     * is NO_LINK (no peer yet); the servo will move it to ACQUIRING/LOCKED as it
     * disciplines to GNSS, independent of link. */
    led_start();
    led_set_state(LED_STATE_NO_LINK);
    task_create_tracked(pps_led_task, "pps_led", PPS_LED_TASK_STACK, NULL, 1, NULL);

    /* Bring up the timing core NOW — driver is started, so the PHC and L2TAP
     * can come up without waiting for a peer/link. This is the architectural
     * decoupling: GNSS disciplining begins at power-on, and PTP serving begins
     * later, whenever a link appears, finding the clock already locked.
     *
     * On failure, timing_core_bringup sets LED_STATE_FAULT and returns false;
     * g_phc_ready stays false so the timing tasks remain safely parked. We do
     * NOT abort app_main — OLED/LED/web diagnostics still run so the failure is
     * visible. */
    if (!timing_core_bringup())
    {
        ESP_LOGE("MAIN", "Timing core bring-up FAILED — clock will not discipline");
    }

    /*
     * NOTE: Promiscuous mode REMOVED.
     *
     * The original code enabled promiscuous mode to debug Linux interop.
     * For normal operation, the EMAC accepts:
     *   - Frames addressed to its unicast MAC (e.g., DHCP responses)
     *   - Frames to known multicast groups (PTP MCAST 01:1B:19:00:00:00 is
     *     accepted via the multicast filter set up by L2TAP)
     *   - Broadcasts (DHCP, ARP)
     *
     * Disabling promiscuous mode roughly halves RX load on a busy network
     * and substantially reduces pressure on the EMAC RX descriptor pool —
     * which was the cause of the "no mem for receive buffer" errors after
     * extended uptime.
     *
     * If you ever need to re-enable for debugging, uncomment:
     *
     *   bool promisc = true;
     *   esp_eth_ioctl(g_eth_hndl, ETH_CMD_S_PROMISCUOUS, &promisc);
     *   g_promiscuous = true;
     *   ESP_LOGI("ETH", "Promiscuous mode: ON");
     */

    /* GNSS UART can come up immediately — independent of network */
    gnss_uart_init();
    uart_flush(GNSS_UART_NUM);

    /* Kick off the TAU1201 timing-mode configurator. Runs as its own task,
     * shares the GNSS UART RX stream via allystar_timing_feed_data() which is
     * called from gnss_task. Fails are reported via the callback and the
     * OLED auto-jumps to screen 5. */
    initialize_allystar_timing_service(timing_config_error_cb);

    /* (DHCP-fallback semaphores + task were created earlier, before
     * esp_eth_start — see the ordering comment there.) */

    /* Launch all tasks now. They self-gate on g_phc_ready (timing core:
     * servo, pps) and on g_link_up / g_l2tap_ready (PTP packet I/O). */

    /* Delay_Resp producer/consumer: create the queue and worker BEFORE the RX
     * task, so enqueue_delay_req always has a valid queue. Worker priority 6
     * sits below the RX receive loop (7) and at/above the Sync/Announce TX
     * task (5), so receiving and Sync cadence both win over Delay_Resp under
     * flood. */
    g_delay_resp_q = xQueueCreate(DELAY_RESP_Q_DEPTH, sizeof(delay_resp_job_t));
    task_create_tracked(delay_resp_task, "dresp", 4096, NULL, 6, NULL);

    task_create_tracked(ptp_rx_task, "ptp_rx", 8192, NULL, 7, NULL);
    task_create_tracked(ptp_tx_task, "ptp_tx", 8192, NULL, 5, NULL);

    /* Role-change logging, off the receive hot path — see bmca_set_role /
     * bmca_log_task. Low priority: it only drains a ring buffer every
     * 500 ms, never anything time-critical.
     *
     * 4096 bytes, not the 2048 the original instrumentation plan called for:
     * bench-tested on real hardware, a genuine role change (a second PTP
     * master on the LAN) triggers the event log's ESP_LOGI call — 8 args
     * incl. %lld/%s/%02X — and newlib's vfprintf on this toolchain needs
     * more headroom than that. Confirmed by a hardware stack-guard fault
     * (Core panic, "Stack protection fault") inside
     * xQueueTakeMutexRecursive/vfprintf with the 2048-byte size. Matches
     * every other task in this file that logs multi-argument lines
     * (heap_mon/dhcp_fb=3072, wdt=4096) — only the trivial single-%d
     * loggers (led, btn) get away with 2048. */
    task_create_tracked(bmca_log_task, "bmca_log", 4096, NULL, 1, NULL);

    task_create_tracked(gnss_task, "gnss", 6144, NULL, 5, NULL);
    task_create_tracked(gnss_signal_task, "gnss_sig", 3072, NULL, 1, NULL);
    task_create_tracked(servo_task, "servo", 6144, NULL, 6, &servo_task_handle);
    task_create_tracked(pps_task, "pps_task", 4096, NULL, 7, NULL);

    task_create_tracked(heap_monitor_task, "heap_mon", 3072, NULL, 1, NULL);

    task_create_tracked(watchdog_task, "wdt", 4096, NULL, 1, NULL);

    /* This task ("main") and the httpd task started below never go through
     * task_create_tracked — register their stack sizes directly so the
     * dashboard's task table doesn't show them with a null stack. */
    task_reg_add("main", CONFIG_ESP_MAIN_TASK_STACK_SIZE);

    ESP_LOGI("MAIN", "All tasks launched. Waiting for Ethernet link...");

    /* Boot-time facts (flash size/mode, heap capability totals, CPU MHz) and
     * the derived "config" JSON fragment: both are constant for the life of
     * the process, so collect/format them once here, before the httpd task
     * (which reads them on every /api/status request) starts. */
    boot_facts_collect();
    config_json_build();

    /* Optional: also kick off DHCP-fallback watcher so it runs in background
     * once link comes up. The IP event handler already handles this. */
    web_server_start();
}