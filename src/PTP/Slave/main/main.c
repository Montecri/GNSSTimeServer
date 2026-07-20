/*
 * ═════════════════════════════════════════════════════════════════════════
 * PTPv2 Slave for ESP32-P4
 * ═════════════════════════════════════════════════════════════════════════
 * Cristiano Reis Monteiro <cristianomonteiro@gmail.com> - May/2026
 * ═════════════════════════════════════════════════════════════════════════
 *
 * ─────────────────────────────────────────────────────────────────────────
 * Build target: PTPv2 SLAVE (slave-only firmware)
 * IEEE 1588-2008, Annex F (L2/Ethernet transport)
 * Target board: Guition JC-ESP32P4-M3-DEV board
 * ESP-IDF Version: 5.5.4
 *
 * PTP-only slave: locks onto a remote grandmaster via Sync / Follow_Up /
 * Delay_Req / Delay_Resp, disciplines the local PHC to the master's clock.
 * No GNSS, no PPS edge capture, no Sync/Announce transmission.
 * ─────────────────────────────────────────────────────────────────────────
 *
 * ─────────────────────────────────────────────────────────────────────────
 * How the code runs at a glance
 *
 * On boot, app_main captures start time, creates sync primitives, then:
 *   1. ethernet_init()      — brings up the Ethernet driver, derives the
 *                             device hostname from the MAC (mac_hostname
 *                             component) and installs it BEFORE the first
 *                             DHCPDISCOVER so the router lists us by name,
 *                             attaches netif glue for L2TAP RX
 *   2. timing_core_bringup()— captures MAC + clockIdentity, opens the L2TAP
 *                             socket, enables the ESP32-P4's PHC hardware
 *                             timestamping, sets g_phc_ready
 *   3. Task spawn           — see task list below
 *   4. web_server_start()   — /health, /api/status JSON, OTA endpoint
 *
 * The PTP data path (bottom-up):
 *   1. Wire RX  → ptp_rx_task decodes Ethernet frames arriving on L2TAP
 *   2. Handlers → capture T1/T2/T3/T4 timestamps and Announce properties;
 *                 BMCA (Best Master Clock Algorithm) picks the best
 *                 observed grandmaster and stability-filters GM identity
 *   3. Servo   → slave_compute_and_apply does the four-timestamp offset
 *                 math (offset = ((t2-t1) - (t4-t3)) / 2) and either steps
 *                 the PHC (large offset, pre-lock) or nudges its frequency
 *                 via a PI controller (small offset, in-lock)
 *   4. Consumers → OLED (4 screens), LED (state machine), web /api/status,
 *                 and periodic gm_diag console reports all read the outputs
 *                 of the servo + GM tracking
 *
 * Background tasks (all lower priority than PTP RX):
 *   ptp_rx_task            — L2TAP read loop; dispatches to handlers
 *   delay_req_task         — drains g_delay_req_q; keeps PHC ioctls off the
 *                            RX hot path
 *   gm_diag_task           — periodic GM quality/health summary to console
 *   slave_watchdog_task    — self-heals via emac_restart_preserve_phc() if
 *                            the PHC stops advancing
 *   slave_led_task         — drives the status LED pattern from s_led_state
 *   slave_led_watch_task   — transitions LED state on Sync freshness
 *                            (LOCKED / HOLDOVER / FAULT)
 *   pps_led_task           — pulses a second LED on every Sync RX (heartbeat)
 *   display_task           — 4-screen OLED render loop at ~2 Hz
 *   button_task            — debounced screen-cycle button
 *   dhcp_fallback_task     — long-lived; applies static IP if DHCP times out
 *
 * OLED screens (button cycles):
 *   1. Main         — lock state, offset (auto-scaled ns/µs), path delay,
 *                     sync-rate label (derived from GM's logSyncInterval)
 *   2. Servo        — lock duration, offset/delay mean±sd, Q100/Q1k quality,
 *                     freq-adjust ppb + linreg drift slope, asymmetry
 *   3. Network      — link + IP + L2TAP-ready flag, hostname (2 rows),
 *                     uptime + free heap
 *   4. Grandmaster  — full GM MAC + stepsRemoved, BMCA fields
 *                     (priority1/2, clockClass, accuracy), time source +
 *                     UTC offset, Sync freshness + delay-path health + !N
 * ─────────────────────────────────────────────────────────────────────────
 *
 * ─────────────────────────────────────────────────────────────────────────
 * File layout (17 sections)
 *
 *   Section 1  — Includes
 *   Section 2  — Configuration #defines (grouped by subsystem)
 *   Section 3  — Type definitions (PTP wire structs, state enums, GM diag)
 *   Section 4  — Global state (grouped by owning subsystem)
 *   Section 5  — Forward declarations (single block so any function may
 *                call any other regardless of emission order)
 *   Section 6  — Small helpers (pure functions, math utilities, PHC read)
 *   Section 7  — EMAC PHC + PTP wire helpers (timestamp encode/decode,
 *                header build, L2TAP send with HW TX timestamping)
 *   Section 8  — BMCA (compares Announces, picks best observed GM)
 *   Section 9  — PTP message handlers (Sync, FollowUp, DelayResp, Announce
 *                + the Delay_Req producer/consumer pair)
 *   Section 10 — Servo + lock management (PI controller, step vs freq
 *                decision, reject streak → reacquire, ever-locked latch)
 *   Section 11 — Background tasks (ptp_rx_task, delay_req_task, gm_diag)
 *   Section 12 — Health monitoring (heap watchdog + EMAC restart preserving
 *                the PHC clock)
 *   Section 13 — LED status indicator (state machine → pattern) + PPS LED
 *   Section 14 — OLED display + 4-screen renderer + debounced button
 *   Section 15 — Network initialization (Ethernet driver, DHCP with static
 *                fallback, MAC-derived hostname, link up/down handling)
 *   Section 16 — Hardware initialization (PHC enable/verify, timing_core
 *                bringup called once from app_main)
 *   Section 17 — app_main (entry point)
 * ─────────────────────────────────────────────────────────────────────────
 */

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 1: Includes
 * ═════════════════════════════════════════════════════════════════════════ */

#include <u8g2.h>
#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include <string.h>
#include <driver/spi_master.h>
#include <stdatomic.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include <inttypes.h>
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "mac_hostname.h"
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

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_err.h"
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

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 2: Configuration #defines
 *
 * Compile-time constants, grouped by intended use. Anything that
 * belongs in a subsystem-local #define (values that only matter to
 * one function — servo clamps, etc.) lives near that code — see the
 * top of Section 10 for SLAVE_FREQ_LIMIT_PPB, for example.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────
 * ESP-IDF version compatibility shims
 *
 * Fills in symbols that older IDF headers may not define. Keep at the
 * very top so subsequent code can rely on them.
 * ───────────────────────────────────────────────────────────────────── */
#define L2TAP_IREC_TYPE_TX_CONFIRM 3

/* ─────────────────────────────────────────────────────────────────────
 * Hardware pin assignments — Ethernet (EMAC + RMII PHY)
 *
 * GPIO numbers on the Guition JC-ESP32P4-M3-DEV board. Change these
 * if you retarget to a board with a different PHY wiring.
 * ───────────────────────────────────────────────────────────────────── */
#define ETH_PHY_ADDR 1
#define ETH_PHY_RST_GPIO 51
#define ETH_MDC_GPIO 31
#define ETH_MDIO_GPIO 52

/* ─────────────────────────────────────────────────────────────────────
 * Hardware pin assignments — OLED display (I2C)
 *
 * SH1106 128×64 OLED wiring on the ESP32-P4's I2C bus.
 * ───────────────────────────────────────────────────────────────────── */
#define PIN_SDA 7
#define PIN_SCL 8

/* ─────────────────────────────────────────────────────────────────────
 * Hardware pin assignments — user LEDs and button
 *
 * LED_GPIO: primary status LED (state machine driven).
 * PPS_LED_GPIO: heartbeat LED, pulses on every Sync RX.
 * DISPLAY_BUTTON_GPIO: cycles OLED screens.
 * ───────────────────────────────────────────────────────────────────── */
#define LED_GPIO 2                  // <-- set to your LED pin
#define LED_ACTIVE_HIGH 1           // 0 if LED is wired active-low
#define PPS_LED_GPIO 3              // <-- set to your second LED pin
#define PPS_LED_ACTIVE_HIGH 1       // 0 if active-low
#define DISPLAY_BUTTON_GPIO 1       // <-- set to your button pin
#define DISPLAY_BUTTON_ACTIVE_LOW 1 // 1 if button shorts to GND with pull-up

/* ─────────────────────────────────────────────────────────────────────
 * Hardware pin assignments — legacy GNSS/PPS (unused on this slave)
 *
 * Vestigial. The slave firmware does NOT read GNSS or discipline off
 * a PPS pulse; it takes time from PTP over Ethernet. These defines
 * linger for structural parity with the master firmware.
 * ───────────────────────────────────────────────────────────────────── */
#define GNSS_UART_NUM UART_NUM_1
#define GNSS_TX_PIN 33
#define GNSS_RX_PIN 32
#define PPS_GPIO 20

/* ─────────────────────────────────────────────────────────────────────
 * PTP protocol constants (wire format)
 *
 * IEEE 1588-2008 message types, flags, EtherType, domain. Do not
 * change without a corresponding wire-format update in the handlers.
 * ───────────────────────────────────────────────────────────────────── */
#define ETH_TYPE_PTP 0x88F7
#define PTP_DOMAIN 0
#define PTP_MSG_SYNC 0x0
#define PTP_MSG_DELAY_REQ 0x1
#define PTP_MSG_FOLLOW_UP 0x8
#define PTP_MSG_DELAY_RESP 0x9
#define PTP_MSG_ANNOUNCE 0xB
#define PTP_FLAG_TWO_STEP 0x0200

// TimePropertiesDS flags (IEEE 1588-2008)
#define PTP_FLAG_LEAP61 (1 << 0)
#define PTP_FLAG_LEAP59 (1 << 1)
#define PTP_FLAG_UTC_OFFSET_VALID (1 << 2)
#define PTP_FLAG_PTP_TIMESCALE (1 << 3)
#define PTP_FLAG_TIME_TRACEABLE (1 << 4)
#define PTP_FLAG_FREQ_TRACEABLE (1 << 5)

/* ─────────────────────────────────────────────────────────────────────
 * PTP timing intervals
 *
 * Rates the slave assumes for master traffic. Actual received rates
 * come from the wire (logSyncInterval / logAnnounceInterval fields
 * in the PTP header) — these are only used for local timeouts.
 * ───────────────────────────────────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────────
 * Configuration
 * ───────────────────────────────────────────────────────────────────────── */
#define PTP_SYNC_INTERVAL_MS 1000
#define PTP_ANNOUNCE_INTERVAL_MS 2000

/* ─────────────────────────────────────────────────────────────────────
 * Networking — DHCP and static-IP fallback
 *
 * DHCP is attempted first; if it doesn't respond within the timeout,
 * ethernet_init applies the static IP so the device is still reachable.
 * ───────────────────────────────────────────────────────────────────── */
#define DHCP_FALLBACK_TIMEOUT_MS 15000 /* wait up to 15 s for DHCP */

/* Fallback static IP — used if DHCP doesn't reply in time.
 * Pick a private subnet you don't use elsewhere. */
#define STATIC_IP_ADDR ESP_IP4TOADDR(192, 168, 5, 2)
#define STATIC_IP_NETMASK ESP_IP4TOADDR(255, 255, 255, 0)
#define STATIC_IP_GATEWAY ESP_IP4TOADDR(192, 168, 5, 1)

/* ─────────────────────────────────────────────────────────────────────
 * Servo tuning — vestigial master-side PI gains
 *
 * Kept for reference / historical diff parity with the master
 * firmware. The slave uses the SLAVE_SERVO_* pair below.
 * ───────────────────────────────────────────────────────────────────── */

/* At top of file: */
#define SERVO_KP 0.4 // was 0.7
#define SERVO_KI 0.3

/* ─────────────────────────────────────────────────────────────────────
 * Servo tuning — slave PI gains (the ones actually used)
 *
 * PTP slave runs at ~1 Hz update rate. KP small enough to avoid
 * ringing on a 1 Hz loop; KI large enough to track crystal drift
 * in a few samples.
 * ───────────────────────────────────────────────────────────────────── */

/* Servo gains. PTP slave runs at ~1 Hz update rate.
 *   KP small enough to avoid ringing on a 1 Hz loop.
 *   KI large enough to track crystal drift in a few samples. */
#define SLAVE_SERVO_KP 0.7
#define SLAVE_SERVO_KI 0.3

/* ─────────────────────────────────────────────────────────────────────
 * Servo — lock detection thresholds
 *
 * SLAVE_LOCK_* / SLAVE_UNLOCK_* are the active thresholds;
 * LOCK_* and MAX_STEP_NS are vestigial from the master pattern.
 * ───────────────────────────────────────────────────────────────────── */

/* Slave lock tracking — analogous to master servo state machine */
#define SLAVE_LOCK_OFFSET_NS 10000    /* 10 µs — declare "good" sample */
#define SLAVE_LOCK_THRESHOLD 5        /* consecutive good samples to lock */
#define SLAVE_UNLOCK_OFFSET_NS 100000 /* 100 µs — drop out of lock */
#define SLAVE_REJECT_STREAK_REACQUIRE 45
#define SLAVE_ANNOUNCE_TIMEOUT_S 10 /* ~5 Announce intervals at logInterval 1 */
#define LOCK_THRESHOLD 5            // consecutive good samples
#define LOCK_OFFSET_NS 10000000     // 1 ms to declare "good"
#define MAX_STEP_NS 2000000000LL    // only allow huge steps pre-lock

/* ─────────────────────────────────────────────────────────────────────
 * Filter windows
 *
 * MEDIAN_WINDOW: 5-sample sliding median (PPS-derived offset).
 * LINREG_WINDOW: linear regression window over recent (t, offset) samples.
 * ASYMMETRY_WINDOW: tracks the two path half-trips separately.
 * ───────────────────────────────────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────────
 * Median filter (5-sample sliding window) for PPS-derived offset
 * ───────────────────────────────────────────────────────────────────────── */
#define MEDIAN_WINDOW 5

/* Linear regression filter — sliding window of (time, offset) pairs */
#define LINREG_WINDOW 16 /* 16-sample window = 16 seconds of history */

/* Asymmetry tracking — track the two half-trips separately.
 *
 * IMPORTANT: PTP cannot observe true link asymmetry from a single exchange
 * (the standard PTP offset equation absorbs any constant asymmetry into the
 * computed offset). What we CAN show is `ms_to_sl` and `sl_to_ms`
 * separately, where:
 *
 *    ms_to_sl = t2 - t1   (master TX → slave RX, ns of slave-PHC view)
 *    sl_to_ms = t4 - t3   (slave TX → master RX, ns of slave-PHC view)
 *
 * If the slave is locked, the means of these two values represent the
 * one-way delay in each direction (plus a constant equal to true offset).
 * If they differ noticeably (i.e. their means differ by more than the
 * delay stddev), the link is asymmetric. The bias on PTP offset is half
 * of the difference between the means. */
#define ASYMMETRY_WINDOW 64

/* ─────────────────────────────────────────────────────────────────────
 * State-transition timeouts
 *
 * HOLDOVER_TIMEOUT_SEC and ANNOUNCE_TIMEOUT_SEC gate Announce-based
 * state; DRIFTING_TO_FAULT_MS gates the red LED's HOLDOVER → FAULT
 * drop; BMCA_STABILITY_THRESHOLD suppresses BMCA flapping on
 * borderline Announce advertisements.
 * ───────────────────────────────────────────────────────────────────── */
#define HOLDOVER_TIMEOUT_SEC 5
#define ANNOUNCE_TIMEOUT_SEC 6

/* Drifting → fault transition: how long we tolerate "no master" before
 * giving up and going to FAULT. Set to a few times the announce interval
 * so a transient blip doesn't cause an alarm. */
#define DRIFTING_TO_FAULT_MS 60000
#define BMCA_STABILITY_THRESHOLD 3

/* ─────────────────────────────────────────────────────────────────────
 * Task priorities, stack sizes, and tick rates
 *
 * PTP RX task runs at PTP_TASK_PRIORITY. LED / display / button tasks
 * are all low priority — they never block anything meaningful.
 * ───────────────────────────────────────────────────────────────────── */
#define PTP_TASK_PRIORITY 5
#define LED_TASK_STACK 2048
#define LED_TASK_PRIO 1  // low priority — never blocks anything
#define LED_TICK_MS 10   // pattern resolution
#define PPS_LED_ON_MS 50 // how long the pulse stays lit (tunable)
#define DISPLAY_TASK_STACK 4096
#define DISPLAY_TASK_PRIO 1    // lowest meaningful priority
#define DISPLAY_REFRESH_MS 500 // 2 Hz refresh
#define BUTTON_TASK_STACK 2048
#define BUTTON_TASK_PRIO 1
#define BUTTON_POLL_MS 20       // debounce poll interval
#define BUTTON_DEBOUNCE_TICKS 3 // 3 × 20ms = 60ms stable required

/* ─────────────────────────────────────────────────────────────────────
 * Delay_Req send queue
 *
 * Producer→consumer queue that decouples Delay_Req TX from the RX
 * hot path. Depth 16 sized for one Sync exchange in flight at 1 Hz
 * with comfortable burst headroom.
 * ───────────────────────────────────────────────────────────────────── */

/* B3: Producer→consumer queue that decouples Delay_Req TX from the
 * RX hot path. handle_follow_up calls enqueue_delay_req() (snapshot +
 * post, never blocks); delay_req_task drains the queue and performs
 * the actual L2TAP write. The queue holds a single uint8_t marker —
 * send_delay_req() builds the message from scratch using the slave's
 * own auto-incrementing sequence counter, so no per-job payload is
 * needed. Depth 16 sized for one Sync exchange in flight at the
 * Sync interval of 1 Hz (typical) with comfortable burst headroom. */
#define DELAY_REQ_Q_DEPTH 16

/* ─────────────────────────────────────────────────────────────────────
 * OLED display configuration
 *
 * Screens the user cycles through with the button, plus the quality
 * thresholds shown on Screen 2's Q100/Q1k row.
 * ───────────────────────────────────────────────────────────────────── */
#define DISPLAY_NUM_SCREENS 4

/* Two-band quality reporting on the servo screen: what fraction of the
 * last window of offset samples sits within each envelope.
 *   TIGHT: "excellent" — realistic ceiling on a quiet LAN with HW
 *          timestamping and a well-behaved GM. Field-experience target.
 *   LOOSE: "acceptable" — SW-timestamped hops or a moderately loaded
 *          switch. If the loose figure is also low, the offset noise
 *          is dominated by the network path, not the PTP servo.
 * Change these to re-tune what "Q%" means on Screen 2; they don't
 * feed the lock detector or any control loop. */
#define SLAVE_QUALITY_THRESHOLD_TIGHT_NS 100
#define SLAVE_QUALITY_THRESHOLD_LOOSE_NS 1000

/* ─────────────────────────────────────────────────────────────────────
 * Sentinels for atomics
 *
 * Magic values used to mean 'never observed'.
 * ───────────────────────────────────────────────────────────────────── */

/* Sentinel for g_gm_log_sync_interval: PTP logSyncInterval is a signed
 * int8_t on the wire; valid range is roughly [-7, +4] across profiles,
 * so 127 is safely "not yet observed". */
#define GM_LOG_SYNC_INTERVAL_UNKNOWN 127

/* ─────────────────────────────────────────────────────────────────────
 * Grandmaster diagnostics window
 *
 * Sliding window and print interval for gm_diag_task.
 * ───────────────────────────────────────────────────────────────────── */

/* ─────────────────────────────────────────────────────────────────────────
 * Grandmaster quality diagnostics
 *
 * These counters are updated by the PTP RX handlers (free of charge — no
 * extra work on the hot path) and consumed by gm_diag_task, which prints
 * a periodic summary to the console. Look at the report to decide whether
 * the connected GM is healthy.
 *
 * Metrics tracked:
 *   - Offset jitter            (std-dev of master offset over window)
 *   - Path delay stability     (std-dev of measured one-way delay)
 *   - Sync interval jitter     (time between consecutive Sync RX)
 *   - FollowUp latency         (Sync RX → FollowUp RX)
 *   - Sync sequence gaps       (missed/reordered Sync messages)
 *   - Announce sequence gaps   (missed Announce messages)
 *   - GM identity stability    (announces from same clockID?)
 *   - Two-step vs one-step
 *
 * Console output every GM_DIAG_INTERVAL_MS — read once a minute or so.
 * ───────────────────────────────────────────────────────────────────────── */
#define GM_DIAG_INTERVAL_MS 10000
#define GM_DIAG_WINDOW 64 /* samples kept for stats */

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 3: Type definitions
 *
 * PTP wire-format structures, BMCA dataset, slave state machine,
 * GM-diagnostics struct, LED states.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─── slave state enum ─── */
/* ─────────────────────────────────────────────────────────────────────────
 * Slave PTP state machine
 * ───────────────────────────────────────────────────────────────────────── */
typedef enum
{
    SLAVE_WAITING, /* waiting for first Sync */
    SLAVE_SYNCING, /* mid-exchange (T1, T2 captured, waiting for T3, T4) */
    SLAVE_SYNCED   /* offset computed, servo running */
} slave_state_t;

/* ─── gm_props_t type ─── */
/* Announce-derived GM properties — captured from the most recent Announce.
 * Spinlock-protected so the OLED render task can read them safely. */
typedef struct
{
    uint8_t clock_class;
    uint8_t accuracy;
    uint8_t priority1;
    uint8_t priority2;
    uint16_t variance;
    int16_t utc_offset;
    uint8_t time_source;
    uint16_t announce_flags;
    /* BMCA hop count from the announcer to the actual grandmaster.
     * 0 means the announcer IS the GM; >0 means we're behind that many
     * boundary clocks. Captured from Announce.stepsRemoved. */
    uint16_t steps_removed;
    bool valid;
    /* Track class transitions (e.g. 6 → 7) — used for the
     * "GM in holdover" warning in gm_diag and on screen 3. */
    uint8_t last_seen_class;
    uint32_t class_transition_count;
} gm_props_t;

/* ─── gm_diag banner + defines ─── */
/* ─────────────────────────────────────────────────────────────────────────
 * Grandmaster quality diagnostics
 *
 * These counters are updated by the PTP RX handlers (free of charge — no
 * extra work on the hot path) and consumed by gm_diag_task, which prints
 * a periodic summary to the console. Look at the report to decide whether
 * the connected GM is healthy.
 *
 * Metrics tracked:
 *   - Offset jitter            (std-dev of master offset over window)
 *   - Path delay stability     (std-dev of measured one-way delay)
 *   - Sync interval jitter     (time between consecutive Sync RX)
 *   - FollowUp latency         (Sync RX → FollowUp RX)
 *   - Sync sequence gaps       (missed/reordered Sync messages)
 *   - Announce sequence gaps   (missed Announce messages)
 *   - GM identity stability    (announces from same clockID?)
 *   - Two-step vs one-step
 *
 * Console output every GM_DIAG_INTERVAL_MS — read once a minute or so.
 * ───────────────────────────────────────────────────────────────────────── */
#define GM_DIAG_INTERVAL_MS 10000
#define GM_DIAG_WINDOW 64 /* samples kept for stats */

/* ─── gm_diag_t type ─── */
typedef struct
{
    /* Counters since boot */
    uint32_t sync_rx_count;
    uint32_t followup_rx_count;
    uint32_t delay_resp_rx_count;
    uint32_t announce_rx_count;
    uint32_t sync_seq_gaps;       /* missed Sync seq IDs */
    uint32_t announce_seq_gaps;   /* missed Announce seq IDs */
    uint32_t followup_orphans;    /* FollowUp w/o matching Sync */
    uint32_t delay_resp_orphans;  /* DelayResp w/o pending DReq */
    uint32_t two_step_count;      /* Syncs with TWO_STEP flag */
    uint32_t one_step_count;      /* Syncs without TWO_STEP flag */
    uint32_t gm_identity_changes; /* GM clockID changes seen */
    uint32_t post_lock_spikes;    /* |offset| > 1µs after lock — GM noise hint */

    /* Sequence tracking */
    uint16_t last_sync_seq_rx;
    bool sync_seq_initialized;
    uint16_t last_announce_seq_rx;
    bool announce_seq_initialized;

    /* GM identity tracking */
    uint8_t current_gm_id[8];
    bool gm_id_known;

    /* Timing windows (ring buffers) */
    int64_t offset_window[GM_DIAG_WINDOW];
    int64_t delay_window[GM_DIAG_WINDOW];
    int64_t sync_interval_window[GM_DIAG_WINDOW];    /* µs between Syncs */
    int64_t followup_latency_window[GM_DIAG_WINDOW]; /* µs Sync→FollowUp */
    int offset_idx, offset_count;
    int delay_idx, delay_count;
    int sync_int_idx, sync_int_count;
    int fu_lat_idx, fu_lat_count;

    /* For computing intervals */
    int64_t last_sync_rx_us;    /* esp_timer µs of last Sync RX */
    int64_t pending_sync_rx_us; /* set on Sync RX, cleared on FollowUp */
    uint16_t pending_sync_seq;  /* matches FollowUp to its Sync */
} gm_diag_t;

/* ─── bmca_dataset_t type ─── */
// BMCA
typedef struct
{
    uint8_t priority1;
    uint8_t clock_class;
    uint8_t accuracy;
    uint16_t variance;
    uint8_t priority2;
    uint8_t clock_id[8];
} bmca_dataset_t;

/* ─── PTP wire-format types (pragma pack + typedefs) ─── */
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

/* ─── servo_state_t enum (vestigial) ─── */
typedef enum
{
    SERVO_UNLOCKED = 0,
    SERVO_ACQUIRING,
    SERVO_LOCKED
} servo_state_t;

/* ─── led_state_t enum ─── */
typedef enum
{
    LED_STATE_NO_LINK = 0, // Ethernet not up
    LED_STATE_ACQUIRING,   // link up, waiting for GNSS / servo not locked
    LED_STATE_LOCKED,      // servo locked, time is good
    LED_STATE_HOLDOVER,    // was locked, GNSS lost, drifting
    LED_STATE_FAULT,       // unrecoverable error
} led_state_t;

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 4: Global state
 *
 * Slave-shaped state grouped by owning subsystem. Atomic flags
 * (g_link_up, g_l2tap_ready, g_phc_ready, g_emac_restarting) and
 * spinlock-protected structs are annotated with the writer / reader
 * task boundaries so lock ownership is obvious.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────
 * Logging tags
 *
 * Prepended to every ESP_LOG* line from the given subsystem.
 * ───────────────────────────────────────────────────────────────────── */
static const char *TAG = "PTP_SERVER";
static const char *LED_TAG = "LED";

/* ─────────────────────────────────────────────────────────────────────
 * Networking — netif, link, DHCP synchronization
 *
 * The netif exists from esp_netif_new() onwards; g_link_up flips
 * true on ETH_UP; g_got_ip_sem is given by the got-IP handler.
 * ───────────────────────────────────────────────────────────────────── */
static esp_netif_t *g_eth_netif = NULL;
static esp_eth_handle_t g_eth_hndl = NULL;
static int g_l2tap_fd = -1;
static SemaphoreHandle_t g_eth_up_sem;
static SemaphoreHandle_t g_got_ip_sem = NULL;
static _Atomic bool g_link_up = false;

/* A5: Signaled by on_link_down(). The long-running dhcp_fallback_task
 * (post-A4) blocks on this between cycles instead of polling g_link_up.
 * Binary semaphore — a give while a give is already pending is a benign
 * no-op. */
static SemaphoreHandle_t g_link_down_sem = NULL;
static _Atomic bool g_l2tap_ready = false;

/* B4: Serializes the shared g_l2tap_fd. Today only one task writes
 * (handle_follow_up calls send_delay_req synchronously from the RX
 * task), but B3 introduces a second writer (delay_req_task) and
 * emac_restart_preserve_phc takes this around its stop/start window. */
static SemaphoreHandle_t g_l2tap_tx_mtx = NULL;
static volatile bool g_static_applied = false;

/* A6: Atomic flag latched while emac_restart_preserve_phc() is in
 * progress. Any PHC ioctl or L2TAP write performed concurrently would
 * race the driver's internal stop/start teardown — see master line 994
 * and 2083 for the pattern. The three slave sites gate on this:
 *   - l2tap_send_ptp()         (fast-rejects to -1)
 *   - slave_compute_and_apply()'s ETH_MAC_ESP_CMD_S_PTP_TIME ioctl
 *   - slave_servo_freq()'s ETH_MAC_ESP_CMD_ADJ_PTP_TIME ioctl */
static _Atomic bool g_emac_restarting = false;
static volatile uint32_t g_emac_restart_count = 0;
static uint8_t g_src_mac[6];

/* ─────────────────────────────────────────────────────────────────────
 * Networking — hostname derived from Ethernet MAC
 *
 * g_hostname is the full dashed form used by DHCP / mDNS.
 * g_host_line1/2 are pre-split OLED halves. All three are populated
 * once in ethernet_init(); render sites guard on g_host_line1[0].
 * ───────────────────────────────────────────────────────────────────── */

/* g_hostname holds the full dashed form used for DHCP / mDNS
 * (e.g. "noble-abbot-marc-hark"). g_host_line1 and g_host_line2 are
 * the two pre-split OLED-friendly halves (max 13 + 17 chars) so the
 * boot splash and the network screen read a buffer instead of running
 * mac_hostname_display_lines() on every refresh. All three are
 * populated once in ethernet_init(); render sites guard with
 * g_host_line1[0] so any code path that beats ethernet_init to the
 * buffer falls back to a static label. */
static char g_hostname[MAC_HOSTNAME_BUF_LEN];
static char g_host_line1[14];
static char g_host_line2[18];

/* ─────────────────────────────────────────────────────────────────────
 * PTP — clock identity and known peers
 *
 * g_clock_id is our own 8-byte clockIdentity (derived from MAC).
 * g_master_mac / g_master_mac_known snapshot the MAC of the currently-
 * tracked GM (needed to address Delay_Req unicast).
 * ───────────────────────────────────────────────────────────────────── */
static uint8_t g_clock_id[8];
static const uint8_t PTP_MCAST_MAC[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};

/* For Delay_Req we need the master's MAC (from Sync sender, which is the master) */
static uint8_t g_master_mac[6] = {0};
static bool g_master_mac_known = false;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — slave state machine
 *
 * SLAVE_WAITING → SLAVE_SYNCING → SLAVE_SYNCED.
 * ───────────────────────────────────────────────────────────────────── */
static slave_state_t g_slave_state = SLAVE_WAITING;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — T1..T4 timestamps for the current Sync/Delay_Req exchange
 *
 * Written by handle_sync / handle_follow_up / handle_delay_resp
 * under g_slave_spinlock; consumed by slave_compute_and_apply.
 * ───────────────────────────────────────────────────────────────────── */

/* PTP exchange timestamps */
static struct timespec g_t1 = {0}; /* master TX of Sync (from FollowUp) */
static struct timespec g_t2 = {0}; /* slave RX of Sync (hw timestamp) */
static struct timespec g_t3 = {0}; /* slave TX of Delay_Req (hw timestamp) */
static struct timespec g_t4 = {0}; /* master RX of Delay_Req (from Delay_Resp) */
static bool g_t1_valid = false;
static bool g_t2_valid = false;
static bool g_t3_valid = false;
static uint16_t g_last_sync_seq = 0xFFFF;    /* track sync sequence */
static uint16_t g_pending_dreq_seq = 0xFFFF; /* outstanding Delay_Req seq */
static uint16_t g_dreq_seq = 0;              /* monotonic seq counter for our Delay_Req */

/* ─── TC residence-time corrections (ns) for the current exchange ───
 * A transparent clock adds its residence time to correctionField:
 *   - Sync (one-step TC) and/or FollowUp (two-step TC) → master→slave leg
 *   - Delay_Req → echoed back in Delay_Resp.correctionField by the
 *     master (IEEE 1588-2008 11.3.2 c) → slave→master leg
 * Written under g_slave_spinlock alongside their timestamps. */
static int64_t g_corr_sync = 0;
static int64_t g_corr_fu = 0;
static int64_t g_corr_dresp = 0;

/* correctionField is a big-endian int64 in units of ns × 2^16 (low 16
 * bits = fractional ns). bswap, then arithmetic >>16 to whole ns
 * (sign-preserving on GCC — negative values are legal, e.g. asymmetry
 * corrections, though a TC's residence time is always positive). */
static inline int64_t ptp_correction_ns(int64_t raw_be)
{
    return ((int64_t)__builtin_bswap64((uint64_t)raw_be)) >> 16;
}

/* Spinlock to protect g_t1..g_t4 across RX callbacks */
static portMUX_TYPE g_slave_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — Delay_Req send queue (RX-hot-path decoupling)
 *
 * handle_follow_up posts a marker onto g_delay_req_q; delay_req_task
 * drains it and performs the actual L2TAP write.
 * ───────────────────────────────────────────────────────────────────── */
static QueueHandle_t g_delay_req_q = NULL;

/* C4: Drop counter for B3's delay_req queue. Exposed in the web JSON. */
static _Atomic uint32_t g_delay_req_drops = 0;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — most recently observed servo outputs
 *
 * Values updated on every accepted Sync exchange; consumed by the
 * OLED, web /api/status, and gm_diag logging.
 * ───────────────────────────────────────────────────────────────────── */

/* Slave servo state */
static int64_t g_slave_offset_ns = 0;
static int64_t g_slave_path_delay_ns = 0;
static double g_slave_freq_ppb = 0;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — PI-servo integrator and its spinlock
 *
 * g_slave_integral_ppb accumulates across calls. g_servo_spinlock
 * protects shared 64-bit servo variables from tearing.
 * ───────────────────────────────────────────────────────────────────── */

/* PI-servo state — integrator preserved across calls. */
static double g_slave_integral_ppb = 0.0;

/* ─────────────────────────────────────────────────────────────────────────
 * Spinlock for shared 64-bit servo variables
 * ───────────────────────────────────────────────────────────────────────── */
static portMUX_TYPE g_servo_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — lock detection and streak counters
 *
 * g_slave_locked is the current flag; g_slave_ever_locked latches
 * once so slave_led_watch_task can distinguish 'was locked, now
 * silent' (HOLDOVER) from 'never locked' (still ACQUIRING).
 * ───────────────────────────────────────────────────────────────────── */
static int g_slave_stable_count = 0;
static bool g_slave_locked = false;

/* B7: Latches true the first time the slave reaches LOCKED with a valid
 * GM identity. Lets slave_led_watch_task distinguish
 * "Sync gone, was once LOCKED" → HOLDOVER from
 * "Sync gone, never LOCKED"   → keep ACQUIRING. */
static volatile bool g_slave_ever_locked = false;
static int64_t g_slave_last_sync_ms = 0; /* esp_timer ms of last good sample */

/* Track when servo entered LOCKED state — used for "lock duration" display */
static volatile int64_t g_lock_acquired_ms = 0;

/* B5: Reject-streak counters. slave_compute_and_apply increments these
 * when an outlier sample is discarded (post-lock spike). The live streak
 * is zeroed on every accepted sample; if it crosses
 * SLAVE_REJECT_STREAK_REACQUIRE we drop lock and reset the linreg state. */
static volatile uint32_t g_slave_total_rejects = 0;
static volatile uint32_t g_slave_consecutive_rejects = 0;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — linreg sample ring (frequency-drift estimation)
 *
 * (t, offset) samples over LINREG_WINDOW seconds; slope is reported
 * on the OLED servo screen and in gm_diag.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct
{
    int64_t t_ns;      /* sample time (slave PHC ns since boot) */
    int64_t offset_ns; /* PTP offset in ns */
} linreg_sample_t;
static linreg_sample_t g_linreg_samples[LINREG_WINDOW];
static int g_linreg_count = 0;
static int g_linreg_idx = 0;

/* ─────────────────────────────────────────────────────────────────────
 * PTP — asymmetry tracking
 *
 * PTP cannot observe true link asymmetry from a single exchange, but
 * if the slave is locked, the means of ms_to_sl vs sl_to_ms reveal
 * any constant asymmetry on the path.
 * ───────────────────────────────────────────────────────────────────── */
static int64_t g_ms_to_sl_window[ASYMMETRY_WINDOW] = {0};
static int64_t g_sl_to_ms_window[ASYMMETRY_WINDOW] = {0};
/* TC residence-time corrections per exchange (ns), same index/lock as
 * the half-trip windows above. Shown as windowed means on OLED screen 2.
 * All-zero on a direct cable; non-zero when a TC is correcting inline. */
static int64_t g_corr_m2s_window[ASYMMETRY_WINDOW] = {0};
static int64_t g_corr_s2m_window[ASYMMETRY_WINDOW] = {0};
static int g_asymm_idx = 0;
static int g_asymm_count = 0;
static portMUX_TYPE g_asymm_lock = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────
 * PHC — hardware clock state
 *
 * g_phc_ready flips true once phc_start_blocking succeeds — tasks
 * that touch the PHC gate on this. g_phc_stepped / g_slave_phc_stepped
 * record whether we've done a discontinuous clock jump.
 * ───────────────────────────────────────────────────────────────────── */

/* B2: "PHC is alive and may be safely read or written." Set true by
 * timing_core_bringup() once phc_start_blocking() succeeds. Tasks that
 * touch the PHC (slave_compute_and_apply, slave_servo_*) gate on this
 * instead of inferring it from g_l2tap_ready (the two are conceptually
 * distinct, even though they happen to be set together today). */
static _Atomic bool g_phc_ready = false;
static volatile bool g_phc_stepped = false;

/* Slave state machine flags */
static volatile bool g_slave_phc_stepped = false;
static volatile bool g_phc_residual_measured = false;
static volatile int64_t g_phc_offset_ns = 0; /* measured steady-state offset */
static volatile uint64_t g_last_progress_phc_s = 0;
static volatile uint64_t g_last_progress_check_ms = 0;

/* ─────────────────────────────────────────────────────────────────────
 * BMCA — best-master and candidate tracking
 *
 * g_best_master is what BMCA currently believes. g_candidate_master +
 * count implement the stability filter. g_i_am_master is a slave-build
 * no-op initialized to false so any status endpoint reports SLAVE
 * from instant zero.
 * ───────────────────────────────────────────────────────────────────── */
static bmca_dataset_t g_best_master = {0};

/* This is the slave build: we never transmit Announces and never
 * win BMCA. Initialize to false so any code that reads this flag
 * during the boot window (web JSON, OLED, logs) reports SLAVE
 * correctly from instant zero. */
static bool g_i_am_master = false;
static bmca_dataset_t g_candidate_master = {0};
static int g_candidate_count = 0;
static time_t g_last_announce_rx = 0;

/* ─────────────────────────────────────────────────────────────────────
 * Grandmaster properties (Announce-derived)
 *
 * Snapshot of the most recent Announce's advertised properties.
 * Spinlock-protected so the OLED render task can read them safely.
 * ───────────────────────────────────────────────────────────────────── */
static gm_props_t g_gm_props = {0};
static portMUX_TYPE g_gm_props_lock = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────
 * Grandmaster diagnostics (health counters)
 *
 * Updated on the PTP RX hot path (essentially free), consumed by
 * gm_diag_task and the OLED GM screen.
 * ───────────────────────────────────────────────────────────────────── */
static gm_diag_t g_gm_diag = {0};
static portMUX_TYPE g_gm_diag_lock = portMUX_INITIALIZER_UNLOCKED;

/* ─────────────────────────────────────────────────────────────────────
 * Sync-arrival heartbeat and rate advertisement
 *
 * g_sync_rx_pulse_seq / g_last_sync_ms drive the LED heartbeat and
 * the 'sync is flowing' indicators. g_gm_log_sync_interval captures
 * the GM's advertised logSyncInterval for the OLED rate label.
 * ───────────────────────────────────────────────────────────────────── */
static _Atomic uint32_t g_sync_rx_pulse_seq = 0; /* increments on every Sync RX */

/* Tracked separately from g_slave_last_sync_ms (which is updated only on
 * successful T1..T4 exchanges). We want the LED heartbeat to flash on
 * *Sync* arrival, not on "exchange completed". */
static volatile int64_t g_last_sync_ms = 0;

/* Most recent logSyncInterval advertised by the GM (carried in the PTP
 * header of every Sync message). Used by Screen 1 to render the sync
 * rate label ("1/s", "2/s", "1/2s", ...) truthfully instead of the old
 * hard-coded "1.0/s". Written from handle_sync, read from
 * render_screen_main; atomic so no lock is required. Sentinel value
 * GM_LOG_SYNC_INTERVAL_UNKNOWN (127) means "no Sync yet observed". */
static _Atomic int8_t g_gm_log_sync_interval = GM_LOG_SYNC_INTERVAL_UNKNOWN;

/* ─────────────────────────────────────────────────────────────────────
 * LED status indicator state
 *
 * s_led_state drives slave_led_task's pattern selection.
 * g_pps_led_pulse_seq gets bumped by handle_sync so the PPS-LED task
 * can pulse the second LED without any additional coupling.
 * ───────────────────────────────────────────────────────────────────── */

// Single shared state — written by app code, read by LED task.
// _Atomic gives us lock-free safe reads/writes on a 32-bit value.
static _Atomic led_state_t s_led_state = LED_STATE_NO_LINK;
static volatile uint32_t g_pps_led_pulse_seq = 0;

/* ─────────────────────────────────────────────────────────────────────
 * OLED display state
 *
 * g_display_screen is the currently-active screen index, cycled by
 * button_task on button press.
 * ───────────────────────────────────────────────────────────────────── */
static _Atomic uint8_t g_display_screen = 0; // 0..DISPLAY_NUM_SCREENS-1

/* ─────────────────────────────────────────────────────────────────────
 * System — boot time
 *
 * Captured very early in app_main; consumed by uptime-display paths.
 * ───────────────────────────────────────────────────────────────────── */

/* Track boot time for uptime display */
static volatile int64_t g_boot_time_ms = 0;

/* ─────────────────────────────────────────────────────────────────────
 * u8g2 display object
 *
 * Structure holding all the state for one SH1106 128×64 OLED.
 * Setup happens in InitDisplay(); render_screen_* functions operate
 * on it via u8g2_* draw calls.
 * ───────────────────────────────────────────────────────────────────── */
u8g2_t u8g2; // a structure which will contain all the data for one display

/* ─────────────────────────────────────────────────────────────────────
 * Legacy GNSS state (unused on this slave)
 *
 * Vestigial. Kept for structural parity with the master firmware.
 * ───────────────────────────────────────────────────────────────────── */
static volatile uint64_t g_gnss_pps_ns = 0;
static volatile bool g_gnss_pps_valid = false;

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 5: Forward declarations
 *
 * One block at the top so any function may safely call any other
 * regardless of emission order below.
 * ═════════════════════════════════════════════════════════════════════════ */

static void handle_sync(const uint8_t *payload, size_t len, const struct timespec *rx_hw_ts, const uint8_t *src_mac);
static void handle_follow_up(const uint8_t *payload, size_t len);
static void handle_delay_resp(const uint8_t *payload, size_t len);
static void send_delay_req(void);
static void enqueue_delay_req(void);
static void slave_compute_and_apply(void);
static void slave_servo_step(int64_t offset_ns);
static void slave_servo_freq(int64_t offset_ns);
static void linreg_push(int64_t t_ns, int64_t offset_ns);
static double linreg_compute_slope_ppb(void);
static void compute_stats_i64(const int64_t *samples, int n,
                              double *out_mean, double *out_stddev,
                              int64_t *out_min, int64_t *out_max);
static void gm_diag_flush_on_lock(int64_t offset_ns, int64_t delay_ns,
                                  int64_t ms_to_sl_ns, int64_t sl_to_ms_ns,
                                  int64_t corr_m2s_ns, int64_t corr_s2m_ns);

static void gm_diag_push_offset(int64_t offset_ns);
static void gm_diag_push_delay(int64_t delay_ns);
static void gm_diag_task(void *arg);

// ───── Forward Declarations ─────
static void announce_to_dataset(const ptp_announce_msg_t *ann, bmca_dataset_t *ds);
static void bmca_update(const bmca_dataset_t *remote);
static void handle_announce(const uint8_t *payload, size_t len);
static void emac_get_time(struct timespec *ts);
static esp_err_t l2tap_init(void);
static bool phc_start_blocking(void);
static void apply_static_ip(esp_netif_t *netif);

static void display_task(void *arg);
static void button_task(void *arg);
static void render_screen_main(void);
static void render_screen_servo(void);
static void render_screen_network(void);
static void render_screen_gm(void);

/* ─── Added by reorganization: cross-section forward decls ─── */
void led_set_state(led_state_t new_state);

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 6: Small helpers
 *
 * Pure functions and short utilities used throughout. Defined BEFORE
 * the code that uses them so a reader scanning top-down sees the
 * building blocks first.
 * ═════════════════════════════════════════════════════════════════════════ */

static int64_t timespec_to_ns(const struct timespec *ts)
{
    return (int64_t)ts->tv_sec * 1000000000LL + (int64_t)ts->tv_nsec;
}

/* Compute mean / stddev / min / max over n int64 samples.
 * Newton's-method sqrt avoids pulling in libm specifically for this. */
static void compute_stats_i64(const int64_t *samples, int n,
                              double *out_mean, double *out_stddev,
                              int64_t *out_min, int64_t *out_max)
{
    if (n <= 0)
    {
        *out_mean = 0;
        *out_stddev = 0;
        *out_min = 0;
        *out_max = 0;
        return;
    }
    double sum = 0;
    int64_t mn = samples[0], mx = samples[0];
    for (int i = 0; i < n; i++)
    {
        sum += (double)samples[i];
        if (samples[i] < mn)
            mn = samples[i];
        if (samples[i] > mx)
            mx = samples[i];
    }
    double mean = sum / n;
    double var = 0;
    for (int i = 0; i < n; i++)
    {
        double d = (double)samples[i] - mean;
        var += d * d;
    }
    var /= n;
    double sd = 0;
    if (var > 0)
    {
        double x = var;
        for (int k = 0; k < 25; k++)
            x = 0.5 * (x + var / x);
        sd = x;
    }
    *out_mean = mean;
    *out_stddev = sd;
    *out_min = mn;
    *out_max = mx;
}

static void linreg_push(int64_t t_ns, int64_t offset_ns)
{
    g_linreg_samples[g_linreg_idx].t_ns = t_ns;
    g_linreg_samples[g_linreg_idx].offset_ns = offset_ns;
    g_linreg_idx = (g_linreg_idx + 1) % LINREG_WINDOW;
    if (g_linreg_count < LINREG_WINDOW)
        g_linreg_count++;
}

/*
 * Least-squares slope of offset vs. time, normalized to ppb.
 * slope = sum((x-mean_x)*(y-mean_y)) / sum((x-mean_x)^2)
 * where x = time in seconds, y = offset in ns.
 * Result has units of ns/s = ppb.
 */
static double linreg_compute_slope_ppb(void)
{
    int n = g_linreg_count;
    if (n < 4)
        return 0.0; /* not enough samples yet */

    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < n; i++)
    {
        sum_x += (double)g_linreg_samples[i].t_ns / 1e9;
        sum_y += (double)g_linreg_samples[i].offset_ns;
    }
    double mean_x = sum_x / n;
    double mean_y = sum_y / n;

    double num = 0, den = 0;
    for (int i = 0; i < n; i++)
    {
        double x = (double)g_linreg_samples[i].t_ns / 1e9 - mean_x;
        double y = (double)g_linreg_samples[i].offset_ns - mean_y;
        num += x * y;
        den += x * x;
    }
    if (den < 1e-9)
        return 0.0;
    return num / den; /* ns per second = ppb */
}

/* ─────────────────────────────────────────────────────────────────────────
 * Grandmaster diagnostics implementation
 * ───────────────────────────────────────────────────────────────────────── */
static void gm_diag_push_offset(int64_t offset_ns)
{
    /* Reject pre-lock / leap-second-edge artifacts (~±1 s) before they pollute
     * the rolling mean/stddev/p2p. A locked PTP servo lives in the sub-µs band;
     * anything beyond 500 ms is not a servo offset, it is a cold-boot
     * timestamp-capture glitch and must not skew the quality report. */
    if (offset_ns > 500000000LL || offset_ns < -500000000LL)
        return;

    portENTER_CRITICAL(&g_gm_diag_lock);
    int i = g_gm_diag.offset_idx;
    g_gm_diag.offset_window[i] = offset_ns;
    g_gm_diag.offset_idx = (i + 1) % GM_DIAG_WINDOW;
    if (g_gm_diag.offset_count < GM_DIAG_WINDOW)
        g_gm_diag.offset_count++;
    portEXIT_CRITICAL(&g_gm_diag_lock);
}

static void gm_diag_push_delay(int64_t delay_ns)
{
    portENTER_CRITICAL(&g_gm_diag_lock);
    int i = g_gm_diag.delay_idx;
    g_gm_diag.delay_window[i] = delay_ns;
    g_gm_diag.delay_idx = (i + 1) % GM_DIAG_WINDOW;
    if (g_gm_diag.delay_count < GM_DIAG_WINDOW)
        g_gm_diag.delay_count++;
    portEXIT_CRITICAL(&g_gm_diag_lock);
}

/* ─────────────────────────────────────────────────────────────────────────
 * gm_diag_flush_on_lock
 *
 * Called exactly once, at the instant the slave servo latches LOCKED, to
 * discard the pre-lock acquisition transient from the statistics windows.
 *
 * Why this exists:
 *   The offset/delay/asymmetry ring buffers are 64 samples deep. During
 *   acquisition the offset runs tens of µs while the servo pulls in. Those
 *   large early samples linger in the window for up to 64 s after lock,
 *   inflating the windowed stddev and tripping the "High offset jitter" and
 *   transient "asymmetric path" warnings ~1 minute in — even though the
 *   clock is already steady. Flushing at lock latch makes the report reflect
 *   only post-lock, steady-state behaviour.
 *
 * Design notes:
 *   - We do NOT zero the counts to 0. The caller has already pushed the
 *     current (good, post-convergence) sample into each window earlier in
 *     slave_compute_and_apply(). Re-seeding the window with that single
 *     sample means the next GM_DIAG report reads n=1 with a sane value
 *     rather than momentarily showing "no samples".
 *   - Sync-interval and FollowUp-latency windows are NOT flushed: those
 *     measure GM-side cadence/turnaround, which is valid from the first
 *     packet and carries no acquisition transient. Keeping them preserves
 *     the longer history that makes their stddev meaningful.
 *   - Same spinlocks and field conventions as the rest of the module, so
 *     ordering vs. the RX-path push helpers is safe.
 * ───────────────────────────────────────────────────────────────────────── */
static void gm_diag_flush_on_lock(int64_t offset_ns, int64_t delay_ns,
                                  int64_t ms_to_sl_ns, int64_t sl_to_ms_ns,
                                  int64_t corr_m2s_ns, int64_t corr_s2m_ns)
{
    /* Re-seed the offset & delay windows with just the current sample. */
    portENTER_CRITICAL(&g_gm_diag_lock);
    g_gm_diag.offset_window[0] = offset_ns;
    g_gm_diag.offset_idx = 1 % GM_DIAG_WINDOW;
    g_gm_diag.offset_count = 1;

    g_gm_diag.delay_window[0] = delay_ns;
    g_gm_diag.delay_idx = 1 % GM_DIAG_WINDOW;
    g_gm_diag.delay_count = 1;

    /* Reset the post-lock spike counter too — spikes only have meaning once
     * locked, and any counted during the acquisition->lock edge are stale. */
    g_gm_diag.post_lock_spikes = 0;
    portEXIT_CRITICAL(&g_gm_diag_lock);

    /* Re-seed the asymmetry half-trip windows with the current half-trips. */
    portENTER_CRITICAL(&g_asymm_lock);
    g_ms_to_sl_window[0] = ms_to_sl_ns;
    g_sl_to_ms_window[0] = sl_to_ms_ns;
    g_corr_m2s_window[0] = corr_m2s_ns;
    g_corr_s2m_window[0] = corr_s2m_ns;
    g_asymm_idx = 1 % ASYMMETRY_WINDOW;
    g_asymm_count = 1;
    portEXIT_CRITICAL(&g_asymm_lock);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 7: EMAC PHC + PTP wire helpers
 *
 * Read the hardware PTP clock, decode/encode wire timestamps,
 * build PTP headers, send L2TAP frames with HW TX timestamping.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────────
 * Helpers
 * ───────────────────────────────────────────────────────────────────────── */
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

static void decode_ptp_timestamp(const ptp_timestamp_t *src, struct timespec *dst)
{
    uint64_t sec = 0;
    sec |= ((uint64_t)src->seconds_msb[0]) << 40;
    sec |= ((uint64_t)src->seconds_msb[1]) << 32;
    sec |= ((uint64_t)src->seconds_lsb[0]) << 24;
    sec |= ((uint64_t)src->seconds_lsb[1]) << 16;
    sec |= ((uint64_t)src->seconds_lsb[2]) << 8;
    sec |= ((uint64_t)src->seconds_lsb[3]);

    uint32_t ns = 0;
    ns |= ((uint32_t)src->nanoseconds[0]) << 24;
    ns |= ((uint32_t)src->nanoseconds[1]) << 16;
    ns |= ((uint32_t)src->nanoseconds[2]) << 8;
    ns |= ((uint32_t)src->nanoseconds[3]);

    dst->tv_sec = (time_t)sec;
    dst->tv_nsec = (long)ns;
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
    /* A6: Fast-reject while emac_restart_preserve_phc() is in flight.
     * A write through the L2TAP fd to a stopped EMAC is at best lost and
     * at worst races the driver's descriptor teardown. emac_restart_preserve_phc
     * ALSO takes g_l2tap_tx_mtx around its esp_eth_stop / esp_eth_start
     * window, so a caller that passed this atomic check just before the
     * restart began will drain its in-flight write() before stop() is
     * called. */
    if (atomic_load(&g_emac_restarting))
        return -1;

    size_t frame_len = 14 + ptp_len;

    /* A7: No heap on the hot path. Largest frame we ever send is a
     * Delay_Req (14 + 44 = 58 bytes); 14 + 96 leaves comfortable
     * headroom and avoids the per-TX malloc/free latency that biased
     * the T3 capture. */
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

    /* B4: Serialize the shared g_l2tap_fd. The fd is O_NONBLOCK so
     * write() returns immediately (EAGAIN if EMAC TX descriptors are
     * full), keeping the hold time tiny. The TX timestamp returns in
     * irec for THIS call's frame, so it must be read while still
     * holding the lock. */
    int ret;
    if (g_l2tap_tx_mtx)
        xSemaphoreTake(g_l2tap_tx_mtx, portMAX_DELAY);

    // TX timestamp is returned synchronously in irec after write()
    ret = write(g_l2tap_fd, &ext, 0);

    // ret > 0 means success AND timestamp is valid
    if (ret > 0 && tx_ts != NULL && irec->type == L2TAP_IREC_TIME_STAMP)
    {
        memcpy(tx_ts, irec->data, sizeof(struct timespec));
    }

    if (g_l2tap_tx_mtx)
        xSemaphoreGive(g_l2tap_tx_mtx);

    return ret;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 8: BMCA (Best Master Clock Algorithm)
 *
 * Slave-side only: compares received Announces to choose the best
 * grandmaster to follow. The slave never competes as a master, so
 * no Announce TX path.
 * ═════════════════════════════════════════════════════════════════════════ */

/* -----------------------------------------------------------------------
 * GNSS Grandmaster Code
 * ----------------------------------------------------------------------- */
static void announce_to_dataset(const ptp_announce_msg_t *ann, bmca_dataset_t *ds)
{
    ds->priority1 = ann->grandmaster_priority1;
    ds->clock_class = ann->grandmaster_clock_class;
    ds->accuracy = ann->grandmaster_clock_accuracy;
    ds->variance = ntohs(ann->grandmaster_clock_variance);
    ds->priority2 = ann->grandmaster_priority2;
    memcpy(ds->clock_id, ann->grandmaster_identity, 8);
}

static void bmca_update(const bmca_dataset_t *remote)
{
    /* Slave is fixed in slave role — accept any master that appears */
    g_i_am_master = false;
    memcpy(&g_best_master, remote, sizeof(bmca_dataset_t));
    return;
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 9: PTP message handlers
 *
 * Per-message-type handlers called from ptp_rx_task. Sync /
 * Follow_Up / Delay_Resp / Announce. Plus the Delay_Req producer
 * (enqueue_delay_req) and the synchronous sender (send_delay_req,
 * invoked by delay_req_task).
 * ═════════════════════════════════════════════════════════════════════════ */

static void handle_announce(const uint8_t *payload, size_t len)
{
    if (len < sizeof(ptp_announce_msg_t))
        return;

    const ptp_announce_msg_t *ann = (const ptp_announce_msg_t *)payload;

    bmca_dataset_t remote;
    announce_to_dataset(ann, &remote);

    g_last_announce_rx = time(NULL);

    /* ── Slave bonus: capture GM properties for OLED screen 3 ──
     * We store the most recent Announce-advertised properties and watch
     * for clockClass transitions (6 → 7 indicates the GM is in holdover). */
    {
        uint16_t flags = ntohs(ann->hdr.flags);
        uint8_t cls = ann->grandmaster_clock_class;

        portENTER_CRITICAL(&g_gm_props_lock);
        if (g_gm_props.valid && g_gm_props.last_seen_class != cls)
        {
            g_gm_props.class_transition_count++;
        }
        g_gm_props.clock_class = cls;
        g_gm_props.last_seen_class = cls;
        g_gm_props.accuracy = ann->grandmaster_clock_accuracy;
        g_gm_props.priority1 = ann->grandmaster_priority1;
        g_gm_props.priority2 = ann->grandmaster_priority2;
        g_gm_props.variance = ntohs(ann->grandmaster_clock_variance);
        g_gm_props.utc_offset = (int16_t)ntohs(ann->current_utc_offset);
        g_gm_props.time_source = ann->time_source;
        g_gm_props.announce_flags = flags;
        g_gm_props.steps_removed = ntohs(ann->steps_removed);
        g_gm_props.valid = true;
        portEXIT_CRITICAL(&g_gm_props_lock);
    }

    /* ── GM diagnostics: Announce accounting ── */
    uint16_t seq = ntohs(ann->hdr.sequence_id);
    portENTER_CRITICAL(&g_gm_diag_lock);
    g_gm_diag.announce_rx_count++;

    /* Announce sequence-gap detection */
    if (g_gm_diag.announce_seq_initialized)
    {
        uint16_t expected = (uint16_t)(g_gm_diag.last_announce_seq_rx + 1);
        if (seq != expected)
        {
            uint16_t gap = (uint16_t)(seq - expected);
            if (gap < 1000)
                g_gm_diag.announce_seq_gaps += gap;
        }
    }
    else
    {
        g_gm_diag.announce_seq_initialized = true;
    }
    g_gm_diag.last_announce_seq_rx = seq;

    /* GM identity stability — count distinct clockIDs */
    if (!g_gm_diag.gm_id_known)
    {
        memcpy(g_gm_diag.current_gm_id, remote.clock_id, 8);
        g_gm_diag.gm_id_known = true;
    }
    else if (memcmp(g_gm_diag.current_gm_id, remote.clock_id, 8) != 0)
    {
        g_gm_diag.gm_identity_changes++;
        memcpy(g_gm_diag.current_gm_id, remote.clock_id, 8);
    }
    portEXIT_CRITICAL(&g_gm_diag_lock);

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

    // Apply BMCA only after stability threshold
    if (g_candidate_count >= BMCA_STABILITY_THRESHOLD)
    {
        bmca_update(&remote);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * Logic Functions
 * ───────────────────────────────────────────────────────────────────────── */
static void handle_sync(const uint8_t *payload, size_t len,
                        const struct timespec *rx_hw_ts,
                        const uint8_t *src_mac)
{
    if (len < sizeof(ptp_sync_msg_t))
        return;

    const ptp_sync_msg_t *sync = (const ptp_sync_msg_t *)payload;
    uint16_t seq = ntohs(sync->hdr.sequence_id);
    uint16_t flags = ntohs(sync->hdr.flags);

    /* Publish the GM's advertised logSyncInterval so Screen 1 can label
     * the sync rate correctly (see render_screen_main). Kept as a plain
     * atomic — no need to take g_gm_diag_lock or g_gm_props_lock for a
     * single int8_t that only the OLED reads. */
    atomic_store(&g_gm_log_sync_interval,
                 (int8_t)sync->hdr.log_msg_interval);

    /* Heartbeat: feed the red-LED task and the "drifting" detector.
     * These two are deliberately first — they should fire even if the rest
     * of this handler bails out early on a malformed packet. */
    atomic_fetch_add(&g_sync_rx_pulse_seq, 1);
    g_last_sync_ms = esp_timer_get_time() / 1000;

    /* ── GM diagnostics: Sync RX bookkeeping ── */
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&g_gm_diag_lock);
    g_gm_diag.sync_rx_count++;
    if (flags & PTP_FLAG_TWO_STEP)
        g_gm_diag.two_step_count++;
    else
        g_gm_diag.one_step_count++;

    /* Sync sequence-gap detection */
    if (g_gm_diag.sync_seq_initialized)
    {
        uint16_t expected = (uint16_t)(g_gm_diag.last_sync_seq_rx + 1);
        if (seq != expected)
        {
            /* gap = seq - expected, accounting for 16-bit wrap */
            uint16_t gap = (uint16_t)(seq - expected);
            if (gap < 1000) /* sanity — ignore obvious resets/wraparounds */
                g_gm_diag.sync_seq_gaps += gap;
        }
    }
    else
    {
        g_gm_diag.sync_seq_initialized = true;
    }
    g_gm_diag.last_sync_seq_rx = seq;

    /* Sync interval (µs between successive Sync RX) */
    if (g_gm_diag.last_sync_rx_us != 0)
    {
        int64_t dt = now_us - g_gm_diag.last_sync_rx_us;
        int i = g_gm_diag.sync_int_idx;
        g_gm_diag.sync_interval_window[i] = dt;
        g_gm_diag.sync_int_idx = (i + 1) % GM_DIAG_WINDOW;
        if (g_gm_diag.sync_int_count < GM_DIAG_WINDOW)
            g_gm_diag.sync_int_count++;
    }
    g_gm_diag.last_sync_rx_us = now_us;

    /* Remember this Sync's RX time for FollowUp-latency measurement */
    g_gm_diag.pending_sync_rx_us = now_us;
    g_gm_diag.pending_sync_seq = seq;
    portEXIT_CRITICAL(&g_gm_diag_lock);

    /* Remember master's MAC for the OLED network screen only.
     * Delay_Req is deliberately sent to the standard PTP multicast
     * address (TC-friendly and standards-correct), never unicast. */
    if (!g_master_mac_known)
    {
        memcpy(g_master_mac, src_mac, 6);
        g_master_mac_known = true;
        ESP_LOGI("SLAVE", "Master MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 src_mac[0], src_mac[1], src_mac[2],
                 src_mac[3], src_mac[4], src_mac[5]);
    }

    /* Guard: a missing RX hardware timestamp (irec not filled by the
     * driver) arrives as exact 0.0. Storing it as T2 would produce a
     * monster offset and a garbage re-step from a single driver hiccup.
     * Mirrors the TX-side "real HW timestamps can't be 0.0" check in
     * send_delay_req. Heartbeat/diagnostics above already counted this
     * Sync; only the timing capture is skipped. */
    if (rx_hw_ts->tv_sec == 0 && rx_hw_ts->tv_nsec == 0)
    {
        ESP_LOGW("SLAVE", "Sync seq=%u: RX HW timestamp missing — skipping T2 capture",
                 seq);
        return;
    }

    taskENTER_CRITICAL(&g_slave_spinlock);
    g_t2 = *rx_hw_ts;
    g_t2_valid = true;
    g_last_sync_seq = seq;
    g_corr_sync = ptp_correction_ns(sync->hdr.correction_field);
    g_corr_fu = 0; /* reset; the matching FollowUp (two-step) sets it */
    /* Two-step: T1 will arrive in FollowUp.
     * One-step (no TWO_STEP flag): T1 is in originTimestamp, capture now. */
    if (!(flags & PTP_FLAG_TWO_STEP))
    {
        decode_ptp_timestamp(&sync->origin_timestamp, &g_t1);
        g_t1_valid = true;
    }
    taskEXIT_CRITICAL(&g_slave_spinlock);

    /* One-step exchange completion: with no FollowUp coming, the
     * Delay_Req trigger must fire here (the two-step path fires it from
     * handle_follow_up). Without this, a one-step master never receives
     * a Delay_Req and the slave sits in SYNCING forever, silently. */
    if (!(flags & PTP_FLAG_TWO_STEP) && g_master_mac_known)
    {
        enqueue_delay_req();
    }

    ESP_LOGD("SLAVE", "Sync seq=%u T2=%lld.%09ld",
             seq, (long long)rx_hw_ts->tv_sec, (long)rx_hw_ts->tv_nsec);
}

static void handle_follow_up(const uint8_t *payload, size_t len)
{
    if (len < sizeof(ptp_sync_msg_t))
        return;

    const ptp_sync_msg_t *fu = (const ptp_sync_msg_t *)payload;
    uint16_t seq = ntohs(fu->hdr.sequence_id);

    /* ── GM diagnostics: FollowUp accounting ── */
    int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&g_gm_diag_lock);
    g_gm_diag.followup_rx_count++;
    if (g_gm_diag.pending_sync_rx_us != 0 && seq == g_gm_diag.pending_sync_seq)
    {
        int64_t lat = now_us - g_gm_diag.pending_sync_rx_us;
        int i = g_gm_diag.fu_lat_idx;
        g_gm_diag.followup_latency_window[i] = lat;
        g_gm_diag.fu_lat_idx = (i + 1) % GM_DIAG_WINDOW;
        if (g_gm_diag.fu_lat_count < GM_DIAG_WINDOW)
            g_gm_diag.fu_lat_count++;
        g_gm_diag.pending_sync_rx_us = 0;
    }
    else
    {
        g_gm_diag.followup_orphans++;
    }
    portEXIT_CRITICAL(&g_gm_diag_lock);

    taskENTER_CRITICAL(&g_slave_spinlock);
    /* Only accept FollowUp matching the most recent Sync */
    if (seq == g_last_sync_seq)
    {
        decode_ptp_timestamp(&fu->origin_timestamp, &g_t1);
        g_corr_fu = ptp_correction_ns(fu->hdr.correction_field);
        g_t1_valid = true;
    }
    taskEXIT_CRITICAL(&g_slave_spinlock);

    ESP_LOGD("SLAVE", "FollowUp seq=%u T1=%lld.%09ld",
             seq, (long long)g_t1.tv_sec, (long)g_t1.tv_nsec);

    /* B3: Now we have T1 and T2 — enqueue a Delay_Req for the
     * delay_req_task to send. Decoupling avoids stalling ptp_rx during
     * the L2TAP write + TX-timestamp wait, so back-to-back Syncs on a
     * chatty network are not missed. */
    if (g_master_mac_known)
    {
        enqueue_delay_req();
    }
}

static void handle_delay_resp(const uint8_t *payload, size_t len)
{
    if (len < sizeof(ptp_delay_resp_msg_t))
        return;

    const ptp_delay_resp_msg_t *resp = (const ptp_delay_resp_msg_t *)payload;
    uint16_t seq = ntohs(resp->hdr.sequence_id);

    /* Must be addressed to us — check requesting_port_id.clock_identity */
    if (memcmp(resp->requesting_port_id.clock_identity, g_clock_id, 8) != 0)
        return;

    portENTER_CRITICAL(&g_gm_diag_lock);
    g_gm_diag.delay_resp_rx_count++;
    portEXIT_CRITICAL(&g_gm_diag_lock);

    taskENTER_CRITICAL(&g_slave_spinlock);
    bool seq_match = (seq == g_pending_dreq_seq);
    if (seq_match)
    {
        decode_ptp_timestamp(&resp->receive_timestamp, &g_t4);
        g_corr_dresp = ptp_correction_ns(resp->hdr.correction_field);
    }
    bool ready = seq_match && g_t1_valid && g_t2_valid && g_t3_valid;
    taskEXIT_CRITICAL(&g_slave_spinlock);

    if (!seq_match)
    {
        portENTER_CRITICAL(&g_gm_diag_lock);
        g_gm_diag.delay_resp_orphans++;
        portEXIT_CRITICAL(&g_gm_diag_lock);
        ESP_LOGD("SLAVE", "Delay_Resp seq mismatch: got %u, expected %u",
                 seq, g_pending_dreq_seq);
        return;
    }

    ESP_LOGD("SLAVE", "Delay_Resp seq=%u T4=%lld.%09ld",
             seq, (long long)g_t4.tv_sec, (long)g_t4.tv_nsec);

    if (ready)
    {
        slave_compute_and_apply();
    }
}

/* B3: Producer side. Called from the ptp_rx task at most once per
 * Sync/FollowUp exchange. Never blocks; on queue full, increments
 * g_delay_req_drops so the overload is observable on the web JSON. */
static void enqueue_delay_req(void)
{
    if (!g_delay_req_q)
        return;
    uint8_t marker = 1;
    if (xQueueSend(g_delay_req_q, &marker, 0) != pdTRUE)
    {
        atomic_fetch_add(&g_delay_req_drops, 1);
    }
}

static void send_delay_req(void)
{
    ptp_delay_req_msg_t req = {0};
    uint16_t seq = g_dreq_seq++;

    build_ptp_header(&req.hdr, PTP_MSG_DELAY_REQ, sizeof(ptp_delay_req_msg_t),
                     seq, 0, 0, 0x01);
    /* Per IEEE 1588, originTimestamp in Delay_Req is left as zeros;
     * the slave records its own TX timestamp locally as T3. */

    struct timespec t3_local = {0};
    int ret = l2tap_send_ptp(&req, sizeof(req), &t3_local);
    if (ret <= 0)
    {
        ESP_LOGW("SLAVE", "Delay_Req send failed: %d errno=%d", ret, errno);
        return;
    }

    if (t3_local.tv_sec == 0 && t3_local.tv_nsec == 0)
    {
        /* EMAC didn't fill the TX timestamp irec. This shouldn't happen
         * after PHC has been disciplined (real HW timestamps can't be
         * 0.0 once we're at year-2026 wall-clock). Fall back to a software
         * PHC read, but warn — the resulting T3 will be ~10s of µs late
         * and the computed offset will be biased by half that. */
        ESP_LOGW("SLAVE", "Delay_Req seq=%u: HW TX timestamp unavailable, using SW fallback",
                 seq);
        emac_get_time(&t3_local); /* fallback */
    }

    taskENTER_CRITICAL(&g_slave_spinlock);
    g_t3 = t3_local;
    g_t3_valid = true;
    g_pending_dreq_seq = seq;
    taskEXIT_CRITICAL(&g_slave_spinlock);

    ESP_LOGD("SLAVE", "Delay_Req sent seq=%u T3=%lld.%09ld",
             seq, (long long)t3_local.tv_sec, (long)t3_local.tv_nsec);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 10: Servo + lock management
 *
 * Slave's truth source is the wire (master's Sync timestamps).
 * slave_compute_and_apply does the four-timestamp PTP offset math
 * and decides between step and freq correction. Reject-streak +
 * reacquire (B5) and ever-locked latch (B7) live here.
 * ═════════════════════════════════════════════════════════════════════════ */

static void slave_compute_and_apply(void)
{
    struct timespec t1, t2, t3, t4;
    int64_t corr_m2s, corr_s2m;

    taskENTER_CRITICAL(&g_slave_spinlock);
    t1 = g_t1;
    t2 = g_t2;
    t3 = g_t3;
    t4 = g_t4;
    corr_m2s = g_corr_sync + g_corr_fu;
    corr_s2m = g_corr_dresp;
    /* Invalidate so we wait for next full exchange */
    g_t1_valid = false;
    g_t2_valid = false;
    g_t3_valid = false;
    taskEXIT_CRITICAL(&g_slave_spinlock);

    int64_t t1_ns = timespec_to_ns(&t1);
    int64_t t2_ns = timespec_to_ns(&t2);
    int64_t t3_ns = timespec_to_ns(&t3);
    int64_t t4_ns = timespec_to_ns(&t4);

    /* PTP standard (IEEE 1588-2008 11.2 / 11.3), TC-aware:
     *   offset = ((t2 - t1 - corr_m2s) - (t4 - t3 - corr_s2m)) / 2
     *   delay  = ((t2 - t1 - corr_m2s) + (t4 - t3 - corr_s2m)) / 2
     * corr_* carry the switch residence times accumulated in the
     * correctionFields; with no TC inline they are simply 0 and this
     * reduces to the classic four-timestamp form.
     * Where positive offset means slave PHC is AHEAD of master.
     */
    int64_t ms_to_sl = (t2_ns - t1_ns) - corr_m2s; /* master->slave one-way + offset */
    int64_t sl_to_ms = (t4_ns - t3_ns) - corr_s2m; /* slave->master one-way - offset */
    int64_t offset = (ms_to_sl - sl_to_ms) / 2;
    int64_t delay = (ms_to_sl + sl_to_ms) / 2;

    /* Bench observable: non-zero corrections prove the switch's TC is
     * matching and correcting our frames. Stuck at 0 with a TC inline
     * ⇒ wrong Protocol / VLAN / domain on the switch's PTP page. */
    // Being printed as ESP_LOGI elsewhere
    // ESP_LOGD("SLAVE", "TC corr m2s=%lld ns s2m=%lld ns",
    //          (long long)corr_m2s, (long long)corr_s2m);

    /* Sanity check: huge offsets = first sync, do a step */
    if (llabs(offset) > 1000000LL && !g_slave_phc_stepped)
    {
        ESP_LOGW("SLAVE", "Large offset %lld ns - stepping PHC", (long long)offset);
        slave_servo_step(offset);
        g_slave_phc_stepped = true;
        g_slave_state = SLAVE_SYNCED;
        return;
    }

    g_slave_offset_ns = offset;
    g_slave_path_delay_ns = delay;
    g_slave_state = SLAVE_SYNCED;

    /* Feed offset + delay into GM-quality diagnostics windows */
    gm_diag_push_offset(offset);
    gm_diag_push_delay(delay);

    /* Asymmetry tracker — record the two half-trips separately.
     *
     * PTP fundamentally cannot recover absolute link asymmetry from a single
     * exchange; any constant asymmetry is absorbed into the offset. But by
     * showing the *mean* of (t2-t1) and (t4-t3) over a window we can:
     *
     *   - Verify symmetry: their means should differ by ~0 on a direct cable.
     *   - Measure each direction's one-way delay (mean ± stddev each way).
     *   - Detect a routed/switch'd asymmetric path: persistent mean
     *     difference reveals the offset bias (≈ Δ/2). */
    {
        portENTER_CRITICAL(&g_asymm_lock);
        g_ms_to_sl_window[g_asymm_idx] = ms_to_sl;
        g_sl_to_ms_window[g_asymm_idx] = sl_to_ms;
        g_corr_m2s_window[g_asymm_idx] = corr_m2s;
        g_corr_s2m_window[g_asymm_idx] = corr_s2m;
        g_asymm_idx = (g_asymm_idx + 1) % ASYMMETRY_WINDOW;
        if (g_asymm_count < ASYMMETRY_WINDOW)
            g_asymm_count++;
        portEXIT_CRITICAL(&g_asymm_lock);
    }

    /* Track post-lock disturbances >1µs — these usually indicate GM-side
     * noise (e.g. GNSS PPS jitter) rather than slave servo problems.
     *
     * B5: Reject-streak driven re-acquire. A post-lock spike both
     * increments the post_lock_spikes counter (kept) and counts against
     * a reject streak. If the GM goes pathological (its Sync TX path is
     * starved by load and timestamps drift into the noise), the streak
     * grows; on crossing SLAVE_REJECT_STREAK_REACQUIRE we drop lock and
     * reset the servo so we don't waste cycles disciplining against
     * noise. The lifetime counter is kept for the web/OLED diagnostics. */
    if (g_slave_locked && llabs(offset) > 1000)
    {
        portENTER_CRITICAL(&g_gm_diag_lock);
        g_gm_diag.post_lock_spikes++;
        portEXIT_CRITICAL(&g_gm_diag_lock);

        g_slave_total_rejects++;
        g_slave_consecutive_rejects++;

        if (g_slave_consecutive_rejects >= SLAVE_REJECT_STREAK_REACQUIRE)
        {
            ESP_LOGW("SLAVE",
                     "Reject streak %u >= %d — forcing reacquire",
                     (unsigned)g_slave_consecutive_rejects,
                     SLAVE_REJECT_STREAK_REACQUIRE);
            g_slave_locked = false;
            g_slave_stable_count = 0;
            g_lock_acquired_ms = 0;
            g_slave_consecutive_rejects = 0;
            g_linreg_count = 0;
            g_linreg_idx = 0;
            led_set_state(LED_STATE_ACQUIRING);
            return;
        }

        /* B5-FIX: a rejected sample must NOT reach the servo or the
         * linreg window below — falling through would inject the very
         * spike we just rejected (KP+KI at 1 Hz re-adds ~the full spike
         * as phase error and pollutes the integrator until it unwinds).
         * Refresh the exchange timestamp so the rx task's 5 s master-
         * timeout doesn't preempt the 45-reject streak logic above.
         * Diagnostics (offset/delay/asymmetry windows, OLED offset)
         * were already fed before this check and still see the spike. */
        g_slave_last_sync_ms = esp_timer_get_time() / 1000;
        return;
    }
    else
    {
        /* Accepted sample (within ±1 µs while locked, OR not currently locked). */
        g_slave_consecutive_rejects = 0;
    }

    /* Push to linreg window using slave PHC time as x-axis */
    linreg_push(t2_ns, offset);

    /* Apply frequency correction */
    slave_servo_freq(offset);

    /* Re-step only on genuinely large excursions. Below that, let the
     * frequency servo trim — re-stepping every cycle creates an endless
     * "step → drift → re-step" loop because each step zeros out the
     * crystal-drift estimate.
     *
     * 500 µs threshold: well above per-second drift from any reasonable
     * crystal (≤ ~50 ppm = 50 µs/s), but small enough that we genuinely
     * step out of any pathological hole. */
    if (llabs(offset) > 500000LL)
    {
        ESP_LOGW("SLAVE", "Residual offset %lld ns - re-stepping", (long long)offset);
        slave_servo_step(offset);
        g_slave_stable_count = 0;
        if (g_slave_locked)
        {
            g_slave_locked = false;
            led_set_state(LED_STATE_ACQUIRING);
        }
        return;
    }

    /* ─────────────────────────────────────────────
     * Lock tracking
     * ───────────────────────────────────────────── */
    g_slave_last_sync_ms = esp_timer_get_time() / 1000;

    /* Is the grandmaster advertising a traceable clockClass right now?
     * During cold boot the GM sits in clockClass 7 (HOLDOVER) until its own
     * GNSS servo locks. We must not advertise a trustworthy LOCKED to a GM
     * that is itself not yet traceable, even though our PHC may track it. */
    bool gm_traceable;
    {
        gm_props_t gp_lk;
        portENTER_CRITICAL(&g_gm_props_lock);
        gp_lk = g_gm_props;
        portEXIT_CRITICAL(&g_gm_props_lock);
        /* Traceable if we have a valid Announce and class < 7 (e.g. 6 = LOCKED).
         * If no Announce yet, treat as not-traceable. */
        gm_traceable = gp_lk.valid && (gp_lk.clock_class < 7);
    }

    if (!g_slave_locked)
    {
        /* Acquiring — count consecutive good samples, but only allow LOCK to
         * latch once the GM itself is traceable. */
        if (llabs(offset) < SLAVE_LOCK_OFFSET_NS && gm_traceable)
        {
            if (++g_slave_stable_count >= SLAVE_LOCK_THRESHOLD)
            {
                g_slave_locked = true;
                /* B7: Latch — never goes false again within a power cycle.
                 * Lets slave_led_watch_task distinguish "Sync gone, was
                 * once LOCKED → HOLDOVER" from "Sync gone, never LOCKED
                 * → keep ACQUIRING". */
                g_slave_ever_locked = true;
                ESP_LOGI("SLAVE", "LOCKED (offset=%lld ns, GM traceable)", (long long)offset);
                led_set_state(LED_STATE_LOCKED);
                g_lock_acquired_ms = esp_timer_get_time() / 1000;

                /* Discard the pre-lock acquisition transient so the GM_DIAG
                 * report reflects steady-state behaviour from here on. The
                 * current sample is already in the windows; re-seed with it. */
                gm_diag_flush_on_lock(offset, delay, ms_to_sl, sl_to_ms,
                                      corr_m2s, corr_s2m);
            }
        }
        else
        {
            g_slave_stable_count = 0;
        }
    }
    else
    {
        /* Locked — drop out on a clear servo excursion OR if the GM drops
         * back into holdover (class >= 7): its time is no longer traceable,
         * so neither is ours. */
        if (llabs(offset) > SLAVE_UNLOCK_OFFSET_NS)
        {
            g_slave_locked = false;
            g_slave_stable_count = 0;
            g_lock_acquired_ms = 0;
            ESP_LOGW("SLAVE", "Lock lost (offset=%lld ns)", (long long)offset);
            led_set_state(LED_STATE_ACQUIRING);
        }
        else if (!gm_traceable)
        {
            g_slave_locked = false;
            g_slave_stable_count = 0;
            g_lock_acquired_ms = 0;
            ESP_LOGW("SLAVE", "Lock withheld — GM advertising clockClass >= 7 (HOLDOVER), time not traceable");
            led_set_state(LED_STATE_ACQUIRING);
        }
    }

    ESP_LOGI("SLAVE",
             "offset=%lld ns delay=%lld ns corr=%lld/%lld freq=%.0f ppb %s",
             (long long)offset, (long long)delay,
             (long long)corr_m2s, (long long)corr_s2m, g_slave_freq_ppb,
             g_slave_locked ? "[LOCKED]"
                            : (gm_traceable ? "[ACQUIRING]" : "[ACQUIRING-GM HOLDOVER]"));
}

static void slave_servo_step(int64_t offset_ns)
{
    /* Step PHC by -offset. Read current, subtract, write back.
     *
     * NOTE: we deliberately do NOT zero the frequency adjustment here.
     * Crystal drift is a property of the hardware, not the time-value
     * we're stepping. Zeroing freq every step throws away the servo's
     * accumulated knowledge of the local oscillator's drift rate, which
     * causes the "step → drift → re-step" oscillation that prevents lock.
     */

    /* A6/B2: Skip if the PHC isn't ready yet, or if an EMAC restart is
     * in flight. The S_PTP_TIME write races the driver's stop/start. */
    if (!atomic_load(&g_phc_ready) || atomic_load(&g_emac_restarting))
        return;

    struct timespec now;
    emac_get_time(&now);
    int64_t now_ns = timespec_to_ns(&now);
    int64_t target_ns = now_ns - offset_ns;
    if (target_ns < 0)
        target_ns = 0;

    eth_mac_time_t t = {
        .seconds = (uint32_t)(target_ns / 1000000000LL),
        .nanoseconds = (uint32_t)(target_ns % 1000000000LL),
    };
    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_S_PTP_TIME, &t);

    /* Reset linreg window — the (t, offset) history is no longer valid
     * after a time discontinuity. Frequency estimate is preserved. */
    g_linreg_count = 0;
    g_linreg_idx = 0;
}

#define SLAVE_FREQ_LIMIT_PPB 500000.0     /* ±500 ppm output clamp */
#define SLAVE_INTEGRAL_LIMIT_PPB 200000.0 /* ±200 ppm integrator clamp */

static void slave_servo_freq(int64_t offset_ns)
{
    /*
     * PI servo:
     *
     *   offset > 0 → slave PHC ahead of master → freq must be NEGATIVE (slow PHC)
     *   offset < 0 → slave PHC behind master   → freq must be POSITIVE (speed PHC)
     *
     * Units:
     *   offset is in ns. Update rate is 1 Hz. So offset itself measures
     *   the per-second phase error, i.e. the instantaneous frequency
     *   error in ppb. KP=1 would mean "fully cancel the error in one
     *   sample" (overshoots); KP=0.7 leaves 30% to the next sample.
     *
     *   Integrator accumulates the residual to chase out the steady-state
     *   crystal drift component.
     */

    /* Integrator update (ns·1Hz = ppb·s contribution per sample) */
    g_slave_integral_ppb += SLAVE_SERVO_KI * (double)offset_ns;

    /* Anti-windup */
    if (g_slave_integral_ppb > SLAVE_INTEGRAL_LIMIT_PPB)
        g_slave_integral_ppb = SLAVE_INTEGRAL_LIMIT_PPB;
    if (g_slave_integral_ppb < -SLAVE_INTEGRAL_LIMIT_PPB)
        g_slave_integral_ppb = -SLAVE_INTEGRAL_LIMIT_PPB;

    double correction_ppb =
        SLAVE_SERVO_KP * (double)offset_ns + g_slave_integral_ppb;

    /* Update g_slave_freq_ppb under the servo spinlock so that the
     * emac_restart_preserve_phc snapshot (in the watchdog) reads a
     * coherent value with the freq we apply below. */
    taskENTER_CRITICAL(&g_servo_spinlock);
    g_slave_freq_ppb = -correction_ppb;

    /* Clamp output */
    if (g_slave_freq_ppb > SLAVE_FREQ_LIMIT_PPB)
        g_slave_freq_ppb = SLAVE_FREQ_LIMIT_PPB;
    if (g_slave_freq_ppb < -SLAVE_FREQ_LIMIT_PPB)
        g_slave_freq_ppb = -SLAVE_FREQ_LIMIT_PPB;

    int32_t adj = (int32_t)g_slave_freq_ppb;
    taskEXIT_CRITICAL(&g_servo_spinlock);

    /* A6/B2: Skip the actual ioctl write if PHC isn't ready yet, or if
     * an EMAC restart is in flight. The integrator state above stays
     * updated so the servo doesn't lose track during the dead window. */
    if (!atomic_load(&g_phc_ready) || atomic_load(&g_emac_restarting))
        return;

    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 11: Background tasks
 *
 * Long-running FreeRTOS tasks for PTP RX, Delay_Req TX, and
 * periodic diagnostics reporting.
 * ═════════════════════════════════════════════════════════════════════════ */

/* ─────────────────────────────────────────────────────────────────────────
 * Tasks
 * ───────────────────────────────────────────────────────────────────────── */
static void ptp_rx_task(void *arg)
{
    /* Wait for L2TAP to be ready (set by on_link_up on first connect) */
    while (!atomic_load(&g_l2tap_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // ESP_LOGI(TAG, "ptp_rx_task started");

    size_t irec_space = L2TAP_IREC_SPACE(sizeof(struct timespec));

    union
    {
        uint8_t buf[L2TAP_IREC_SPACE(sizeof(struct timespec))];
        l2tap_irec_hdr_t align;
    } irec_u;

    /* Allocate RX scratch buffer once for the lifetime of this task.
     * Never freed because this task never exits. */
    uint8_t *frame_buf = calloc(1, 1536);
    assert(frame_buf);

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

        // Timeout → still need to check Announce timeout
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

        /* Accept only PTPv2 frames in OUR domain (port of the identical
         * gate in master.c's rx path, added there after a real cross-
         * domain incident). versionPTP is the low nibble of byte 1 —
         * the high nibble is minorVersionPTP in 1588-2019, so mask it.
         * On a shared switch, foreign-domain or v1 frames would
         * otherwise feed our T1/T2 capture and BMCA. */
        {
            const ptp_header_t *rx_hdr =
                (const ptp_header_t *)(eth_frame + 14);
            if ((rx_hdr->version_ptp & 0x0F) != 0x02 ||
                rx_hdr->domain_number != PTP_DOMAIN)
                goto timeout_check;
        }

        uint8_t msg_type = eth_frame[14] & 0x0F;

        // ESP_LOGD("PTP_RX", "Type 0x%X @ %lld.%09ld",
        //          msg_type,
        //          (long long)hw_ts.tv_sec,
        //          (long)hw_ts.tv_nsec);

        switch (msg_type)
        {

        case PTP_MSG_SYNC:
            handle_sync(eth_frame + 14, eth_len - 14, &hw_ts,
                        eth_frame + 6 /* src MAC */);
            break;

        case PTP_MSG_FOLLOW_UP:
            handle_follow_up(eth_frame + 14, eth_len - 14);
            break;

        case PTP_MSG_DELAY_RESP:
            handle_delay_resp(eth_frame + 14, eth_len - 14);
            break;

        case PTP_MSG_ANNOUNCE:
            handle_announce(eth_frame + 14, eth_len - 14);
            break;

        default:
            break;
        }

    timeout_check:

        /* If we haven't gotten a fresh Sync exchange in 5+ seconds,
         * drop out of LOCKED — the master is gone or unreachable. */
        if (g_slave_locked)
        {
            int64_t now_ms = esp_timer_get_time() / 1000;
            if ((now_ms - g_slave_last_sync_ms) > 5000)
            {
                g_slave_locked = false;
                g_slave_stable_count = 0;
                ESP_LOGW("SLAVE", "Master timeout — dropping lock");
                led_set_state(LED_STATE_ACQUIRING);
            }
        }

        /* Announce timeout — the check this label always promised. If no
         * Announce arrives for SLAVE_ANNOUNCE_TIMEOUT_S, the stored GM
         * properties are stale: invalidate them so gm_traceable evaluates
         * false and slave_compute_and_apply withholds/drops LOCK instead
         * of trusting a frozen clockClass. Re-arms on the next Announce
         * (handle_announce sets valid=true and g_last_announce_rx). */
        if (g_last_announce_rx != 0 &&
            (time(NULL) - g_last_announce_rx) > SLAVE_ANNOUNCE_TIMEOUT_S)
        {
            portENTER_CRITICAL(&g_gm_props_lock);
            bool was_valid = g_gm_props.valid;
            g_gm_props.valid = false;
            portEXIT_CRITICAL(&g_gm_props_lock);
            g_last_announce_rx = 0; /* one-shot until the next Announce */
            if (was_valid)
                ESP_LOGW("SLAVE",
                         "Announce timeout — GM properties stale, lock withheld");
        }
        // ESP_LOGW("PTP_RX_TASK", "PTP RX task is working");
        /* slave never reclaims master role — no action needed */
    } /* end of while(1) */
} /* end of ptp_rx_task */

/* B3+D4: Consumer side. Reuses the old "ptp_tx_task" slot — the
 * vestigial body was `vTaskDelete(NULL);`. Drains the delay_req queue
 * and calls send_delay_req() in its own context, so the RX hot path is
 * never blocked by an L2TAP write. The task self-gates on the queue's
 * blocking xQueueReceive, so zero CPU between Delay_Req sends. */
static void delay_req_task(void *arg)
{
    /* Wait until L2TAP is ready before we try to send anything. */
    while (!atomic_load(&g_l2tap_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    while (1)
    {
        uint8_t marker;
        if (xQueueReceive(g_delay_req_q, &marker, portMAX_DELAY) == pdTRUE)
        {
            send_delay_req();
        }
    }
}

static void gm_diag_task(void *arg)
{
    while (!atomic_load(&g_l2tap_ready))
        vTaskDelay(pdMS_TO_TICKS(500));

    /* Let the master speak a few times before first report */
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* Snapshot buffer lives in .bss — the struct is ~2.6 KB and putting it
     * on the stack along with newlib's vfprintf working set blows the task
     * stack. Only one gm_diag_task instance exists, so static is fine. */
    static gm_diag_t snap;

    while (1)
    {
        portENTER_CRITICAL(&g_gm_diag_lock);
        snap = g_gm_diag;
        portEXIT_CRITICAL(&g_gm_diag_lock);

        double off_mean, off_sd;
        int64_t off_min, off_max;
        compute_stats_i64(snap.offset_window, snap.offset_count,
                          &off_mean, &off_sd, &off_min, &off_max);

        /* B6: Populate g_phc_offset_ns with the slave's measured PHC
         * residual = rolling mean of recent offsets. Web status page
         * surfaces this as the "PHC residual" reading; the master writes
         * a structurally identical value in servo_task. */
        if (snap.offset_count > 0)
            g_phc_offset_ns = (int64_t)off_mean;

        double dly_mean, dly_sd;
        int64_t dly_min, dly_max;
        compute_stats_i64(snap.delay_window, snap.delay_count,
                          &dly_mean, &dly_sd, &dly_min, &dly_max);

        double si_mean, si_sd;
        int64_t si_min, si_max;
        compute_stats_i64(snap.sync_interval_window, snap.sync_int_count,
                          &si_mean, &si_sd, &si_min, &si_max);

        double fu_mean, fu_sd;
        int64_t fu_min, fu_max;
        compute_stats_i64(snap.followup_latency_window, snap.fu_lat_count,
                          &fu_mean, &fu_sd, &fu_min, &fu_max);

        int64_t now_us = esp_timer_get_time();
        int64_t since_sync_ms =
            snap.last_sync_rx_us == 0 ? -1 : (now_us - snap.last_sync_rx_us) / 1000;

        ESP_LOGI("GM_DIAG", "================ GM Quality Report ================");

        if (snap.sync_rx_count == 0)
        {
            ESP_LOGW("GM_DIAG", "No Sync messages received — check link / VLAN / GM running");
            ESP_LOGI("GM_DIAG", "Announces RX: %lu (gaps: %lu)",
                     (unsigned long)snap.announce_rx_count,
                     (unsigned long)snap.announce_seq_gaps);
            ESP_LOGI("GM_DIAG", "===================================================");
            vTaskDelay(pdMS_TO_TICKS(GM_DIAG_INTERVAL_MS));
            continue;
        }

        if (snap.gm_id_known)
        {
            ESP_LOGI("GM_DIAG",
                     "GM clockID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x  changes=%lu",
                     snap.current_gm_id[0], snap.current_gm_id[1],
                     snap.current_gm_id[2], snap.current_gm_id[3],
                     snap.current_gm_id[4], snap.current_gm_id[5],
                     snap.current_gm_id[6], snap.current_gm_id[7],
                     (unsigned long)snap.gm_identity_changes);
        }

        ESP_LOGI("GM_DIAG", "Lock: %s   last Sync %lld ms ago",
                 g_slave_locked ? "LOCKED" : "ACQUIRING",
                 (long long)since_sync_ms);

        if (snap.offset_count > 0)
            ESP_LOGI("GM_DIAG",
                     "Offset ns:  mean=%+.0f stddev=%.0f min=%+lld max=%+lld p2p=%lld n=%d",
                     off_mean, off_sd,
                     (long long)off_min, (long long)off_max,
                     (long long)(off_max - off_min),
                     snap.offset_count);
        else
            ESP_LOGI("GM_DIAG", "Offset ns: (no samples — DelayResp missing?)");

        if (snap.delay_count > 0)
            ESP_LOGI("GM_DIAG",
                     "Path delay: mean=%lld ns stddev=%lld ns min=%lld max=%lld",
                     (long long)dly_mean, (long long)dly_sd,
                     (long long)dly_min, (long long)dly_max);

        if (snap.sync_int_count > 0)
            ESP_LOGI("GM_DIAG",
                     "Sync intvl: mean=%.0f us stddev=%.0f us min=%lld max=%lld",
                     si_mean, si_sd,
                     (long long)si_min, (long long)si_max);

        if (snap.fu_lat_count > 0)
            ESP_LOGI("GM_DIAG",
                     "FU latency: mean=%.0f us stddev=%.0f us min=%lld max=%lld",
                     fu_mean, fu_sd,
                     (long long)fu_min, (long long)fu_max);

        ESP_LOGI("GM_DIAG",
                 "Counts: Sync=%lu(gaps=%lu) FU=%lu(orph=%lu) DResp=%lu(orph=%lu) Ann=%lu(gaps=%lu)",
                 (unsigned long)snap.sync_rx_count,
                 (unsigned long)snap.sync_seq_gaps,
                 (unsigned long)snap.followup_rx_count,
                 (unsigned long)snap.followup_orphans,
                 (unsigned long)snap.delay_resp_rx_count,
                 (unsigned long)snap.delay_resp_orphans,
                 (unsigned long)snap.announce_rx_count,
                 (unsigned long)snap.announce_seq_gaps);

        ESP_LOGI("GM_DIAG", "Sync mode: two-step=%lu  one-step=%lu  post-lock-spikes>1us=%lu",
                 (unsigned long)snap.two_step_count,
                 (unsigned long)snap.one_step_count,
                 (unsigned long)snap.post_lock_spikes);

        /* ── Slave-bonus diagnostics: asymmetry + GM Announce properties ── */
        {
            /* Forward / reverse half-trip means.
             *
             * On a symmetric link with a locked slave:
             *   mean(ms→sl) ≈ mean(sl→ms) ≈ path_delay
             *
             * If these differ persistently, the link is asymmetric and the
             * offset estimate is biased by approximately half the difference. */
            int asy_n;
            int64_t mssl_buf[ASYMMETRY_WINDOW];
            int64_t slms_buf[ASYMMETRY_WINDOW];
            portENTER_CRITICAL(&g_asymm_lock);
            asy_n = g_asymm_count;
            memcpy(mssl_buf, g_ms_to_sl_window, sizeof(int64_t) * asy_n);
            memcpy(slms_buf, g_sl_to_ms_window, sizeof(int64_t) * asy_n);
            portEXIT_CRITICAL(&g_asymm_lock);

            if (asy_n > 0)
            {
                double sum_a = 0, sum_b = 0;
                for (int i = 0; i < asy_n; i++)
                {
                    sum_a += (double)mssl_buf[i];
                    sum_b += (double)slms_buf[i];
                }
                double mean_a = sum_a / asy_n;
                double mean_b = sum_b / asy_n;
                double diff = mean_a - mean_b;
                ESP_LOGI("GM_DIAG",
                         "Half-trips: m->s mean=%.0f ns,  s->m mean=%.0f ns,  "
                         "diff=%+.0f ns (bias≈%+.0f ns)  n=%d",
                         mean_a, mean_b, diff, diff / 2.0, asy_n);

                /* Warn if the means differ enough to matter — but only
                 * when the offset window is genuinely settled. During
                 * acquisition the offset mean is large (tens of µs), and
                 * because (ms→sl mean) − (sl→ms mean) reduces to
                 * 2 × offset_mean by PTP arithmetic, the diff will look
                 * "asymmetric" purely as an artifact of the moving offset.
                 *
                 * Gate: require the offset window to be full AND offset
                 * stddev to be small (< 1 µs). When both hold, the window
                 * contains only post-lock samples and the diff reflects a
                 * true persistent path asymmetry rather than transient
                 * acquisition residue. */
                double abs_diff = diff < 0 ? -diff : diff;
                bool window_settled =
                    (snap.offset_count >= GM_DIAG_WINDOW) &&
                    (off_sd < 1000.0);
                if (asy_n >= 16 && window_settled && abs_diff > 1000.0)
                    ESP_LOGW("GM_DIAG",
                             "→ Path is asymmetric: %.0f ns "
                             "diff in half-trip means biases offset by ~%.0f ns.",
                             diff, diff / 2.0);
            }

            /* GM Announce-derived properties */
            gm_props_t gp;
            portENTER_CRITICAL(&g_gm_props_lock);
            gp = g_gm_props;
            portEXIT_CRITICAL(&g_gm_props_lock);

            if (gp.valid)
            {
                ESP_LOGI("GM_DIAG",
                         "GM Announce: class=%u accuracy=0x%02X UTC=%+d "
                         "pri1=%u pri2=%u src=0x%02X flags=0x%04X class_trans=%lu",
                         (unsigned)gp.clock_class,
                         (unsigned)gp.accuracy,
                         (int)gp.utc_offset,
                         (unsigned)gp.priority1,
                         (unsigned)gp.priority2,
                         (unsigned)gp.time_source,
                         (unsigned)gp.announce_flags,
                         (unsigned long)gp.class_transition_count);

                /* clockClass 7 = "HOLDOVER" per IEEE 1588-2008 Table 5.
                 * If the GM advertises class 7 we should NOT trust its
                 * time as fully traceable — it's free-running on local
                 * crystal, possibly drifting at tens of µs/min. */
                if (gp.clock_class >= 7 && gp.clock_class < 248)
                    ESP_LOGW("GM_DIAG",
                             "→ GM advertising clockClass %u (HOLDOVER). "
                             "Time is no longer fully traceable.",
                             (unsigned)gp.clock_class);
                /* The first transition is the normal cold-boot HOLDOVER→LOCKED
                 * settle (class 7 → 6) as the GM's own GNSS servo acquires.
                 * Only warn when the GM has flapped more than once, which
                 * indicates it is genuinely dropping in and out of holdover. */
                if (gp.class_transition_count > 1)
                    ESP_LOGW("GM_DIAG",
                             "→ GM class has transitioned %lu time(s) — "
                             "master is dropping in and out of holdover.",
                             (unsigned long)gp.class_transition_count);
            }
        }

        /* Verdict heuristics — translate numbers into something actionable */
        if (snap.sync_seq_gaps > 0)
            ESP_LOGW("GM_DIAG", "→ Sync packet loss (%lu gaps). Check switch QoS / network load.",
                     (unsigned long)snap.sync_seq_gaps);
        if (snap.gm_identity_changes > 0)
            ESP_LOGW("GM_DIAG", "→ GM identity changed %lu time(s). Multiple GMs on the LAN?",
                     (unsigned long)snap.gm_identity_changes);
        /* Only flag steady-state jitter — pre-lock samples pollute the window
         * for the first ~minute after acquisition, and complaining about them
         * is just noise. Require: locked, full window, and last Sync recent. */
        if (g_slave_locked && snap.offset_count >= GM_DIAG_WINDOW &&
            since_sync_ms >= 0 && since_sync_ms < 3000 &&
            off_sd > 5000)
            ESP_LOGW("GM_DIAG", "→ High offset jitter (%.0f ns stddev). GM clock noisy or congested network.",
                     off_sd);
        if (snap.delay_count >= 4 && dly_sd > 5000)
            ESP_LOGW("GM_DIAG", "→ Unstable path delay (%lld ns stddev). Network asymmetry / queueing.",
                     (long long)dly_sd);
        if (snap.fu_lat_count >= 4 && fu_mean > 50000)
            ESP_LOGW("GM_DIAG", "→ Slow FollowUp (%.0f us). GM CPU-bound or has poor PTP stack.",
                     fu_mean);
        if (since_sync_ms > 3000)
            ESP_LOGW("GM_DIAG", "→ No recent Sync (%lld ms). Master gone?", (long long)since_sync_ms);

        ESP_LOGI("GM_DIAG", "===================================================");
        vTaskDelay(pdMS_TO_TICKS(GM_DIAG_INTERVAL_MS));
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 12: Health monitoring
 *
 * Self-healing infrastructure: heap watchdog, EMAC restart with
 * PHC preservation, watchdog task. The watchdog uses
 * emac_get_time() (not g_pps_capture_ns) because the slave has no
 * PPS source.
 * ═════════════════════════════════════════════════════════════════════════ */

static void heap_monitor_task(void *arg)
{
    /* Wait a bit for system to settle before first measurement */
    vTaskDelay(pdMS_TO_TICKS(10000));

    size_t baseline_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI("HEAP", "Baseline free internal heap: %u bytes",
             (unsigned)baseline_free);

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

        vTaskDelay(pdMS_TO_TICKS(60000));
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

    /* ── Step 1: Snapshot PHC and esp_timer simultaneously ──
     * Read PHC first, then esp_timer immediately. Keep these two
     * statements adjacent — any code between them adds capture error.
     * Disable interrupts to ensure no preemption between samples. */
    struct timespec ts_before;
    int64_t mono_before_us;

    portMUX_TYPE capture_lock = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&capture_lock);
    emac_get_time(&ts_before);
    mono_before_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&capture_lock);

    int64_t phc_ns_before = (int64_t)ts_before.tv_sec * 1000000000LL +
                            (int64_t)ts_before.tv_nsec;

    /* Snapshot the current frequency adjustment so we can re-apply it.
     * g_slave_freq_ppb is updated by slave_servo_freq; spinlock-protected read. */
    taskENTER_CRITICAL(&g_servo_spinlock);
    double saved_freq_ppb = g_slave_freq_ppb;
    taskEXIT_CRITICAL(&g_servo_spinlock);

    ESP_LOGI("WDT", "PHC snapshot: %lld.%09ld (mono=%lld us, freq_ppb=%.0f)",
             (long long)ts_before.tv_sec,
             (long)ts_before.tv_nsec,
             (long long)mono_before_us,
             saved_freq_ppb);

    /* A6: Two-stage gating against in-flight L2TAP TX and PHC ioctls.
     *   (1) Set g_emac_restarting so new l2tap_send_ptp / S_PTP_TIME /
     *       ADJ_PTP_TIME callers fast-reject instead of touching the EMAC
     *       while we're stopping it.
     *   (2) Take g_l2tap_tx_mtx so any in-flight L2TAP write that already
     *       passed the atomic check has drained before esp_eth_stop runs.
     * The mutex is also held during start, so the early returns below
     * MUST reach cleanup_unlock so the mutex never leaks. */
    atomic_store(&g_emac_restarting, true);
    if (g_l2tap_tx_mtx)
        xSemaphoreTake(g_l2tap_tx_mtx, portMAX_DELAY);

    /* ── Step 2: Stop driver ── */
    esp_err_t err = esp_eth_stop(g_eth_hndl);
    if (err != ESP_OK)
    {
        ESP_LOGE("WDT", "esp_eth_stop failed: %s", esp_err_to_name(err));
        goto cleanup_unlock;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* ── Step 3: Restart driver ── */
    err = esp_eth_start(g_eth_hndl);
    if (err != ESP_OK)
    {
        ESP_LOGE("WDT", "esp_eth_start failed: %s", esp_err_to_name(err));
        goto cleanup_unlock;
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
     * Read esp_timer as late as possible — right before the ioctl write —
     * to minimize gap between our extrapolation and the write taking effect. */
    int64_t mono_after_us = esp_timer_get_time();
    int64_t dt_us = mono_after_us - mono_before_us;
    int64_t dt_ns = dt_us * 1000LL;

    /* Apply frequency correction. saved_freq_ppb is in ppb (1e-9 units). */
    int64_t freq_correction_ns = (int64_t)((double)dt_ns * saved_freq_ppb * 1e-9);
    int64_t target_ns = phc_ns_before + dt_ns + freq_correction_ns;

    if (target_ns < 0)
    {
        ESP_LOGE("WDT", "Computed negative target time, aborting");
        goto cleanup_unlock;
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
        goto cleanup_unlock;
    }

    /* ── Step 6: Re-apply frequency adjustment ── */
    int32_t adj_ppb = (int32_t)saved_freq_ppb;
    esp_eth_ioctl(g_eth_hndl, ETH_MAC_ESP_CMD_ADJ_PTP_TIME, &adj_ppb);

    g_emac_restart_count++;

    ESP_LOGW("WDT",
             "EMAC restart complete: dt=%lld us, freq_corr=%lld ns, target=%u.%09u",
             (long long)dt_us,
             (long long)freq_correction_ns,
             t.seconds, t.nanoseconds);

cleanup_unlock:
    /* A6: Release the mutex BEFORE clearing the atomic flag, so a caller
     * waiting on the mutex sees g_emac_restarting=true until the moment
     * it actually acquires; mirrors master line 3317. */
    if (g_l2tap_tx_mtx)
        xSemaphoreGive(g_l2tap_tx_mtx);
    atomic_store(&g_emac_restarting, false);
}

static void watchdog_task(void *arg)
{
    /* Wait 60 s for system to fully come up before arming */
    vTaskDelay(pdMS_TO_TICKS(60000));

    /* Slave PHC freshness source.
     *
     * The master reads g_pps_capture_ns, which its PPS ISR updates every
     * second. The slave has no PPS source, so we sample the PHC directly
     * via emac_get_time(). The hardware PTP register is updated by the
     * EMAC; if the EMAC RX descriptor pool is exhausted, the EMAC stalls
     * and the PHC counter stops advancing in lockstep — exactly the
     * failure mode this watchdog targets.
     *
     * Read returns 0 if the PHC isn't running yet; treat that as "not
     * ready" and don't sample until it advances at least once. */
    struct timespec ts_now = {0};
    emac_get_time(&ts_now);
    g_last_progress_phc_s = (uint64_t)ts_now.tv_sec;
    g_last_progress_check_ms = esp_timer_get_time() / 1000;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(30000));

        uint64_t now_ms = esp_timer_get_time() / 1000;
        emac_get_time(&ts_now);
        uint64_t cur_phc_s = (uint64_t)ts_now.tv_sec;

        bool phc_frozen = (cur_phc_s == g_last_progress_phc_s);
        uint64_t elapsed = now_ms - g_last_progress_check_ms;

        /* ───── Check 1: PHC freeze ─────
         * The PHC seconds counter should advance by ~30 between checks.
         * If it doesn't advance at all over a 30 s wall-clock window, the
         * EMAC RX path is most likely jammed. PHC keeping up while link
         * is down is fine — link state is checked separately by the LED
         * watch task. */
        if (phc_frozen && elapsed > 30000)
        {
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
                emac_get_time(&ts_now);
                g_last_progress_phc_s = (uint64_t)ts_now.tv_sec;
                g_last_progress_check_ms = esp_timer_get_time() / 1000;
            }
            else if (g_emac_restart_count >= 10)
            {
                ESP_LOGE("WDT",
                         "EMAC restart limit reached — system needs reboot");
            }
        }
        else if (!phc_frozen)
        {
            g_last_progress_phc_s = cur_phc_s;
            g_last_progress_check_ms = now_ms;
        }

        /* ───── Check 2: Heap pressure ─────
         * Should not trigger under normal operation. If it does, there is
         * a leak somewhere — try a restart anyway in case it clears lwIP
         * buffers. */
        size_t free_now = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

        if (free_now < 50000)
        {
            ESP_LOGE("WDT",
                     "Critical heap pressure (%u bytes free) — attempting recovery",
                     (unsigned)free_now);

            if (g_emac_restart_count < 10)
            {
                emac_restart_preserve_phc();
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 13: LED status indicator
 *
 * Single status LED driven by s_led_state, plus a secondary LED
 * that pulses on every received Sync (heartbeat). slave_led_watch_task
 * transitions between LOCKED / HOLDOVER / FAULT based on Sync
 * freshness.
 * ═════════════════════════════════════════════════════════════════════════ */

// ============================================================================
// LED status indicator
// ----------------------------------------------------------------------------
// Single GPIO LED, low-priority task, drives blink patterns from servo state.
//
// Patterns:
//   Solid off  : no Ethernet link (cable unplugged, PHY down)
//   Slow blink : link up, but not yet locked (no GNSS, or servo acquiring)
//   Solid on   : LOCKED — time output is trustworthy
//   Heartbeat  : HOLDOVER — was locked, lost GNSS, still serving (degrading)
//   Fast blink : FAULT — something broken, check logs
//
// The task reads a shared atomic state variable. It never blocks the servo,
// PPS handler, or RX/TX paths. A 10 ms tick gives clean pattern timing
// without measurable CPU cost.
// ============================================================================

static void led_write(bool on)
{
    gpio_set_level(LED_GPIO, LED_ACTIVE_HIGH ? on : !on);
}

// Public API: any task/ISR-safe context can call this.
void led_set_state(led_state_t new_state)
{
    atomic_store(&s_led_state, new_state);
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
    xTaskCreate(led_task, "led", LED_TASK_STACK, NULL, LED_TASK_PRIO, NULL);
}

/* On the slave, the red LED (formerly PPS heartbeat on the master) pulses on
 * every received PTP Sync message. Same hardware, same visual rhythm, but
 * the meaning is "I'm hearing the grandmaster." If the LED stops pulsing,
 * the master heartbeat is gone.
 *
 * Sync arrival is signalled by `g_sync_rx_pulse_seq`, incremented from
 * `handle_sync`. The pps_led_task name is preserved to avoid disturbing
 * existing references; functionally it is now a generic ~1 Hz heartbeat. */
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

    ESP_LOGI("PPS_LED", "task started on GPIO %d (pulse=%d ms) — Sync RX heartbeat",
             PPS_LED_GPIO, PPS_LED_ON_MS);

    uint32_t last_seen_seq = 0;

    while (1)
    {
        /* Watch the Sync RX counter; the GM's PPS-capture counter is no
         * longer authoritative on the slave (PPS GPIO is unused here). */
        uint32_t now_seq = atomic_load(&g_sync_rx_pulse_seq);

        if (now_seq != last_seen_seq)
        {
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

/* ─────────────────────────────────────────────────────────────────────────
 * slave_led_watch_task
 *
 * The slave's headline state machine for the green LED:
 *
 *   ACQUIRING ─(lock acquired)─→ LOCKED ─(no Sync for 5 s)─→ HOLDOVER
 *                  ↑                                            │
 *                  └──────────(Sync resumes)─────────────────────┘
 *                                                              │
 *                                                       (no Sync for 60 s)
 *                                                              ↓
 *                                                           FAULT
 *
 * handle_sync / slave_compute_and_apply manage ACQUIRING↔LOCKED via
 * led_set_state(). This task only handles the LOCKED→HOLDOVER→FAULT and
 * HOLDOVER→ACQUIRING transitions, which are time-based and need a periodic
 * checker (no event fires on the *absence* of Sync).
 * ───────────────────────────────────────────────────────────────────────── */
static void slave_led_watch_task(void *arg)
{
    /* Wait for L2TAP so we don't churn the LED state while the stack is up. */
    while (!atomic_load(&g_l2tap_ready))
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Establish baseline so we don't immediately flag "Sync gone" before
     * we ever heard one. */
    int64_t last_seen_ms = g_last_sync_ms;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t cur_sync_ms = g_last_sync_ms;
        led_state_t s = atomic_load(&s_led_state);

        if (cur_sync_ms == 0)
        {
            /* Never heard a Sync since boot — leave ACQUIRING/NO_LINK alone. */
            continue;
        }

        int64_t age_ms = now_ms - cur_sync_ms;

        /* Detect resumption of Sync flow after silence. The slave_compute path
         * will manage LOCKED transitions; here we just step out of HOLDOVER /
         * FAULT back into ACQUIRING when Sync returns. */
        if (cur_sync_ms != last_seen_ms)
        {
            if (s == LED_STATE_HOLDOVER || s == LED_STATE_FAULT)
            {
                ESP_LOGI("LED", "Sync resumed after silence — back to ACQUIRING");
                led_set_state(LED_STATE_ACQUIRING);
            }
            last_seen_ms = cur_sync_ms;
        }

        /* B7: Lock loss → drifting (≥5 s without a Sync, while
         * previously LOCKED). Gate on g_slave_ever_locked so a slave
         * that has *never* achieved lock (cold boot, GM still in
         * holdover) does not enter HOLDOVER. "Drifting" requires
         * something to drift from. */
        if (s == LED_STATE_LOCKED && g_slave_ever_locked && age_ms > 5000)
        {
            ESP_LOGW("LED", "No Sync for %lld ms — entering DRIFTING (heartbeat)",
                     (long long)age_ms);
            led_set_state(LED_STATE_HOLDOVER);
        }
        /* Drifting too long → fault */
        else if (s == LED_STATE_HOLDOVER && age_ms > DRIFTING_TO_FAULT_MS)
        {
            ESP_LOGE("LED", "No Sync for %lld ms — FAULT (master truly gone)",
                     (long long)age_ms);
            led_set_state(LED_STATE_FAULT);
        }
    }
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 14: OLED display + button
 *
 * Four rotating screens driven by DISPLAY_NUM_SCREENS:
 *   1. Main — lock state, offset, path delay, sync-rate label
 *   2. Servo — lock duration, offset/delay stats, freq & drift slope,
 *              two-band Q100/Q1k quality, asymmetry
 *   3. Network — link + IP, hostname, uptime + heap
 *   4. Grandmaster — GM MAC + hops, BMCA fields, time source, flow health
 * A debounced button cycles through them. The boot splash includes
 * the firmware version read from version.txt via
 * esp_app_get_description().
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
    u8g2_DrawStr(&u8g2, 0, 12, "PTP Slave");

    /* Device name on two rows (worst case 13 + 17 chars, always fits
     * the 21-column budget). Falls back to "ESP32-P4" only if the OLED
     * task ever beats ethernet_init to the buffer (it can't in the
     * current app_main ordering — display_task delays 2 s before
     * drawing — but the guard costs nothing). */
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

    while (1)
    {
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
            render_screen_network();
            break;
        case 3:
            render_screen_gm();
            break;
        default:
            render_screen_main();
            break;
        }

        /* Tiny screen-number indicator in bottom-right corner. The
         * firmware version is shown on the boot splash and on every
         * web page; not re-drawn here because the bottom row is
         * already used by each screen's own status line (GM, asymmetry,
         * etc.). */
        u8g2_SetFont(&u8g2, u8g2_font_4x6_tf);
        char idx[8];
        snprintf(idx, sizeof(idx), "%u/%u", (unsigned)(screen + 1),
                 (unsigned)DISPLAY_NUM_SCREENS);
        u8g2_DrawStr(&u8g2, 128 - 16, 64, idx);

        u8g2_SendBuffer(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * Slave Screen 1: Main status — what users glance at most
 *   - Lock state (big font)
 *   - Current offset (ns)
 *   - Path delay (ns)
 *   - Sync rate indicator
 *   - GM clockID (last 4 bytes) — which master we're tracking
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_main(void)
{
    /* Snapshot state */
    led_state_t state = atomic_load(&s_led_state);
    int64_t offset = g_slave_offset_ns;
    int64_t delay = g_slave_path_delay_ns;
    bool locked = g_slave_locked;
    int64_t last_sync = g_last_sync_ms;
    int64_t now_ms = esp_timer_get_time() / 1000;

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
        state_str = locked ? "LOCKED" : "ACQUIRING";
        break;
    case LED_STATE_LOCKED:
        state_str = "LOCKED";
        break;
    case LED_STATE_HOLDOVER:
        state_str = "DRIFTING";
        break;
    case LED_STATE_FAULT:
        state_str = "FAULT";
        break;
    }
    u8g2_DrawStr(&u8g2, 0, 12, state_str);

    /* Separator line */
    u8g2_DrawHLine(&u8g2, 0, 16, 128);

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

    /* Offset */
    if (locked || state == LED_STATE_HOLDOVER)
    {
        if (llabs(offset) < 10000LL)
            snprintf(buf, sizeof(buf), "Off: %+lld ns", (long long)offset);
        else
            snprintf(buf, sizeof(buf), "Off: %+lld us", (long long)(offset / 1000));
    }
    else
    {
        snprintf(buf, sizeof(buf), "Off: ---");
    }
    u8g2_DrawStr(&u8g2, 0, 28, buf);

    /* Path delay */
    if (last_sync != 0)
        snprintf(buf, sizeof(buf), "Delay: %lld ns", (long long)delay);
    else
        snprintf(buf, sizeof(buf), "Delay: ---");
    u8g2_DrawStr(&u8g2, 0, 40, buf);

    /* Sync rate indicator.
     *
     * Label reflects the GM's advertised logSyncInterval (from the Sync
     * header, captured in handle_sync). Stale threshold scales with the
     * advertised interval — 2× interval, floored at 1500 ms so a 1/s
     * link isn't declared stale on a single dropped packet.
     *
     *   logSyncInterval interpretation:
     *     -7 →  128/s        ("128/s")
     *     -1 →    2/s        ("2/s")
     *      0 →    1/s        ("1/s")
     *      1 →  1 per 2 s    ("1/2s")
     *      4 →  1 per 16 s   ("1/16s")
     *
     * If we haven't seen a Sync yet (sentinel value), we still show
     * "waiting" — falling back to the 2 s threshold used previously so
     * the "stale" wording doesn't fire during the initial startup gap. */
    if (last_sync == 0)
    {
        snprintf(buf, sizeof(buf), "Sync: --- (waiting)");
    }
    else
    {
        int8_t log_si = atomic_load(&g_gm_log_sync_interval);

        /* Build a short rate label ("N/s" or "1/Ns") plus the expected
         * interval in ms for the stale check. rate_lbl is sized for
         * gcc's worst-case %u analysis (10-digit uint32) plus the
         * "1/" and "s" literals — realistic PTP values are 1-3 digits,
         * but sizing to gcc's bound keeps -Werror=format-truncation
         * quiet without changing runtime. */
        char rate_lbl[16];
        int64_t interval_ms;
        if (log_si == GM_LOG_SYNC_INTERVAL_UNKNOWN)
        {
            snprintf(rate_lbl, sizeof(rate_lbl), "?");
            interval_ms = 1000; /* neutral default while we wait */
        }
        else if (log_si < 0)
        {
            /* Faster than 1 Hz: rate_hz = 2^(-log_si) */
            unsigned hz = 1u << (-log_si);
            snprintf(rate_lbl, sizeof(rate_lbl), "%u/s", hz);
            interval_ms = 1000 / (int64_t)hz;
            if (interval_ms < 1)
                interval_ms = 1;
        }
        else if (log_si == 0)
        {
            snprintf(rate_lbl, sizeof(rate_lbl), "1/s");
            interval_ms = 1000;
        }
        else
        {
            /* Slower than 1 Hz: period_s = 2^log_si */
            unsigned period_s = 1u << log_si;
            snprintf(rate_lbl, sizeof(rate_lbl), "1/%us", period_s);
            interval_ms = 1000LL * (int64_t)period_s;
        }

        /* Stale = 2× advertised interval, floored at 1500 ms so a 1 Hz
         * link isn't declared stale on a single missed Sync. */
        int64_t stale_ms = 2 * interval_ms;
        if (stale_ms < 1500)
            stale_ms = 1500;

        int64_t age_ms = now_ms - last_sync;
        if (age_ms < stale_ms)
        {
            snprintf(buf, sizeof(buf), "Sync: %s OK", rate_lbl);
        }
        else
        {
            /* Clamp the printed age to 4 digits (9999 s ≈ 2.8 h).
             * Anything staler is functionally "the GM is gone"; the
             * clamp keeps the row inside the 21-char OLED budget
             * ("Sync: 1/16s stl 9999s" = exactly 21 chars) and
             * satisfies -Wformat-truncation. */
            int64_t age_s_64 = age_ms / 1000;
            if (age_s_64 > 9999)
                age_s_64 = 9999;
            snprintf(buf, sizeof(buf), "Sync: %s stl %ds",
                     rate_lbl, (int)age_s_64);
        }
    }
    u8g2_DrawStr(&u8g2, 0, 52, buf);

    /* Bottom line: GM clockID short form (last 4 bytes) */
    if (g_master_mac_known)
    {
        snprintf(buf, sizeof(buf), "GM ..%02X:%02X:%02X:%02X",
                 g_master_mac[2], g_master_mac[3],
                 g_master_mac[4], g_master_mac[5]);
    }
    else
    {
        snprintf(buf, sizeof(buf), "GM: ---");
    }
    u8g2_DrawStr(&u8g2, 0, 63, buf);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Slave Screen 2: Servo / quality — engineer's view
 *   - Lock duration / drifting age
 *   - Offset mean & stddev (live, last window)
 *   - Path delay mean & stddev
 *   - Quality bar: % of last window within ±100 ns
 *   - Frequency adjustment (ppb)
 *   - Linreg slope (GM apparent drift rate vs slave PHC)
 *   - Asymmetry mean
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_servo(void)
{
    led_state_t state = atomic_load(&s_led_state);
    double freq_ppb = g_slave_freq_ppb;
    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t lock_start = g_lock_acquired_ms;
    int64_t last_sync = g_last_sync_ms;
    bool locked = g_slave_locked;

    /* Take a thread-safe snapshot of the offset / delay windows we already
     * accumulate for the GM diagnostics — they ARE the live quality data. */
    int off_n, dly_n;
    int64_t off_buf[GM_DIAG_WINDOW];
    int64_t dly_buf[GM_DIAG_WINDOW];

    portENTER_CRITICAL(&g_gm_diag_lock);
    off_n = g_gm_diag.offset_count;
    dly_n = g_gm_diag.delay_count;
    memcpy(off_buf, g_gm_diag.offset_window, sizeof(int64_t) * off_n);
    memcpy(dly_buf, g_gm_diag.delay_window, sizeof(int64_t) * dly_n);
    portEXIT_CRITICAL(&g_gm_diag_lock);

    double off_mean = 0, off_sd = 0, dly_mean = 0, dly_sd = 0;
    int64_t dummy_mn, dummy_mx;
    if (off_n > 0)
        compute_stats_i64(off_buf, off_n, &off_mean, &off_sd, &dummy_mn, &dummy_mx);
    if (dly_n > 0)
        compute_stats_i64(dly_buf, dly_n, &dly_mean, &dly_sd, &dummy_mn, &dummy_mx);

    /* Linreg slope = GM's apparent drift relative to our PHC, ppb */
    double slope_ppb = linreg_compute_slope_ppb();

    /* Asymmetry: difference of means between forward and reverse half-trips.
     * Half this value is the offset bias caused by link asymmetry. */
    int asy_n;
    int64_t mssl_buf[ASYMMETRY_WINDOW];
    int64_t slms_buf[ASYMMETRY_WINDOW];
    int64_t cm2s_sum = 0, cs2m_sum = 0;
    portENTER_CRITICAL(&g_asymm_lock);
    asy_n = g_asymm_count;
    memcpy(mssl_buf, g_ms_to_sl_window, sizeof(int64_t) * asy_n);
    memcpy(slms_buf, g_sl_to_ms_window, sizeof(int64_t) * asy_n);
    /* Sum in-place instead of copying two more 512 B buffers to this
     * task's stack; 64 adds hold the lock a few hundred ns longer. */
    for (int i = 0; i < asy_n; i++)
    {
        cm2s_sum += g_corr_m2s_window[i];
        cs2m_sum += g_corr_s2m_window[i];
    }
    portEXIT_CRITICAL(&g_asymm_lock);

    int64_t asy_diff = 0;
    if (asy_n > 0)
    {
        int64_t sum_a = 0, sum_b = 0;
        for (int i = 0; i < asy_n; i++)
        {
            sum_a += mssl_buf[i];
            sum_b += slms_buf[i];
        }
        asy_diff = (sum_a - sum_b) / asy_n;
    }

    /* Two-band quality: percentage of last-window offset samples inside
     * the TIGHT (±100 ns default) and LOOSE (±1 µs default) envelopes.
     * See SLAVE_QUALITY_THRESHOLD_* defines for the tunables. */
    int qual_tight = 0, qual_loose = 0;
    if (off_n > 0)
    {
        int good_tight = 0, good_loose = 0;
        for (int i = 0; i < off_n; i++)
        {
            int64_t a = llabs(off_buf[i]);
            if (a <= SLAVE_QUALITY_THRESHOLD_TIGHT_NS)
                good_tight++;
            if (a <= SLAVE_QUALITY_THRESHOLD_LOOSE_NS)
                good_loose++;
        }
        qual_tight = (good_tight * 100) / off_n;
        qual_loose = (good_loose * 100) / off_n;
    }

    char buf[32];

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "SERVO");
    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    /* Lock duration / drifting age.
     * Use the servo's lock flag (g_slave_locked) as the source of truth,
     * not the LED state — same convention Screen 1 uses, so the two
     * screens never disagree. */
    if (locked && lock_start > 0)
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
    else if (state == LED_STATE_HOLDOVER && last_sync > 0)
    {
        int64_t age_s = (now_ms - last_sync) / 1000;
        snprintf(buf, sizeof(buf), "Drifting: %llds", (long long)age_s);
    }
    else
    {
        snprintf(buf, sizeof(buf), "(not locked)");
    }
    u8g2_DrawStr(&u8g2, 0, 22, buf);

    /* Offset: mean/sd + two-band quality (Q tight / Q loose). Reads
     * as "Off +12/8ns Q95/99" — mean/sd in ns, then % samples inside
     * ±TIGHT / ±LOOSE. If the two numbers are close, most of the
     * residual noise is inside TIGHT; if TIGHT is small and LOOSE is
     * high, the path is noisier than HW-timestamping ideal but still
     * within the LOOSE envelope. */
    if (off_n > 0)
        snprintf(buf, sizeof(buf), "Off %+d/%dns Q%d/%d",
                 (int)off_mean, (int)off_sd, qual_tight, qual_loose);
    else
        snprintf(buf, sizeof(buf), "Off: --");
    u8g2_DrawStr(&u8g2, 0, 32, buf);

    /* Delay: mean±sd */
    if (dly_n > 0)
        snprintf(buf, sizeof(buf), "Dly %d ns sd%d", (int)dly_mean, (int)dly_sd);
    else
        snprintf(buf, sizeof(buf), "Dly: --");
    u8g2_DrawStr(&u8g2, 0, 42, buf);

    /* Frequency adjustment + linreg slope */
    snprintf(buf, sizeof(buf), "Adj %+dppb Sl%+d",
             (int)freq_ppb, (int)slope_ppb);
    u8g2_DrawStr(&u8g2, 0, 52, buf);

    /* Asymmetry + TC residence-time correction (windowed means).
     * These read together: TC m2s/s2m are the mean corrections stamped
     * into each leg (0/0 on a direct cable = decode sanity check;
     * non-zero behind a TC switch = the TC is matching + correcting),
     * and Asy is what remains AFTER those corrections were subtracted,
     * i.e. residual path asymmetry. Auto-scales to µs ("u") when the
     * residence means outgrow four digits. */
    if (asy_n > 0)
    {
        int64_t cm2s_mean = cm2s_sum / asy_n;
        int64_t cs2m_mean = cs2m_sum / asy_n;
        int64_t tc_max = (cm2s_mean > cs2m_mean) ? cm2s_mean : cs2m_mean;
        if (tc_max >= 10000 || tc_max <= -10000)
            snprintf(buf, sizeof(buf), "Asy%+d TC%.1f/%.1fu",
                     (int)asy_diff,
                     (double)cm2s_mean / 1000.0,
                     (double)cs2m_mean / 1000.0);
        else
            snprintf(buf, sizeof(buf), "Asy%+d TC%d/%d",
                     (int)asy_diff, (int)cm2s_mean, (int)cs2m_mean);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Asy --");
    }
    u8g2_DrawStr(&u8g2, 0, 62, buf);
}

/* ─────────────────────────────────────────────────────────────────────────
 * Slave Screen 3: Network — identity + connectivity (dedicated page,
 * mirrors the master's NETWORK screen so a Wireshark / router lookup
 * against the DHCP hostname finds the same string on both sides)
 *   - Link + IP (with '!' after UP for link-up-but-L2TAP-not-ready)
 *   - Our hostname (two rows, from mac_hostname_display_lines)
 *   - Uptime + free heap
 * GM info moved to the dedicated Screen 4 (render_screen_gm) below.
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

/* ─────────────────────────────────────────────────────────────────────────
 * Slave Screen 4: Grandmaster — designed around the four questions an
 * operator asks (in order) when they suspect a GM issue:
 *
 *   Row 1 — WHO:   GM MAC + stepsRemoved (BMCA hop count)
 *   Row 2 — WHY:   priority1 : priority2, clockClass, accuracy
 *                  (the full BMCA selection tuple, so "why did we pick
 *                  this GM" is answerable from the display alone)
 *   Row 3 — TRUST: timeSource (GPS/PTP/OSC/...) + UTC offset — the two
 *                  fields that say whether the GM is disciplined to a
 *                  primary reference or is free-running
 *   Row 4 — FLOW:  Sync freshness ("Ns ago") + Delay-path health
 *                  (delay_resp / sync ratio, %) + total protocol
 *                  anomalies. FLOW is the row that flags "silent GM"
 *                  (Sync stops), "one-way path" (D% collapses to 0),
 *                  and "network dropping frames" (! rises) — the three
 *                  most common live failures.
 *
 * All fields read via the same locks / atomics used elsewhere; no new
 * synchronization is introduced by this screen.
 * ───────────────────────────────────────────────────────────────────────── */
static void render_screen_gm(void)
{
    /* Snapshot GM props under the same spinlock other readers use. */
    gm_props_t gp;
    portENTER_CRITICAL(&g_gm_props_lock);
    gp = g_gm_props;
    portEXIT_CRITICAL(&g_gm_props_lock);

    /* Snapshot diag counters. Same lock used everywhere else that reads
     * g_gm_diag. Row 4 sums the four packet-level anomaly counters into
     * one "!" number so a single glance catches network / protocol
     * misbehaviour; the individual values remain in the gm_diag log and
     * on the web /api/status endpoint for triage. */
    uint32_t sync_cnt, dresp_cnt;
    uint32_t sync_gaps, ann_gaps, fu_orphans, dresp_orphans;
    portENTER_CRITICAL(&g_gm_diag_lock);
    sync_cnt = g_gm_diag.sync_rx_count;
    dresp_cnt = g_gm_diag.delay_resp_rx_count;
    sync_gaps = g_gm_diag.sync_seq_gaps;
    ann_gaps = g_gm_diag.announce_seq_gaps;
    fu_orphans = g_gm_diag.followup_orphans;
    dresp_orphans = g_gm_diag.delay_resp_orphans;
    portEXIT_CRITICAL(&g_gm_diag_lock);

    int64_t now_ms = esp_timer_get_time() / 1000;
    int64_t last_sync_ms = g_last_sync_ms; /* volatile 64-bit read; worst-
                                            * case tearing shows a stale
                                            * age of a few ms — invisible
                                            * at the display refresh rate */

    /* Row scratch buffer sized to gcc's worst-case
     * -Wformat-truncation analysis for the FLOW row
     * ("Sync %s %s !%u" with age_frag[12] + d_frag[10] + up-to-10-digit
     *  %u = 39 bytes). Realistic outputs stay ≤ 21 chars (see clamps
     * on hrs, D%, and anomalies below); the extra bytes just satisfy
     * the compiler without changing runtime. */
    char buf[48];

    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
    u8g2_DrawStr(&u8g2, 0, 10, "GRANDMASTER");
    u8g2_DrawHLine(&u8g2, 0, 12, 128);

    /* Row 1 — WHO: full GM MAC + stepsRemoved.
     * "GM AABBCCDDEEFF sR2" is 18 chars. sR=0 means the announcer IS
     * the grandmaster; sR>0 means we're that many boundary-clock hops
     * behind the actual GM — useful for topology sanity ("did BMCA
     * pick a nearby GM or a distant one?"). */
    if (g_master_mac_known)
    {
        if (gp.valid)
            snprintf(buf, sizeof(buf), "GM %02X%02X%02X%02X%02X%02X sR%u",
                     g_master_mac[0], g_master_mac[1], g_master_mac[2],
                     g_master_mac[3], g_master_mac[4], g_master_mac[5],
                     (unsigned)gp.steps_removed);
        else
            snprintf(buf, sizeof(buf), "GM %02X%02X%02X%02X%02X%02X",
                     g_master_mac[0], g_master_mac[1], g_master_mac[2],
                     g_master_mac[3], g_master_mac[4], g_master_mac[5]);
    }
    else
    {
        snprintf(buf, sizeof(buf), "GM: ---");
    }
    u8g2_DrawStr(&u8g2, 0, 24, buf);

    /* Row 2 — WHY (BMCA reasoning): priority1 : priority2, class,
     * accuracy. This is the full BMCA-comparable tuple minus variance
     * (which almost never differentiates in practice). Format is
     * "P128:128 Cls6 A21" (max 17 chars). Priorities are the primary
     * selectors; class/accuracy are the tiebreakers. */
    if (gp.valid)
    {
        snprintf(buf, sizeof(buf), "P%u:%u Cls%u A%02X",
                 (unsigned)gp.priority1, (unsigned)gp.priority2,
                 (unsigned)gp.clock_class, (unsigned)gp.accuracy);
    }
    else
    {
        snprintf(buf, sizeof(buf), "BMCA: --");
    }
    u8g2_DrawStr(&u8g2, 0, 35, buf);

    /* Row 3 — TRUST: what's driving the GM + UTC offset.
     * timeSource codes are from IEEE 1588-2008 §7.6.2.6. Anything other
     * than an external reference (GPS/ATOM/RAD) or another PTP source
     * means the GM is essentially free-running on its own oscillator —
     * accuracy claims should be read skeptically in that case. UTC
     * offset alongside is important for consumers that need wall-clock
     * time (e.g. a display driver combining PTP time with UTC leap
     * seconds). Format: "Src:GPS UTC+37" (max ~16 chars). */
    if (gp.valid)
    {
        const char *src;
        switch (gp.time_source)
        {
        case 0x10:
            src = "ATOM";
            break;
        case 0x20:
            src = "GPS";
            break;
        case 0x30:
            src = "RAD";
            break;
        case 0x40:
            src = "PTP";
            break;
        case 0x50:
            src = "NTP";
            break;
        case 0x60:
            src = "MAN";
            break;
        case 0x90:
            src = "OTH";
            break;
        case 0xA0:
            src = "OSC";
            break;
        default:
            src = "?";
            break;
        }
        snprintf(buf, sizeof(buf), "Src:%s UTC%+d",
                 src, (int)gp.utc_offset);
    }
    else
    {
        snprintf(buf, sizeof(buf), "Src: --");
    }
    u8g2_DrawStr(&u8g2, 0, 46, buf);

    /* Row 4 — FLOW: is the exchange healthy right now?
     *   Sync freshness:  how long since the most recent Sync (Ns / Nm / Nh).
     *   D%:              delay_resp count as % of sync count. In a
     *                    healthy 2-way exchange this stays near 100.
     *                    A collapse to 0 means the delay path broke
     *                    (multicast blocked in one direction, ACL,
     *                    GM not responding) — the failure mode the
     *                    old absolute-count view made invisible.
     *   !N:              aggregate packet-level anomalies (Sync gaps,
     *                    Announce gaps, FollowUp orphans, DelayResp
     *                    orphans). Any non-zero value is worth
     *                    investigating; per-counter breakdown lives
     *                    in the gm_diag log and the web endpoint.
     * Format: "Sync 2s D:98% !0" (max 20 chars at typical values). */
    {
        /* Sync-age fragment. age_frag oversized (worst realistic case
         * is "99999h" = 6 chars); the extra bytes keep
         * -Wformat-truncation quiet without changing runtime. */
        char age_frag[12];
        if (last_sync_ms == 0)
        {
            snprintf(age_frag, sizeof(age_frag), "--");
        }
        else
        {
            int64_t age_ms = now_ms - last_sync_ms;
            if (age_ms < 0)
                age_ms = 0;
            int64_t age_s = age_ms / 1000;
            if (age_s < 60)
            {
                snprintf(age_frag, sizeof(age_frag), "%llds",
                         (long long)age_s);
            }
            else if (age_s < 3600)
            {
                snprintf(age_frag, sizeof(age_frag), "%lldm",
                         (long long)(age_s / 60));
            }
            else
            {
                /* Clamp hours to 4 digits (9999 h ≈ 416 days);
                 * anything staler is functionally "GM long gone" and
                 * we don't want an unbounded %lld eating the row. */
                int64_t hrs = age_s / 3600;
                if (hrs > 9999)
                    hrs = 9999;
                snprintf(age_frag, sizeof(age_frag), "%dh", (int)hrs);
            }
        }

        /* Delay-path health fragment */
        char d_frag[10];
        if (sync_cnt == 0)
        {
            snprintf(d_frag, sizeof(d_frag), "D:--");
        }
        else
        {
            uint32_t pct = (uint32_t)((uint64_t)dresp_cnt * 100u / (uint64_t)sync_cnt);
            if (pct > 999)
                pct = 999;
            snprintf(d_frag, sizeof(d_frag), "D:%u%%", (unsigned)pct);
        }

        /* Aggregate protocol-anomaly count, clamped for display so
         * one very noisy hour doesn't push the row off-screen. */
        uint32_t anomalies = sync_gaps + ann_gaps + fu_orphans + dresp_orphans;
        if (anomalies > 9999)
            anomalies = 9999;

        snprintf(buf, sizeof(buf), "Sync %s %s !%u",
                 age_frag, d_frag, (unsigned)anomalies);
    }
    u8g2_DrawStr(&u8g2, 0, 57, buf);
}

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 15: Network initialization
 *
 * Ethernet driver, L2TAP, IP event handler, DHCP with static-IP
 * fallback, link up/down handling. The dhcp_fallback_task runs for
 * the lifetime of the device, retrying DHCP on every link-up edge.
 *
 * ethernet_init() also derives the device's human-readable hostname
 * from its Ethernet MAC (via the mac_hostname component) and installs
 * it on the netif BEFORE esp_eth_start(), so the very first DHCPDISCOVER
 * carries it in Option 12 and the router lists us by name. Same call
 * also pre-splits the string into two OLED-friendly halves that the
 * boot splash (display_task) and the NETWORK screen consume.
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
    /* A4: Runs for the lifetime of the device. On EVERY link-up edge it
     * retries DHCP from scratch and only falls back to the static IP if
     * DHCP does not answer within the timeout THIS cycle. This fixes
     * the latch bug where a unit that fell back to static (because no
     * DHCP server was present at first boot) kept the static IP forever,
     * even after being replugged into a network that does have DHCP.
     *
     * Per cycle:
     *   0. Arm for the cycle: clear the static latch, drain stale GOT_IP.
     *   1. Wait for the link to be up.
     *   2. (Re)start the DHCP client. apply_static_ip() stops it on
     *      fallback, so it must be restarted explicitly here, not just
     *      on first boot.
     *   3. Wait up to DHCP_FALLBACK_TIMEOUT_MS for IP_EVENT_ETH_GOT_IP.
     *   4. On timeout, apply the static fallback for this cycle.
     *   5. Block until the link drops (g_link_down_sem, A5), then loop
     *      to handle the next plug-in. */
    for (;;)
    {
        /* Prepare for the next link cycle BEFORE the link can come up,
         * so the GOT_IP handler is fully armed before any IP for this
         * cycle arrives:
         *   - clear the static-applied latch so ip_event_handler will
         *     signal GOT_IP again (it suppresses the give while
         *     g_static_applied);
         *   - drain any stale GOT_IP give from a *previous* cycle.
         *     Draining here (not after link-up) can't discard a
         *     current-cycle give, because a current-cycle GOT_IP can
         *     only arrive after the link is up. */
        g_static_applied = false;
        xSemaphoreTake(g_got_ip_sem, 0);

        /* 1. Wait for link up */
        while (!atomic_load(&g_link_up))
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        /* 2. (Re)start DHCP. If it was never stopped this returns
         *    ALREADY_STARTED, which is fine. If a previous cycle fell
         *    back to static, this is what actually re-enables DHCP. */
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

        /* 5. Block until the link drops — zero CPU between link events
         *    (no polling). on_link_down gives g_link_down_sem. If the
         *    link already dropped during the DHCP wait above, a give is
         *    pending and this returns immediately. A spurious/early
         *    wake is harmless: we loop and step 1 re-checks g_link_up
         *    before doing anything. */
        xSemaphoreTake(g_link_down_sem, portMAX_DELAY);
        ESP_LOGI("IP", "Link down — will retry DHCP on next link-up");
    }
}

/* B1: on_link_up is now strictly about LINK / PTP-serving concerns.
 * The one-shot timing core bring-up (MAC, L2TAP, PHC) moved to
 * timing_core_bringup() which runs once at init. A link going up no
 * longer has any bearing on whether the slave is ready to discipline —
 * only on whether PTP packets flow. */
static void on_link_up(void)
{
    ESP_LOGI("ETH", "Link is UP, MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             g_src_mac[0], g_src_mac[1], g_src_mac[2],
             g_src_mac[3], g_src_mac[4], g_src_mac[5]);

    /* Refresh the PTP multicast filter on every link-up. The driver
     * de-dupes internally, so this is idempotent. */
    uint8_t ptp_mcast_all[6] = {0x01, 0x1B, 0x19, 0x00, 0x00, 0x00};
    esp_err_t ferr = esp_eth_ioctl(g_eth_hndl, ETH_CMD_ADD_MAC_FILTER, ptp_mcast_all);
    if (ferr == ESP_OK)
    {
        ESP_LOGI(TAG, "PTP multicast filter added");
    }
    else
    {
        ESP_LOGW(TAG, "PTP multicast filter add: %s", esp_err_to_name(ferr));
    }

    atomic_store(&g_link_up, true);

    /* Only update LED if it was reflecting "no link" — don't overwrite
     * a real timing state (LOCKED, HOLDOVER, ACQUIRING) just because
     * the cable was reconnected. */
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
    /* A5: Wake dhcp_fallback_task so it can re-arm DHCP on the next
     * link-up. Binary semaphore — a give while a give is already
     * pending is a benign no-op (give on a full binary sem just
     * fails silently). */
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

    /* Derive a human-readable hostname from the Ethernet MAC and
     * install it on the netif BEFORE esp_eth_start() runs in
     * app_main. The DHCP client (auto-started above) won't send its
     * DHCPDISCOVER until the physical link comes up, so setting the
     * hostname here guarantees it appears in DHCP Option 12 and the
     * router lists the device by name instead of MAC.
     *
     * mac_hostname_display_lines() then pre-splits the same string
     * into two OLED-friendly halves for the boot splash and the
     * NETWORK screen. Both calls are silently non-fatal — the render
     * paths guard on g_host_line1[0] and fall back to a static
     * label if these buffers are empty. */
    if (mac_hostname_get(g_hostname, sizeof(g_hostname),
                         MAC_HOSTNAME_DASHED) == ESP_OK)
    {
        mac_hostname_apply(g_eth_netif, g_hostname);
        mac_hostname_display_lines(g_hostname,
                                   g_host_line1, sizeof(g_host_line1),
                                   g_host_line2, sizeof(g_host_line2));
    }

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

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 16: Hardware initialization
 *
 * PHC enable + verify, plus the link-independent timing core
 * bring-up (MAC, clock_id, L2TAP, PHC) called once from app_main
 * so identity is correct from instant zero.
 * ═════════════════════════════════════════════════════════════════════════ */

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
            // ❌ REMOVED: don't set time here. Servo will COARSE STEP it.
            return true;
        }

        ESP_LOGW("INIT", "PHC start retry (%d), delta=%lld", i, (long long)delta);
    }

    ESP_LOGE("INIT", "PHC FAILED TO START after retries");
    return false;
}

/* Called from eth_event_handler on every link-up event.
 * Idempotent — safe to call multiple times. */
/* B1: Bring up the timing core. Depends ONLY on the EMAC driver being
 * started (esp_eth_start), not on a link/peer. Called once from app_main
 * after esp_eth_start and before the timing tasks are launched.
 *
 * Steps, all link-independent:
 *   1. Read the MAC address and derive the PTP clock identity.
 *      ETH_CMD_G_MAC_ADDR works as soon as the driver is started; no
 *      peer required. Doing it here means the BMCA dataset, OLED, and
 *      web status all see the correct identity from instant zero,
 *      rather than zeros until first link-up.
 *   2. Initialize + bind L2TAP to the driver handle. Binding (and
 *      timestamp enable) only needs the driver; frames simply won't
 *      arrive until a link exists, which is fine.
 *   3. Start the PHC (PTP clock). phc_start_blocking only toggles
 *      PTP_ENABLE and reads the PHC counter — no link needed.
 *
 * Returns true on success. On failure the LED is set to FAULT and the
 * timing tasks remain parked on g_phc_ready/g_l2tap_ready (never set),
 * which is the safe outcome. */
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

    /* B2: PHC is alive — any task gated on g_phc_ready may now proceed. */
    atomic_store(&g_phc_ready, true);
    ESP_LOGI("INIT", "Timing core ready (PHC running, L2TAP bound) — "
                     "slave will discipline as soon as Sync arrives");
    return true;
}

#include "web_server.h"

/* ═════════════════════════════════════════════════════════════════════════
 * SECTION 17: app_main
 *
 * Entry point. Creates synchronization primitives, brings up the
 * ethernet driver, then timing_core_bringup(), then spawns all
 * tasks, then starts the web server.
 * ═════════════════════════════════════════════════════════════════════════ */

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    // esp_log_level_set("*", ESP_LOG_WARN);
    //  esp_log_level_set("PTP_SERVER", ESP_LOG_INFO);

    /* ── OLED bring-up ──
     * Same hardware as the grandmaster, same I2C wiring (SDA/SCL configured
     * via u8g2_esp32_hal). Bringing this up before everything else so the
     * "booting..." splash is visible immediately. */
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0,
                                          u8g2_esp32_i2c_byte_cb,
                                          u8g2_esp32_gpio_and_delay_cb);

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    ESP_LOGI(TAG, "u8g2_InitDisplay");
    u8g2_InitDisplay(&u8g2); /* sends init sequence; display in sleep until SetPowerSave */

    /* Record boot time for uptime display */
    g_boot_time_ms = esp_timer_get_time() / 1000;

    /* Launch button + display tasks (low priority — never block timing-critical work) */
    xTaskCreate(button_task, "btn", BUTTON_TASK_STACK, NULL, BUTTON_TASK_PRIO, NULL);
    xTaskCreate(display_task, "display", DISPLAY_TASK_STACK, NULL, DISPLAY_TASK_PRIO, NULL);

    /* Synchronization primitives — create BEFORE any task or event
     * handler can reference them. */
    g_eth_up_sem = xSemaphoreCreateBinary();
    g_got_ip_sem = xSemaphoreCreateBinary();
    g_link_down_sem = xSemaphoreCreateBinary(); /* A5 */
    g_l2tap_tx_mtx = xSemaphoreCreateMutex();   /* B4 */

    /* B3: Delay_Req producer/consumer queue. Created BEFORE delay_req_task
     * is spawned so the consumer always has a valid queue handle. */
    g_delay_req_q = xQueueCreate(DELAY_REQ_Q_DEPTH, sizeof(uint8_t));

    /* Ethernet driver init + start. Will not block waiting for link. */
    ethernet_init();
    ESP_ERROR_CHECK(esp_eth_start(g_eth_hndl));
    ESP_LOGI("ETH", "Ethernet started (link state will be reported on connect)");

    /* Start LED early so user gets immediate feedback */
    led_start();
    led_set_state(LED_STATE_NO_LINK);
    xTaskCreate(pps_led_task, "pps_led", 2048, NULL, 1, NULL);

    /* Drifting / fault state-machine watcher (LOCKED → HOLDOVER → FAULT).
     * Reads g_last_sync_ms periodically. Low priority — purely advisory. */
    xTaskCreate(slave_led_watch_task, "led_watch", 3072, NULL, 1, NULL);

    /* B1: Bring up the timing core NOW — driver is started, so MAC,
     * clock_id, L2TAP, and PHC can come up without waiting for a peer
     * or link. This is the architectural decoupling: the slave is
     * ready to discipline as soon as Sync arrives, and the web /
     * OLED / logs see the real MAC and clock_id from the first
     * status read. On failure, timing_core_bringup sets LED_STATE_FAULT
     * and returns false; g_phc_ready and g_l2tap_ready stay false so
     * the timing tasks remain safely parked. We do NOT abort app_main —
     * OLED/LED/web diagnostics still run so the failure is visible. */
    if (!timing_core_bringup())
    {
        ESP_LOGE("MAIN", "Timing core bring-up FAILED — slave will not discipline");
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
     *   ESP_LOGI("ETH", "Promiscuous mode: ON");
     */

    /* A4: DHCP fallback watcher — runs for the lifetime of the device. */
    xTaskCreate(dhcp_fallback_task, "dhcp_fb", 3072, NULL, 2, NULL);

    /* Launch the PTP tasks. They self-gate on g_l2tap_ready / g_phc_ready. */

    /* B3: Delay_Req producer/consumer split. delay_req_task drains
     * g_delay_req_q and performs the L2TAP write, off the RX hot path.
     * Priority 5 sits below ptp_rx (7) so receiving Sync wins over
     * sending Delay_Req under flood. */
    xTaskCreate(delay_req_task, "delay_req", 4096, NULL, 5, NULL);

    xTaskCreate(ptp_rx_task, "ptp_rx", 8192, NULL, 7, NULL);

    xTaskCreate(heap_monitor_task, "heap_mon", 3072, NULL, 1, NULL);

    /* A2: Watchdog — checks every 30 s that the PHC seconds counter is
     * advancing (sampled via emac_get_time, since the slave has no PPS).
     * Triggers emac_restart_preserve_phc on freeze >60 s or critical
     * heap pressure. */
    xTaskCreate(watchdog_task, "wdt", 4096, NULL, 1, NULL);

    /* GM-quality diagnostics — logs offset/delay/jitter/loss every 10 s.
     * 6144 stack: ESP_LOGI with %f format on RISC-V softfloat pulls in
     * newlib's full vfprintf which uses ~3 KB of stack per call. */
    xTaskCreate(gm_diag_task, "gm_diag", 6144, NULL, 2, NULL);

    ESP_LOGI("SLAVE", "PTP slave initialized");
    ESP_LOGI("MAIN", "All tasks launched. Waiting for Ethernet link...");

    /* A1: Start the web server so the slave is reachable for status
     * (GET /, GET /api/status, GET /api/health) and OTA upload
     * (GET /ota, POST /api/ota). */
    web_server_start();
}