/**
 * @file allystar_timing.h
 *
 * TAU1201 timing-mode configurator for the ESP32-P4 PTP Grandmaster.
 *
 * Applies a sequence of CFG-* messages to the TAU1201 over UART_NUM_1 every
 * boot, in RAM only (no CFG-CFG save). Module reverts to factory default on
 * power-down by design.
 *
 * Protocol per ALLYSTAR GNSS Receiver Protocol Specification V2.3:
 *   Frame:    0xF1 0xD9 | GroupID | SubID | Length(LE u16) | Payload | CK1 CK2
 *   Checksum: 8-bit Fletcher over [GroupID .. end-of-payload], masked 0xFF
 *
 * Integration:
 *   1. Call initialize_allystar_timing_service(cb) once during boot, after
 *      gps_uart_init() has installed the UART driver.
 *   2. Inside the existing gps_task UART-read loop, after uart_read_bytes()
 *      returns >0, call allystar_timing_feed_data(buf, len). This forks the
 *      RX byte stream to the timing module's parser while leaving the normal
 *      NMEA pipeline intact.
 *   3. On any failed step the error callback fires with the step name. The
 *      callback runs in the timing task context.
 *
 * Threading: the configurator runs in its own FreeRTOS task. It uses a
 * stream-buffer pump driven by gps_task; the timing task itself only writes
 * to UART (TX is mutex-protected). When the sequence terminates (success or
 * failure), initialization_sequence_done flips to true and feed_data() becomes
 * a cheap early-out.
 */

#ifndef ALLYSTAR_TIMING_H
#define ALLYSTAR_TIMING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Survey duration in seconds. The configurator caps the survey at this
 * many seconds; main.c reads this for OLED display purposes. */
#define ALLYSTAR_SURVEY_MAX_SECONDS  (15 * 60)

#ifdef __cplusplus
extern "C" {
#endif

/* When true, allystar_timing_feed_data() is a no-op. Polled by gps_task to
 * avoid the cost of forwarding bytes once the sequence is finished. */
extern volatile bool initialization_sequence_done;

/* When non-NULL, last-known timing-config error step name (static string).
 * NULL means "no error so far". Polled by OLED render code to show status. */
extern const char *volatile g_timing_error_step;

/* When true, the sequence finished cleanly. */
extern volatile bool g_timing_config_ok;

/**
 * Error callback. Fires once if any CFG step fails after retries, or if the
 * 3D-fix wait times out. failed_step_name is a static string from the .rodata
 * of this translation unit; no need to copy.
 */
typedef void (*allystar_timing_error_cb_t)(const char *failed_step_name);

/**
 * Spawn the configurator task. Safe to call multiple times; second and later
 * calls are no-ops (the task is only created once per boot).
 *
 * @param error_callback May be NULL.
 */
void initialize_allystar_timing_service(allystar_timing_error_cb_t error_callback);

/**
 * Fork UART RX bytes into the timing module. Cheap no-op once the sequence
 * has finished. Call from gps_task immediately after uart_read_bytes().
 */
void allystar_timing_feed_data(const uint8_t *buffer, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* ALLYSTAR_TIMING_H */
