/* ============================================================================
 *  ota_credentials.h  --  HTTP Basic Auth credential for the OTA endpoint
 *
 *  Defines the single shared secret used to authenticate POST /api/ota
 *  (firmware upload). Without a real value here, /api/ota is disabled
 *  (returns 503) — the build fails closed rather than open.
 *
 *  ─── HOW TO SET ──────────────────────────────────────────────────────────
 *
 *  1. Choose a username and a strong password. This is your ONLY remote-
 *     flashing barrier and Basic Auth travels in cleartext on plain HTTP,
 *     so use ≥16 random characters. Longer is better.
 *
 *  2. Compute the Base64 of "user:password" (literal colon, no quotes):
 *
 *         echo -n "admin:correct-horse-battery-staple" | base64
 *
 *     PowerShell equivalent:
 *
 *         [Convert]::ToBase64String(
 *             [Text.Encoding]::UTF8.GetBytes("admin:correct-horse-battery-staple"))
 *
 *  3. Paste the resulting Base64 string into OTA_BASIC_AUTH_B64 below.
 *
 *  ─── SECURITY NOTES ──────────────────────────────────────────────────────
 *
 *  - ADD THIS FILE TO .gitignore. Commit a template with the sentinel.
 *  - Basic Auth over plain HTTP is sniffable on the wire. For real
 *    production, terminate TLS in front of the device, or move the OTA
 *    endpoint to a physically isolated management interface.
 *  - Leaving the sentinel value disables network OTA entirely. The first
 *    real credential change requires a one-time UART reflash to take
 *    effect, which is intentional — it forces a physical step before
 *    network OTA is reachable.
 *  - The OTA handler also requires Content-Type: application/octet-stream
 *    on the POST. This is deliberate: application/octet-stream is NOT a
 *    CORS-safelisted Content-Type, so a hostile cross-origin webpage
 *    cannot silently replay your cached Basic Auth credentials via fetch()
 *    — the browser would have to issue an OPTIONS preflight first, which
 *    the device refuses. Legitimate flows (the device's own /ota page,
 *    and curl with the right flag) set this Content-Type naturally.
 *
 *  ─── USAGE FROM A BROWSER / CURL ─────────────────────────────────────────
 *
 *  Browser: navigate to http://<device-ip>/ota. The page will POST and the
 *  browser will prompt for credentials; enter the same user/password you
 *  encoded above. Credentials are cached for the browser session. The page
 *  already sends Content-Type: application/octet-stream — no extra steps.
 *
 *  curl: the -H flag is REQUIRED. Without it, curl defaults to
 *  application/x-www-form-urlencoded for --data-binary, which the OTA
 *  handler now rejects as the CSRF mitigation described above.
 *
 *      curl -u admin:correct-horse-battery-staple \
 *           -H "Content-Type: application/octet-stream" \
 *           --data-binary @firmware.bin \
 *           http://<device-ip>/api/ota
 *
 *  PowerShell equivalent (Invoke-WebRequest):
 *
 *      $cred = [Convert]::ToBase64String(
 *          [Text.Encoding]::UTF8.GetBytes("admin:correct-horse-battery-staple"))
 *      Invoke-WebRequest -Uri http://<device-ip>/api/ota `
 *          -Method POST `
 *          -ContentType "application/octet-stream" `
 *          -Headers @{ Authorization = "Basic $cred" } `
 *          -InFile firmware.bin
 *
 *  ─── EXPECTED RESPONSES ──────────────────────────────────────────────────
 *
 *      200 OK         -> upload succeeded; device reboots in ~1 second
 *      400 Bad Req    -> body short, image too large, validation failed,
 *                        or Content-Type was not application/octet-stream
 *      401 Unauth     -> missing/wrong Authorization header
 *      409 Conflict   -> another OTA upload is already in progress
 *      500 Internal   -> OTA partition missing, malloc failed, write failed
 *      503 Svc Unav   -> OTA_BASIC_AUTH_B64 still set to the sentinel value
 * ============================================================================ */

#ifndef OTA_CREDENTIALS_H
#define OTA_CREDENTIALS_H

/* Base64("user:password"). The literal "REPLACE_ME" disables OTA entirely. */
//#define OTA_BASIC_AUTH_B64  "REPLACE_ME"

/* Default credentials admin:admin */
#define OTA_BASIC_AUTH_B64  "YWRtaW46YWRtaW4="

#endif /* OTA_CREDENTIALS_H */