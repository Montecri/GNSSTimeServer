# GNSS Time Servers Built From Scratch - The PPSv2 Edition


Two homemade time servers, one obsession: turning a noisy one-pulse-per-second tick from the GPS satellites overhead into a clock the rest of your network can trust. They come from the same workbench and the same stubborn insistence on building from scratch — they differ only in **how precise you need to be**.

- 🛰️ The **GNSS Time Server** speaks **NTP and RDATE** and keeps a whole network honest to the **millisecond**. Portable, battery-backed, WiFi-enabled — happy in a HAM shack, a home lab, or a field bag.
- 📡 The **PTP Grandmaster** speaks **PTPv2 (IEEE 1588-2008)** and disciplines a wired Ethernet segment to the **sub-microsecond** — for industrial automation, telecom, broadcast, finance, and instrumentation, where "close enough" stopped being close enough.

Same maker, same itch, three orders of magnitude apart.

<br>

📰 The story behind both builds, in two parts:
*[An IoT Maker Tale: Stratum-1 Time Server Built From Scratch](https://www.linkedin.com/pulse/iot-maker-tale-stratum-1-time-server-built-from-scratch-monteiro/)* — and its sequel, *An IoT Maker Tale, Part II: From Stratum-1 NTP to a Sub-Microsecond PTP Grandmaster* (link to be added).

<br>

[!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://www.buymeacoffee.com/cristmon)

---

## 🧭 Which one is for you?

Both are GNSS-disciplined, self-contained, dissectable-from-the-inside time sources built on inexpensive microcontrollers, with the same maker-friendly DNA — an informative OLED, a push button to cycle screens, status LEDs, and a web dashboard. The table is the fastest way to choose; the rest of this README walks through *why* each row is what it is.

| | 🛰️ **GNSS Time Server** | 📡 **PTP Grandmaster** |
| --- | --- | --- |
| **Protocols** | NTP + RDATE (RFC 868) | PTPv2 (IEEE 1588-2008), Layer-2 Ethernet |
| **Typical accuracy** | Milliseconds across the network | **Sub-microsecond** (mean error −74 ns) |
| **Brain** | ESP8266 / ESP32 + Arduino | ESP32-P4 + ESP-IDF |
| **Timestamping** | Software | **Hardware** (ESP32-P4 on-chip PTP-capable EMAC) |
| **Connectivity** | WiFi + 10/100 Ethernet (W5500) | Wired Gigabit Ethernet (IP101 PHY) |
| **Holdover clock** | DS3231 TCXO, battery-backed | On-chip PHC + software servo |
| **Power** | Dual (mains + 18650), portable | USB-C |
| **Field-ready** | Yes — runs on battery, WiFi out of the box | Bench / rack — wired Ethernet by nature |
| **Best for** | Home labs, HAM shacks, general sync, field use | Industrial, telecom, broadcast, finance, instrumentation |

The short version: reach for the **GNSS Time Server** when you want a portable box that keeps any network's clocks accurate and runs anywhere. Reach for the **PTP Grandmaster** when milliseconds aren't enough and you need wired sub-microsecond precision.

<br>

---

## 🛰️ Where time comes from

Both devices start at the same place: the constellation of GPS / GNSS satellites overhead, each carrying atomic clocks accurate to about a second every few hundred million years — a free Stratum-0 reference floating over our heads. A GNSS module locks onto them and emits two things that matter: the current time, and a **Pulse Per Second (PPS)** — a clean (almost) 1 Hz electrical edge marking the top of each second.

That PPS edge is the heartbeat. Everything downstream is a question of how faithfully you can capture it and how cleanly you can pass it on.

- The **GNSS Time Server** disciplines a battery-backed **DS3231 TCXO** real-time clock from the satellites and answers NTP/RDATE requests. The temperature-compensated crystal holds time gracefully during satellite outages.
- The **PTP Grandmaster** goes further: it captures the PPS edge *inside the GPIO interrupt* and uses it to discipline the ESP32-P4's on-chip **PTP Hardware Clock (PHC)** through a software servo, then hardware-timestamps PTP frames as they leave the wire. No external RTC — the servo itself "remembers" the crystal's drift and coasts through brief outages.

The single biggest lever on precision is the GNSS module. A navigation-grade receiver (NEO-6m, ATGM336H) assumes it's moving and constantly re-estimates its position, and that noise rides straight onto the PPS edge. A **timing-mode** receiver like the **AllyStar TAU1201** knows it's stationary and spends its math on the time solution instead — dropping PPS jitter from hundreds of nanoseconds to roughly **10 ns**. The GNSS Time Server works beautifully with either; the PTP Grandmaster's sub-microsecond numbers depend on it.

---

## ⭐ What both share — and where they part ways

Everything below is organized by capability, comparing the two builds side by side. Where a feature belongs to only one device, it's marked 🛰️ (NTP/RDATE) or 📡 (PTP); shared traits are marked 🤝.

### 🔌 Connectivity & protocols

- 🛰️ **NTP and RDATE (RFC 868)** served over WiFi (client *or* access point) and 10/100 Mbps Ethernet via a hardwired **W5500** controller.
- 📡 **PTPv2 over Layer-2 Ethernet** (Annex F) — raw frames, no IP/UDP overhead — with two-step **Sync + Follow_Up**, **Announce**, and **Delay_Req / Delay_Resp** exchanges, on wired Gigabit through an **IP101** PHY.
- 📡 **Plug-and-play addressing** — DHCP first, automatic static-IP fallback after 15 s if no server replies. (The NTP build offers WiFi configuration via its web page instead.)
- 🤝 Both come up on the network with no hand-holding and announce themselves clearly once linked.

> [!IMPORTANT]
> **PTP belongs on wired Ethernet — never WiFi.** The protocol's sub-microsecond accuracy assumes a deterministic, hardware-timestamped physical layer; WiFi's association, retry, and queueing jitter would swamp it entirely. The NTP/RDATE server, by contrast, is perfectly at home on WiFi — millisecond-class accuracy tolerates it comfortably.

### ⏱️ The clock and its discipline

- 🛰️ **DS3231 TCXO RTC**, satellite-synched and CR2032-backed, with PPS aligning the top-of-second.
- 📡 **On-chip PHC** steered by a three-state software servo (`UNLOCKED → ACQUIRING → LOCKED`): a coarse step on acquisition, a fine residual correction, then a slow **PID loop** whose derivative term tracks the thermal transients of a crystal warming up — hundreds of ppb/sec of drift that an integrator alone is too slow to catch.
- 📡 A **32-sample running-mean averager** (≈5.7× noise reduction) is the servo's working signal, and frequency corrections are **slew-rate-limited** so the servo physically cannot chase a transient faster than the averager smooths it.
- 📡 **TAI ↔ UTC** handling (37 s offset, configurable for future leap seconds).

### 🛰️ GNSS interface & signal diagnostics

- 🤝 **Multi-constellation NMEA** — GPS, GLONASS, Galileo, BeiDou (plus QZSS / multi-GNSS GN talker on the PTP build).
- 📡 **PPS captured inside the ISR**, eliminating 10–50 µs of FreeRTOS scheduling jitter that would otherwise be injected straight into the offset measurement.
- 📡 Live **signal-quality telemetry**: per-constellation SNR (avg/min/max), satellites in view vs. used, satellite churn (PRN entries/exits per window), and PPS edge-to-edge jitter (min/max/mean-abs/outliers).
- 📡 **Fix-validity gating** — time is only trusted when RMC confirms a valid fix, so a coasting receiver can't feed stale time into the servo.

### 🧭 Grandmaster behaviour (📡 PTP only)

- **Best Master Clock Algorithm (BMCA)** with full priority1 / clockClass / accuracy / variance / priority2 / clockIdentity ordering, so multiple grandmasters can coexist and fail over gracefully.
- A **stability filter** that requires 3 consecutive Announces from the same candidate before changing roles, and a **6-second announce timeout** before reclaiming the master role — together giving the master role a clean hysteresis that shields downstream slaves from needless flapping.
- **Dynamic clock quality** advertising: clockClass 6 / accuracy 0x21 when locked, 7 / 0x23 in holdover, with a 30-second post-holdover settling window so recovering slaves don't rail their servos against a still-settling master.
- **Slave tracking** — up to 8 simultaneous slave clock IDs with a 30-second activity timeout.

### 🛡️ Holdover & resilience

- 🛰️ The **DS3231 TCXO** keeps time through satellite outages; the unit is **dual-powered** (mains + 18650) so it rides out power blips too.
- 📡 **Smart holdover** distinguishes a *sentence-only* outage (NMEA stopped but PPS still alive — preserve all learned servo state) from a *true* PPS loss (re-acquire from scratch).
- 📡 A **self-healing watchdog** can restart the EMAC while preserving PHC continuity across the ~120 ms dead window — slaves see one missed Sync, not a full re-acquire — backed by PHC-freeze and heap-pressure detection, torn-read protection on the PHC second-boundary, and a leak-free pure-arithmetic `timegm`.

### 🖥️ Status at a glance — display, button, LEDs, dashboard

This is the maker-friendly heart both devices share: you can read their health across the room, drill in with one button, or pull up a full dashboard from a browser.

- 🤝 An **OLED display** with a **push button** to cycle informative screens, plus **status LEDs** and a **web dashboard**. No laptop required to know what's going on.
- 🛰️ **Boot diagnostics** report the reason for the last shutdown and the start-up status of each module — so if it ever hangs on boot, you know exactly which module to blame. A **dual-function button** short-presses to switch the display and long-presses to toggle WiFi. Dedicated 5 mm LEDs cover GPS lock, PPS, and WiFi; a rich web dashboard reports MCU / RTC / GNSS / WiFi telemetry (uptime, heap, temperature, IP, MAC, satellites, precision, lat/long, and more), updated every 5 seconds from a lightweight JSON string with no full-page reloads. The page is a single gzip-compressed HTML under ~20 KB with no external dependencies, password-protected for critical functions, and supports **OTA firmware upgrades**.
- 📡 **Five navigable screens** — 📊 Main (lock state, UTC, offset, GNSS status, active slaves), ⚙️ Servo (lock duration, quality %, peak spread, frequency adjustment, integrator, PPS count), 📡 GNSS Signal (verdict, SNR range, SVs in view vs. used, fix quality), 🌐 Network (link, IP, uptime, heap), and 🔧 Timing Config (TAU1201 configurator status, which **auto-jumps into view on failure**). The lock LED has five distinct patterns — off (no link), slow blink (acquiring), solid (locked), heartbeat (holdover), fast blink (fault) — and a second LED pulses on **every PPS edge**.

---

## 🧰 Parts

### 🛰️ GNSS Time Server

- Amica NodeMCU (ESP8266 / ESP-12) **or** ESP32Duino (ESP32-WROOM-32)
- DS3231 RTC
- GNSS module — tested with Neo-6m V2, ATGM336H, ZKMICRO AT6558D, TOPGNSS AllyStar TAU1201
- 1.3" SH1106 OLED display
- **Mains-powered:** Hi-Link 5V/3W · Mini-360 DC-DC buck · TP4056 module · 18650 holder · 0.5 A/230 V fuse · 10D561K varistor
- **Adapter-powered:** 18650 battery shield · 5.5 × 2.1 mm (or suitable) socket
- **Ethernet version:** WIZnet W5500 module
- Resistors (150, 100, 150 Ω for the LEDs) · and momentary push button

### 📡 PTP Grandmaster

- **JC-ESP32P4-M3-DEV (Guition board)** (Espressif) — hardware PTP-capable EMAC
- **AllyStar TAU1201** GNSS module, configured in timing mode
- **IP101** Ethernet PHY (on-board on the EV-Board variant used here)
- 1.3″ SH1106 OLED (128×64, I²C, address `0x78`)
- Momentary push button · 5 mm LEDs (lock + PPS) with current-limiting resistors
- Active GPS antenna with clear sky view
- RJ45 cable (Cat 5e+ — for back-to-back testing - No crossover needed for connectiong two JC-ESP32P4-M3-DEV since they are MDI/MDIX)
- 5 V supply (USB-C or barrel jack, board-dependent)

> [!TIP]
> The **TAU1201 is the single highest-impact part choice** in either build, and the make-or-break one for the PTP grandmaster. A navigation-grade receiver works — the firmware accepts NMEA from any of them — but PPS jitter will be hundreds of nanoseconds rather than ~10 ns, and the achievable accuracy scales with it. If the budget allows, fit the TAU1201.

> [!NOTE]
> On the NTP/RDATE build, some functionalities are ESP32-only, due either to ESP8266 limitations or features not yet backported.

---

## 🛠️ Building each one

The two devices use **different toolchains** — this is the clearest practical fork between them.

**🛰️ GNSS Time Server — Arduino / PlatformIO**

```
Libraries:
  paulstoffregen/Time @ ^1.6
  makuna/RTC          @ ^2.3.5
  mikalhart/TinyGPSPlus @ ^1.0.3
  olikraus/U8g2       @ ^2.28.8
  arcao/Syslog        @ ^2.0.0
```

Wiring diagrams (ESP8266, ESP32, ESP32-with-Ethernet) and the power-supply diagram are below.

**📡 PTP Grandmaster — ESP-IDF 5.5.4**

```
Libraries:
  olikraus/U8g2           @ ^2.36.19
  mkfrey/u8g2-hal-esp-idf @ ^v3-compat
```

> [!IMPORTANT]
> On its first boot in a new location, the PTP grandmaster spends a few minutes on a **TAU1201 position survey** before timing-mode engages — screen 5 shows progress. This is part of the timing mode process activation, it will compute it's current location during this window and then revert to timing only calculation.

---

## 🧪 Measuring sub-microsecond honestly (📡 PTP)

A grandmaster's accuracy claim is only as good as the instrument measuring it. A software-timestamping slave on a generic PC would drown sub-microsecond behaviour in its own kernel jitter — so the measurement instrument has to be at least as good as the device under test. The project therefore includes a **companion slave unit on identical ESP32-P4 hardware**, with its own hardware-timestamped state machine, servo, OLED, button, LEDs, and web dashboard reporting offset, path delay, and lock quality.

```
   ┌──────────────────┐                    ┌──────────────────┐
   │  GPS antenna     │                    │                  │
   │       │          │                    │                  │
   │       ▼          │   Ethernet cable   │                  │
   │  ESP32-P4 GM     │◄──────────────────►│  ESP32-P4 Slave  │
   │  (this repo)     │       100 Mbps     │  (sibling repo)  │
   │                  │                    │                  │
   │  OLED + LEDs     │                    │  OLED + LEDs     │
   └──────────────────┘                    └──────────────────┘
```

Captured by that slave, on a typical evening with the antenna on the balcony:

| Metric | Value |
| --- | --- |
| 🎯 Within ±300 ns of true GPS time | ~30% |
| 🎯 Within ±1 µs | ~74% |
| 🎯 Within ±2 µs | ~89% |
| 📏 Mean error | −74 ns |
| ⏱️ Cold-boot lock time | ~30 s |
| 🛰️ Raw PPS jitter (TAU1201, timing mode) | ~10 ns |
| 🔄 EMAC-restart PHC continuity | < 1 µs residual |

For context, commercial PTP grandmasters delivering similar numbers typically start north of **US$2,000**. The bill of materials here — for *both* master and slave — is closer to a nice dinner.

> [!NOTE]
> **What's next:** a **PTP-capable switch** (transparent- or boundary-clock function preferred) to run multiple slaves against the master at once, and a third unit configured as a boundary clock. Switched-network testing is where the interesting questions surface — master load scaling, BMCA failover under real witnesses, switch-induced residence delay.

---


## ⚠️ Before you build — read these

<p align="center"><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"><b>   ATTENTION   </b><img src="https://user-images.githubusercontent.com/38574378/132773469-08fb7b59-2f9d-4641-9665-c8d50d3904bc.png"></p>

🚨 **GNSS serial speed (both builds).** Most GNSS modules ship at 9600 bps. That's fine for GPS alone, but can be too slow for multi-constellation work. The code favors **115200 bps**, which is the practical limit and pairs well with raising the module from 1 Hz to 10 Hz updates. Tools like *GnssToolKit3* can change a module's rate and baud across several brands.

🚨 **PTP needs wired Ethernet (📡 PTP).** Never attempt PTP over WiFi — association/retry/queueing jitter dwarfs the protocol's accuracy budget and no servo tuning can recover it.

🚨 **TAU1201 timing mode after a survey (📡 PTP).** First boot in a new location takes several minutes to reach `READY` while the module surveys its position. Survey window is configurable in `allystar_timing.h`.

🚨 **Domain 0 by default (📡 PTP).** The grandmaster announces on PTP domain 0 and will join the BMCA election on any network already running PTP there. Set a different domain in `main.c` (`#define PTP_DOMAIN`) before deploying alongside existing infrastructure.

🚨 **GNSS time is TAI, not UTC (📡 PTP).** The firmware advertises `currentUtcOffset = 37` and sets `PTP_TIMESCALE` on the wire. Slave software that treats PTP timestamps as UTC will be off by the TAI−UTC offset. This is correct per IEEE 1588 — just worth knowing when integrating with software that's never met a proper grandmaster.

---

## 🤖 A note on AI tooling

The first build, three years ago, was stitched together by hand from two PDFs and an Arduino-forum thread. The second would not have reached this caliber, in the time one person had, without the generation of AI tools available in 2026. Yes, the code is heavilly built by AI. Don't like it? Go there and fix!

The PID tuning still had to be measured against reality. The interrupt path still had to be debugged on real hardware. The architectural decisions are still mine to defend. But AI compressed weeks of reading-and-experimenting into hours and let a single maker take on the scope of *two* firmware codebases — grandmaster and slave — in one project cycle. For anyone hesitating to start something that feels too large: the tools have changed, and the barrier is lower than you think.

---

## 📚 Related & lineage

- 🛰️ [**GNSSTimeServer**](https://github.com/Montecri/GNSSTimeServer) — the NTP/RDATE server, featured by Hackaday and exhibited at Maker Faire Rome 2023
- 🔀 [**Dual Display PIR-Enabled branch**](https://github.com/Montecri/GPSTimeServer/tree/Dual-Display) — an alternative build of the NTP/RDATE server
- 📡 **PTPGrandmaster** — the sub-microsecond follow-up (this project)
- 📡 **PTPGrandmasterSlave** *(coming soon)* — the companion slave used to validate the PTP numbers above

---

## 🙏 Acknowledgements

The NTP/RDATE server stands on the shoulders of [w8bh.net](http://w8bh.net/avr/clock2.pdf) and the [Arduino NTP time-server thread](https://forum.arduino.cc/t/ntp-time-server/192816), with contributions from @mmarkin and @sjthespian. The PTP grandmaster stands on the open-source PTP community, Espressif's ESP-IDF PTP examples, and the IEEE 1588 specification itself. And all stand on everyone who sent feedback over the years and asked me to pursue a PTP version of the GNSSTimeServer — that community is the reason there's a Part II.

---

## 📜 License

MIT License — see [LICENSE](LICENSE) for details.

[!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://www.buymeacoffee.com/cristmon)

73 📻
