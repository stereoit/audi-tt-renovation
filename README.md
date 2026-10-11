# Audi TT Mk1 (8N) Jaeger Instrument Cluster EEPROM & Bench Diagnostics

A technical reference, bench wiring guide, and EEPROM decoding toolkit for the early **Audi TT Mk1 (8N) 1.8T 225HP** instrument cluster manufactured by **Magneti Marelli / Jaeger** (Part Number: `8N1 919 860 C`).

---

## Table of Contents

- [1. Overview & Unit Identification](#1-overview--unit-identification)
- [2. CAN-Bus Architecture & Protocol Analysis](#2-can-bus-architecture--protocol-analysis)
- [3. Hardware Tools Available](#3-hardware-tools-available)
- [4. Bench Wiring & Harness Schematic](#4-bench-wiring--harness-schematic)
  - [4.1 Wiring Diagram (ASCII Schematic)](#41-wiring-diagram-ascii-schematic)
  - [4.2 Cluster Pinouts (Blue & Green 32-Pin Connectors)](#42-cluster-pinouts-blue--green-32-pin-connectors)
  - [4.3 OBD-II Diagnostic Port Pinout](#43-obd-ii-diagnostic-port-pinout)
  - [4.4 Bench Setup Using the OBD Splitter](#44-bench-setup-using-the-obd-splitter)
- [5. Software Tools & Diagnostic Procedures](#5-software-tools--diagnostic-procedures)
  - [5.1 FTDI Driver Configuration (Critical Latency Setting)](#51-ftdi-driver-configuration-critical-latency-setting)
  - [5.2 VAG K+CAN Commander 1.4 (EEPROM & PIN Extraction)](#52-vag-kcan-commander-14-eeprom--pin-extraction)
  - [5.3 VCDS-Lite (Diagnostics & Actuator Tests)](#53-vcds-lite-diagnostics--actuator-tests)
  - [5.4 kw1281test (CLI Tool for Marelli Dumps)](#54-kw1281test-cli-tool-for-marelli-dumps)
  - [5.5 MPPS & ME7 Tools (Engine ECU Integration)](#55-mpps--me7-tools-engine-ecu-integration)
- [6. EEPROM Memory Architecture & Decoding Reference](#6-eeprom-memory-architecture--decoding-reference)
  - [6.1 What the 64 KB "direct read" actually contains](#61-what-the-64-kb-direct-read-actually-contains)
  - [6.2 Odometer (Mileage) Decoding](#62-odometer-mileage-decoding)
  - [6.3 Immobiliser PIN / SKC](#63-immobiliser-pin--skc-unverified)
  - [6.4 Transponder Key Data](#64-transponder-key-data)
- [7. Common Failure Modes of Jaeger M73 Clusters](#7-common-failure-modes-of-jaeger-m73-clusters)
- [8. Dump Tooling (`tools/eeprom_tool.py`)](#8-dump-tooling-toolseeprom_toolpy)
- [9. Using the 920 (CAN / Immo 3) Cluster in the Early 919 Car](#9-using-the-920-can--immo-3-cluster-in-the-early-919-car)
- [10. ME7.5 Engine ECU Flash (`ECU/`)](#10-me75-engine-ecu-flash-ecu)
  - [10.1 The three files](#101-the-three-files)
  - [10.2 Flash layout and checksum structure](#102-flash-layout-and-checksum-structure)
  - [10.3 What the "SAI off" modification is](#103-what-the-sai-off-modification-is)
  - [10.4 Plan for the final firmware](#104-plan-for-the-final-firmware)
- [11. Flash Tooling (`tools/me7_tool.py`)](#11-flash-tooling-toolsme7_toolpy)

---

## 1. Overview & Unit Identification

Early production Audi TT (8N) models (1998–1999) equipped with the 1.8T 20V 225 HP engine (engine code **APX**, Quattro) were fitted with instrument clusters manufactured by **Jaeger / Magneti Marelli**.

| Parameter | Specification |
| :--- | :--- |
| **Vehicle** | Audi TT Coupe / Roadster Mk1 (Type 8N, 1998–1999) |
| **Engine** | 1.8T 20V 225 HP (165 kW), APX, ME7.5 (early) |
| **VAG Part Number** | `8N1 919 860 C` (or `8N1919860C`) |
| **Manufacturer** | Magneti Marelli / Jaeger |
| **Marelli Hardware Part Number** | `0905 032 50 9901` (also cross-referenced with `09053259801`, `09053259901`) |
| **Diagnostic Identifier** | `KOMBI+WEGFAHRS. M73 D29` |
| **Software Revision** | `D29` / `V7.66ML EP_2002` |
| **Microcontroller (MCU)** | Motorola / Freescale **MC68HC912DG128** (112-pin LQFP) |
| **Memory Dump Size** | 65,536 bytes (64 KB Full Flash/EEPROM address map) |
| **Internal EEPROM** | 2,048 bytes (mapped at `$0800–$0FFF` / mirrored in RAM space `$3800–$3FFF`) |
| **Immobilizer Generation** | Immobilizer II (Immo 2, ID: `AUZ5Z0...`) |

---

## 2. CAN-Bus Architecture & Protocol Analysis

### Does the `8N1 919 860 C` support CAN-Bus?

> **Verdict: NO for engine instrumentation and drivetrain telemetry.**

In the Volkswagen Audi Group (VAG) instrument cluster numbering system, the 4th, 5th, and 6th digits indicate CAN capability:

1. **`xxx 919 xxx` (Non-CAN Instrument Clusters):**
   - **Engine Speed (RPM)**: Transmitted over a dedicated analog pulsed wire directly from the ECU (Bosch ME7.5 pin 37) to the cluster (Blue Connector T32 Pin 11).
   - **Vehicle Speed**: Transmitted as discrete pulses from Hall sensor **G22** on the gearbox directly to the cluster (Blue Connector T32 Pin 28).
   - **Coolant Temperature**: Analog signal from sensor **G2** (NTC resistor) directly to the cluster (Blue Connector T32 Pin 8).
   - **Fuel Consumption**: Analog square-wave pulse width modulated signal from the ECU to the cluster (Blue Connector T32 Pin 25).
   - **Drivetrain CAN-Bus (Powertrain CAN)**: The cluster does **not** communicate with the engine ECU via CAN. Pins 18 & 19 on the Green 32-pin connector are non-functional or unpopulated on this revision.

2. **`xxx 920 xxx` (CAN-Bus Instrument Clusters, introduced ~mid-1999 / MY2000+):**
   - Engine RPM, vehicle speed, coolant temperature, check engine status (MIL), and ESP telemetry are all transmitted digitally over high-speed Powertrain CAN (500 kbit/s) on Green Connector pins 18 (CAN-High) and 19 (CAN-Low).
   - Immobilizer upgraded to Immo 3.

3. **Display & Radio Communication (919 Clusters):**
   - The Driver Information System (FIS/DIS) communicates with original head units (Audi Concert I / Chorus I) via a **3-wire synchronous bus** (`Clock`, `Data`, `Enable`) on the 32-pin Grey connector (T32b), **not** Infotainment CAN-Bus.
   - Diagnostics and bench flashing are performed exclusively via **ISO 9141-2 K-Line** (pin 28 on the Green connector) using KWP1281 / KWP2000 protocols.

---

## 3. Hardware Tools Available

This project utilizes the following bench equipment:

| Equipment | Purpose & Application |
| :--- | :--- |
| **Regulated 12V Power Source** | Provides clean DC power (12.0 V to 13.8 V, current limit set to 1.5 A – 2.0 A). Powering both the cluster and the diagnostic interface. |
| **OBD-II Splitter (1-to-2 or Breakout)** | Acts as an interface breakout box. Dupont cables insert into one female port to inject power and K-Line; the diagnostic cable plugs into the other female port. |
| **Dupont Cables (M-F / M-M)** | Connects power supply and OBD splitter pins to the cluster's miniature pins in the Blue and Green sockets. |
| **Blue VAG KKL Cable (FTDI FT232)** | High-quality FTDI-based ISO 9141-2 K-Line interface for **VCDS-Lite** (diagnostics, actuator tests, measuring blocks) and **kw1281test**. |
| **Red VAG Cable (VAG Commander 1.4)** | Special interface for **VAG K+CAN Commander 1.4** to perform direct microcontroller memory reads and EEPROM dumps on Magneti Marelli clusters via K-Line. |
| **MPPS & ME7 Tools** | Used for reading/flashing the companion Bosch ME7.5 ECU (06A 906 032 ...) on the bench, managing Immo OFF or ECU-to-Cluster matching. |
| **PC (Windows)** | Host machine running diagnostic software with FTDI Virtual COM Port (VCP) drivers configured to 1 ms latency. |

---

## 4. Bench Wiring & Harness Schematic

### 4.1 Wiring Diagram (ASCII Schematic)

```text
======================================================================================
                         AUDI TT MK1 BENCH WIRING SCHEMATIC
                   Cluster: 8N1 919 860 C (Magneti Marelli / Jaeger)
======================================================================================

  [ +12V DC Regulated Power Supply ]
           │
           ├─── (+) 12V Rail ─────────────────┬───────────────────┬──────────────┐
           │                                  │                   │              │
           │                                  ▼                   ▼              ▼
           │                            OBD Splitter         Cluster T32    Cluster T32
           │                             Pin 16 (VCC)        Pin 23 (Kl.30) Pin 1 (Kl.15)
           │                                                      │              │
           │                                                      │         [Optional]
           │                                                      │         Cluster T32
           │                                                      │         Pin 30 (S-Contact)
           │
           └─── (-) GND Rail ─────────────────┬───────────────────┬──────────────┘
                                              │                   │
                                              ▼                   ▼
                                         OBD Splitter        Cluster T32
                                         Pins 4 & 5 (GND)    Pins 9 & 24 (GND)
                                                                  │
                                                             [Optional]
                                                             Cluster T32
                                                             Pin 7 (Sensor GND)

  [ K-Line Diagnostic Bus ]
         Cluster Green (T32a)                                OBD Splitter
              Pin 28 (K-Line) ─────────────────────────────►  Pin 7 (K-Line)

======================================================================================
  DIAGNOSTIC CABLE CONNECTION:
  
  [ OBD Splitter Port 1 ] ◄── Dupont wires from Power Supply & Cluster
  [ OBD Splitter Port 2 ] ◄── Blue VAG KKL Cable  (for VCDS-Lite / kw1281test)
                                   -OR-
                              Red VAG Commander (for EEPROM / PIN reading)
                                   │
                                   ▼
                            [ USB to PC ]
======================================================================================
```

### 4.2 Cluster Pinouts (Blue & Green 32-Pin Connectors)

#### Blue 32-Pin Connector (T32) — Power & Core Instrumentation

| Pin # | Function | Bench Connection |
| :---: | :--- | :--- |
| **1** | **Terminal 15 (+12V Switched Ignition)** | **Connect to +12V DC** |
| 2 | Brake pad wear indicator | Open / Not connected |
| 3 | Speedometer output 1 | Open |
| 7 | **Terminal 31 (Sensor Earth / Ground)** | **Connect to GND** (recommended) |
| 8 | Coolant temperature sender (G2) | Open (gauge will read cold) |
| **9** | **Terminal 31 (Power Earth / Ground)** | **Connect to GND** |
| 11 | Engine Speed Signal (RPM input from ECU) | Open |
| 17 | Main beam (High beam indicator) | Open |
| 18 | Left turn signal indicator | Open |
| **23** | **Terminal 30 (+12V Constant Battery)** | **Connect to +12V DC** |
| **24** | **Terminal 31 (Power Earth / Ground)** | **Connect to GND** |
| 25 | Fuel consumption signal (from ECU) | Open |
| 28 | Vehicle Speed Sensor (G22 input) | Open |
| **30** | **Terminal 86s / S-contact (Key inserted)** | **Connect to +12V DC** (prevents sleep) |

#### Green 32-Pin Connector (T32a) — Diagnostics & Immobilizer

| Pin # | Function | Bench Connection |
| :---: | :--- | :--- |
| 2 | Transponder Reader Coil 1 (D2) | RFID coil (optional for key testing) |
| 5 | W-Wire (Immobilizer communication to ECU) | Connect to ECU W-Line (if ECU on bench) |
| 7 | Right turn signal indicator | Open |
| 10 | Airbag warning lamp | Open |
| 17 | Transponder Reader Coil 2 (D2) | RFID coil (optional for key testing) |
| 18 | *CAN-High (Powertrain - unused on 919)* | *Not connected on 8N1919860C* |
| 19 | *CAN-Low (Powertrain - unused on 919)* | *Not connected on 8N1919860C* |
| **28** | **K-Line (Bidirectional Diagnostic Wire)** | **Connect to OBD-II Pin 7** |

#### Grey 32-Pin Connector (T32b) — Optional FIS/DIS Display

*Not required for basic power-up, diagnostics, or EEPROM dumping.*
- Pin 11: Display Enable (3-wire radio bus)
- Pin 12: Display Clock (3-wire radio bus)
- Pin 13: Display Data (3-wire radio bus)
- Pins 17, 18, 19: Trip computer stalk switch (Up, Down, Reset)

### 4.3 OBD-II Diagnostic Port Pinout

Standard J1962 female port (looking into the socket):

```text
    ┌──────────────────────────────────────────────┐
    │  \  1   2   3  [4] [5]  6  [7]  8  /         │
    │   \ 9  10  11  12  13  14  15 [16]/          │
    └──────────────────────────────────────────────┘
```

| OBD-II Pin | Description | Connection |
| :---: | :--- | :--- |
| **4** | Chassis Ground | Connect to Power Supply GND |
| **5** | Signal Ground | Connect to Power Supply GND |
| **7** | **K-Line (ISO 9141-2 / KWP1281)** | Connect to Cluster Green (T32a) Pin 28 |
| **16** | **Battery Power (+12V Constant)** | Connect to Power Supply +12V |

### 4.4 Bench Setup Using the OBD Splitter

Using a standard 16-pin OBD splitter makes building a bench rig easy without cutting cables:

1. **Step 1 (Power Supply Prep):** Set power supply to **12.5 V DC** with a **1.5 A current limit**. Do not switch power on yet.
2. **Step 2 (OBD Splitter Tap):**
   - Insert male Dupont pins into Female Port 1 of the OBD splitter:
     - Dupont wire to **OBD Pin 16** -> Power Supply `+12V`.
     - Dupont wire to **OBD Pin 4 & 5** -> Power Supply `GND`.
     - Dupont wire to **OBD Pin 7** -> Connect to Cluster Green (T32a) **Pin 28**.
3. **Step 3 (Cluster Power Wiring):**
   - Connect Power Supply `+12V` to Cluster Blue (T32) **Pin 1**, **Pin 23**, and **Pin 30**.
   - Connect Power Supply `GND` to Cluster Blue (T32) **Pin 9**, **Pin 24**, and **Pin 7**.
4. **Step 4 (Diagnostic Interface):** Plug the Blue KKL or Red VAG Commander cable into Female Port 2 of the OBD splitter.
5. **Step 5 (Power On):** Turn on the 12V power supply.
   - The cluster needles should twitch/home.
   - Odometer LCD and clock LCD will illuminate.
   - Normal idle bench current: **~350 mA to 650 mA** (higher if backlight is active).

---

## 5. Software Tools & Diagnostic Procedures

### 5.1 FTDI Driver Configuration (Critical Latency Setting)

VAG diagnostic protocols (especially KWP1281 baud-rate synchronization on Marelli clusters) fail if the USB FTDI buffer delay is too high:

1. Open **Device Manager** on Windows.
2. Expand **Ports (COM & LPT)** and right-click your FTDI cable (**USB Serial Port**).
3. Select **Properties** -> **Port Settings** -> **Advanced**.
4. Change **Latency Timer (msec)** from `16` down to **`1`**.
5. Set COM Port Number between `COM1` and `COM4` (for VCDS-Lite compatibility).

---

### 5.2 VAG K+CAN Commander 1.4 (EEPROM & PIN Extraction)

The Red VAG Commander cable communicates directly with the Motorola MCU via K-Line.

1. Connect the Red cable to the OBD splitter and launch **VAG K+CAN Commander 1.4**.
2. Select Connection Type: **K-Line**.
3. Target Selection:
   - In the target dropdown, choose **`07 Combined Instrument (Magneti Marelli Direct)`** or **`Magneti Marelli Audi TT / A3 / A4 / A6`**.
4. Click **`Open Channel`**.
   - If successful, the cluster identification string will display: `8N1919860C KOMBI+WEGFAHRS. M73 D29`.
5. Click **`Read EEPROM`**:
   - The software will execute a bootstrap routine in the 68HC12 RAM and stream the 64 KB memory block.
6. Click **`Save to File`** immediately to create a clean backup before attempting any write or modification.
7. The PIN / Login Code will be displayed in the software status window (4 or 5 digits).

---

### 5.3 VCDS-Lite (Diagnostics & Actuator Tests)

Using the Blue FTDI KKL cable:

1. In VCDS-Lite, go to **Options** -> select your COM port -> click **Test** -> Save.
2. Click **Select Control Module** -> **17 - Instruments**.
3. **Identification**:
   - VAG Number: `8N1 919 860 C`
   - Component: `KOMBI+WEGFAHRS. M73 D29`
   - Extra: `AUZ5Z0...`
4. **Output Tests (Function 03)**:
   - Run needle sweep tests (Speedometer, Tachometer, Coolant Temperature, Fuel Level).
   - Test warning buzzer and LCD display segments.
5. **Measuring Blocks (Function 08)**:
   - Group 001: Speed, Engine RPM, Oil Pressure Switch.
   - Group 002: Odometer reading, Fuel level (liters), Fuel sensor resistance.
   - Group 003: Coolant temp (°C), Ambient temp.
   - Group 050: Key status, Immobilizer authorization status.
6. **Immobilizer & Key Matching (Function 11 & 10)**:
   - Enter **Login 11** using the 5-digit PIN (e.g., `00811`).
   - Go to **Adaptation 10** -> **Channel 21** to program keys.
   - Clear learned values in **Channel 00**.

---

### 5.4 kw1281test (CLI Tool for Marelli Dumps)

[kw1281test](https://github.com/gmenounos/kw1281test) is an open-source cross-platform tool capable of reading Marelli M73 memory over K-Line:

```bash
# Wake up cluster at Address 17 and query SKC (PIN)
kw1281test.exe COM3 9600 17 GetSkc

# Dump cluster memory (68HC12 RAM/EEPROM)
kw1281test.exe COM3 9600 17 DumpMarelliMem 14336 2048
```

---

### 5.5 MPPS & ME7 Tools (Engine ECU Integration)

The 225HP TT uses a **Bosch ME7.5** engine ECU (e.g., `06A 906 032 AR / HJ` for APX).
- **MPPS**: Flash read/write over K-Line (boot mode or OBD on bench).
- **ME7EEPROM Tool**: Reads the 95040 SPI EEPROM in the ECU to extract the ECU's immobilizer PIN.
- On early APX (Immo 2), the cluster and ECU match when the PIN in both units matches or when adapted via Channel 50.

---

## 6. EEPROM Memory Architecture & Decoding Reference

Everything in this section was verified against **both** dumps in `EEPROM/` with
`tools/eeprom_tool.py` (see section 8). Items marked *unverified* are plausible
interpretations that have not been confirmed against a diagnostic tool.

### 6.1 What the 64 KB "direct read" actually contains

The 64 KB file is a dump of the whole 68HC912DG128 CPU address space as seen by the
bootstrap routine that VAG K+CAN Commander uploads. Only a small part of it is
non-volatile data you can meaningfully write back:

| CPU address | Size | Content | Persistent? |
| :--- | :--- | :--- | :--- |
| `0x0000 – 0x37FF` | 14 KB | RAM / working memory. The first 50 bytes are identical in both dumps (the tool's loader code). Contains live copies of odometer, key data, PIN etc. that the firmware rebuilds from EEPROM at power-up. | **No** (volatile) |
| `0x3800 – 0x3FFF` | **2 KB** | **Internal EEPROM** – part number, odometer ring buffer, immobiliser data, keys, VIN (920). This is what `kw1281test DumpMarelliMem 14336 2048` reads. | **Yes** |
| `0x4000 – 0xFFFF` | 48 KB | Internal flash – firmware (`V7.66ML` on the 919, `V8.51ML` on the 920), reset vector at `0xFFFE`. The 919 has `0x4000–0x7FFF` erased. | Yes, but not something to edit |

Consequences:

- The "static mileage registers" at `0x0080` (919) are RAM. On the 920 firmware the same
  live copy sits at `0x0F1A`. They are not inputs, they are outputs of the firmware.
- The only thing worth editing / writing back is `0x3800–0x3FFF`. Use
  `eeprom_tool.py extract` to get a 2 KB EEPROM-only image, and `inject` to put a 2 KB
  image back into a 64 KB file if your writing tool insists on the 64 KB format.
  The tool accepts both formats everywhere and always talks in CPU addresses.
- **No checksum was found.** A brute-force search for 8/16-bit sum, negated sum, XOR
  and common CRC-8/CRC-16 values adjacent to any 3–128 byte range, required to hold
  in both dumps, produced zero hits outside the odometer ring (where every hit is an
  artefact of the complement pairs). Integrity is instead provided by redundancy:
  the odometer ring has a bitwise-inverted twin, and the key block is stored three
  times. `eeprom_tool.py verify` checks these invariants.

#### Identification strings

```text
                    919 (TT_919_orig_252622km.bin)     920 (GB_920_dump.bin)
0x3A29 part number  8N1919860C  D29                    8N2920950A  D04
0x384A serial-like  ZAJUK3CD011                        RALMLL5C003
0x3B4E/89/C4 immo   (none in EEPROM)                   AUZ5Z0B3245205  (x3)
0x3BFD VIN slot     (0xFF)                             XXXXXXXXXXXXXXXXX (placeholder)
0x3C18 VIN slot     (0xFF)                             TRUZZZ8N8xxxxxxxx (donor car's VIN)
0xC003 firmware     V7.66ML   EP_2002                  V8.51ML   EP_2002   [flash]
```

`8N2` = right-hand-drive part number (UK donor). `D29`/`D04` is the data-set index shown by VCDS.

---

### 6.2 Odometer (Mileage) Decoding

The odometer is a **36-entry ring buffer of 4-byte big-endian kilometre values** advanced
every 2 km, stored twice:

```text
0x3899 – 0x3928   36 x 4 bytes   bitwise NOT of the plain ring (integrity twin)
0x3929 – 0x39B8   36 x 4 bytes   plain ring, big-endian km
```

(The previous version of this document listed `0x3928`; the plain ring starts at
`0x3929`, the byte at `0x3928` is the last byte of the inverted twin.)

Current mileage = the largest entry. The entry after it (circularly) is the oldest.

```text
919:  newest at index 11 (0x3955): 00 03 DA CE = 252,622 km ; oldest 252,552 km ; all even
920:  newest at index  6 (0x3941): 00 03 8E 47 = 233,031 km ; oldest 232,961 km ; all odd
```

Both parities occur in real clusters, so the firmware does not appear to care about it.
Immediately before the ring there is a 5-byte field (`0x3894`: `03 8E FF FF FF` / `03 6E FF 00 64`)
of unknown purpose; `set-odometer` leaves it alone. If a rewritten cluster misbehaves,
this is the first place to look.

The 920 is a UK (`8N2`, MPH) cluster, yet its ring is in kilometres (233,031 km ≈ 144,799 mi);
the miles value in RAM (`0x0F1E`: `00 02 35 BD` = 144,829) is derived by the firmware.

---

### 6.3 Immobiliser PIN / SKC (*unverified*)

```text
919  0x3880:  ff 03 2b 00 02 0f 05 ff 00 00 03 0a ff ff ff ff
920  0x3880:  ff ff 02 1e 00 ff 00 00 00 00 00 00 ff ff ff ff
```

Reading `0x3881–0x3882` as a big-endian PIN gives **00811** for the 919, with `0x3884 = 02`
matching the two keys stored in the key block. On the 920 the same pattern appears shifted by
one byte (`0x3882`: `02 1E` → **00542**). Neither value has been confirmed against what
VAG Commander / VCDS report, so treat them as candidates. `03 0A` appears in both dumps
(919 `0x388A`, 920 `0x3AB9`) and is probably a constant, not part of the PIN.

---

### 6.4 Transponder Key Data

Key data is stored three times. The copies are byte-identical in both dumps.

```text
919 (Immo 2): 16 bytes at 0x3A3A, 0x3A63, 0x3A8C (stride 41)
   [02 bc] 13 31 3a 5b 79 99 b4 d4 | 29 bd ad fc a9 42 41 43
            key 1 (8 bytes)          key 2 (8 bytes)
   RAM 0x02EB holds key 1 byte-reversed. The byte before copies 2/3 is 02 (key count?).

920 (Immo 3): 35 bytes at 0x3B4B, 0x3B86, 0x3BC1 (stride 59)
   02 | 0c 9c | "AUZ5Z0B3245205" | 12 20 3c 5b 7c 95 b0 20 00 00 | 5b ca 0a 8a | 5b ca 0e f6
   cnt   ?      immobiliser ID      ?                              key 1 ID?     key 2 ID?
```

The 920 additionally stores the donor vehicle's 17-character VIN at `0x3C18` and a
`XXXXXXXXXXXXXXXXX` placeholder at `0x3BFD`. Immo 3 compares cluster and ECU identity, so
these matter if the cluster is ever paired with an Immo 3 ECU (`eeprom_tool.py set-vin`).

---

## 7. Common Failure Modes of Jaeger M73 Clusters

The Magneti Marelli / Jaeger clusters in early Audi TTs are notorious for several well-documented hardware failure modes:

1. **Motorola MC68HC912 Internal Flash Degradation:**
   - **Symptom**: Needles jump randomly, speedometer/tachometer reading incorrectly, gauges completely failing below freezing temperatures (working again once the cabin warms up).
   - **Root Cause**: Deterioration of the internal flash memory cells inside the MC68HC912 MCU silicon.
   - **Remedy**: Reflashing with clean bin dump, replacing the MCU with a new mask set (e.g., `3K91D` / `0L01Y`), or swapping to a VDO cluster.
2. **Defective Stepper Motors:**
   - Fuel and temperature gauge stepper motors fail or stick.
3. **Center FIS/DIS Display Degradation:**
   - LCD horizontal and vertical pixel lines disappearing when warm.
   - Remedy: Soldering a replacement 88x64 display panel with flexible flat cable (FFC).
4. **Parasitic Battery Drain:**
   - MCU fails to enter low-power sleep state on Terminal 30, drawing 150–400 mA continuously when the car is parked.

---

## 8. Dump Tooling (`tools/eeprom_tool.py`)

A dependency-free Python 3 CLI that understands both the 64 KB direct-read format and the
2 KB EEPROM-only format. All addresses are CPU addresses (EEPROM = `0x3800–0x3FFF`).

```bash
# decode a dump (part number, odometer ring, immo/key/VIN data, unknown fields)
python3 tools/eeprom_tool.py info EEPROM/TT_919_orig_252622km.bin
python3 tools/eeprom_tool.py info EEPROM/GB_920_dump.bin

# structural sanity check (ring complement, 36-step sequence, key copies, reset vector)
python3 tools/eeprom_tool.py verify EEPROM/GB_920_dump.bin

# 64 KB -> 2 KB EEPROM image, and back into a 64 KB template
python3 tools/eeprom_tool.py extract EEPROM/GB_920_dump.bin 920_eeprom.bin
python3 tools/eeprom_tool.py inject  EEPROM/GB_920_dump.bin 920_eeprom.bin 920_full.bin

# compare dumps (any mix of 64 KB / 2 KB); read the cluster twice and diff to see what is volatile
python3 tools/eeprom_tool.py diff EEPROM/TT_919_orig_252622km.bin EEPROM/GB_920_dump.bin --eeprom-only

# carry the car's real mileage from the old cluster into the new one
python3 tools/eeprom_tool.py set-odometer EEPROM/GB_920_dump.bin --out 920_252622km.bin \
        --from-dump EEPROM/TT_919_orig_252622km.bin
#   ... or an explicit value; --match-parity keeps the cluster's existing odd/even ring parity
python3 tools/eeprom_tool.py set-odometer 920_eeprom.bin --out out.bin --km 252622 --match-parity

# write the car's VIN into the 920 VIN slots
python3 tools/eeprom_tool.py set-vin 920_252622km.bin --out 920_final.bin --vin TRUZZZ8N...

# low level
python3 tools/eeprom_tool.py hexdump EEPROM/GB_920_dump.bin --start 0x3880 --end 0x3A00
python3 tools/eeprom_tool.py strings EEPROM/GB_920_dump.bin
python3 tools/eeprom_tool.py set-bytes in.bin --out out.bin --at 0x3881 --hex "03 2b"
python3 tools/eeprom_tool.py checksum-scan EEPROM/TT_919_orig_252622km.bin EEPROM/GB_920_dump.bin

# regression tests against the dumps in EEPROM/
python3 -m unittest tools/test_eeprom_tool.py -v
```

`set-odometer` rewrites exactly 144 bytes (`0x3899–0x39B8`), keeps the ring pointer
(newest-entry index) where the cluster had it, regenerates the inverted twin, and refuses to
overwrite its input. `set-*` commands never touch RAM or flash.

---

## 9. Using the 920 (CAN / Immo 3) Cluster in the Early 919 Car

What the dumps tell us, and what they do not:

1. **Mileage** – transferable with `set-odometer` (section 8). This is the legitimate
   case of making a replacement cluster show the vehicle's true mileage; keep the
   original dump as evidence.
2. **Immobiliser generation mismatch** – the 919 car has an Immo 2 ECU that talks to the
   cluster over the W-line (green pin 5). The 920 cluster is Immo 3 and authorises the ECU over
   CAN. They cannot be matched to each other. Practical options:
   - flash the ME7.5 ECU immo-off (MPPS / ME7 EEPROM tools, section 5.5). The cluster's
     immobiliser lamp will keep flashing because it never sees a matching ECU, but the
     engine runs; or
   - fit an Immo 3 ECU and adapt it to the 920 cluster (needs the 920 PIN and the car's VIN
     in the cluster, `set-vin`).
   The donor's keys (`5b ca 0a 8a`, `5b ca 0e f6`) are in the 920 EEPROM; your own keys would
   need adapting via VCDS adaptation channel 21 with the 920's PIN.
3. **Wiring** – the 919 harness has nothing on green pins 18/19. Run a twisted pair from the
   ME7.5 powertrain CAN (CAN-H / CAN-L at the ECU or the ABS unit) to green pins 18/19.
   RPM, coolant temperature and consumption then arrive over CAN instead of blue pins 11/8/25.
   Whether the early APX ME7.5 software broadcasts the cluster-relevant frames is not
   verifiable from these dumps – test on the bench with the ECU before cutting the harness.
4. **Units / coding** – `8N2920950A` is an RHD MPH dial. Country/units are part of the
   cluster coding (VCDS, 17-Instruments, function 07), not the EEPROM fields documented here.

---

## 10. ME7.5 Engine ECU Flash (`ECU/`)

Engine ECU of the car: Bosch ME7.5, VAG part `8N0906018AE`, software `0001`, Bosch
`0261207030` / `1037354178`, EPK `36/1/ME7.5/5/4018.10//F19w2/Dst01o//`, engine
`1.8L R4/5VT` (APX 225 HP). The flash is a 512 KB AM29F400 image; the C167 CPU sees it at
`0x800000`–`0x87FFFF`. All offsets below are **file offsets** (subtract `0x800000` from
any C167 address you see in other tools).

### 10.1 The three files

| File | MD5 | Content |
|---|---|---|
| `8N0906018AE 0001.Bin` | `7602ca25…1361` | original read of the car's ECU, all 43 checksums valid |
| `8N0906018AE_0001_MOD_XTUNER_SAIOFF_done.Bin` | `f0cabb1d…34af` | original + 2 data bytes changed + 20 checksum bytes corrected |
| `8N0906018AE_dyno-chiptunning.stage1.Bin` | `f0cabb1d…34af` | **byte-identical to the SAI-off file** |

So the file saved under the "stage1" name is *not* a stage 1 tune – it is the same SAI-off
file again (`cmp` reports no difference). The dyno-chiptuningfiles answer ("soft tuning
inside, only a few maps changed, no SAI off inside") therefore describes a file that is not
in this directory. Their remark "only a few maps changed" is also what one expects when they
look at the *SAI-off* file (2 bytes changed) – it is possible they diffed that upload against
their own stock and simply reported on it. When the real stage 1 arrives, drop it into
`ECU/` and run `tools/me7_tool.py diff` against the original; every non-checksum run it
reports is a tuned map.

Their other two remarks are correct and matter for the final firmware:

* **"Immo is in the eeprom, we can't see that."** – On ME7.5 the immobiliser state lives in
  the 95040 SPI EEPROM on the ECU board, not in the 512 KB flash. Immo-off is therefore an
  *EEPROM* edit (ME7EEPROM / Galletto / MPPS EEPROM read + patch), independent of whatever
  flash file is used. None of the three flash files touches it, and no flash-only service can
  deliver it.
* **"No SAI off inside."** – consistent with the file analysis: a tuning house that starts
  from a stock file will not carry the XTuner bits across. The SAI-off change has to be
  re-applied on top of whatever stage 1 file is delivered (see 10.4).

### 10.2 Flash layout and checksum structure

```
0x00000-0x0FFFF   boot loader / ROMSYS header @0x08000
0x10000-0x1FFFF   calibration: maps, curves, scalar parameters, ID strings
                   0x1432A EPK, 0x144D5 Bosch HW/SW numbers, 0x15317 VAG part number
                   0x1FC00-0x1FE1F multipoint checksum table (34 x 16-byte descriptors)
0x20000-0x7FFFF   program code; main program checksum pair @0x7FFE0
```

The ECU validates the image with five independent mechanisms. All of them are found and
recomputed by `me7_tool.py`, and the result was verified byte-for-byte against
[ME7Sum](https://github.com/nyetwurk/ME7Sum) (the open-source reference checker):

| Mechanism | Covers | Stored at | Algorithm |
|---|---|---|---|
| ROMSYS startup / param-page / program-pages | a few words each | `0x08038`, `0x1A8AE`, `0x0803C` | 16-bit word sums |
| Main data CRC32, zone 1/2/3 | `0x10000-0x13FFF`, `0x14300-0x17F67`, `0x18191-0x1FBFF` | `0x67058`, `0x6705E`, `0x67064` | standard CRC-32 |
| Background checksums | `0x24000-0x26567`, `0x14000-0x14243` | `0x1422C`, `0x1423C` (+inverse) | 16-bit word sum |
| Main program checksum | `0x00000-0x0FBFF` + `0x20000-0x7FFFF` | `0x7FFE0` (+inverse) | 16-bit word sum |
| Multipoint (34 blocks of 16 KB) | whole flash | `0x1FC00` table, value+inverse each | 16-bit word sum |

A calibration change therefore always ripples into: one multipoint block, one CRC32 zone,
and (because the multipoint table itself is inside block 10 and the CRC values are inside
block 28 of the program area) one or two further multipoint blocks plus the main program
checksum. Three calibration windows (`0x14000-0x142FF`, `0x17F68-0x18190`, `0x1FC00-0x1FFFF`)
are not covered by any CRC32 zone, only by multipoint sums.

### 10.3 What the "SAI off" modification is

`me7_tool.py diff` of original vs. XTuner file:

```
0x14EE9-0x14EEA [2] calibration          f8 33 -> fb 3f
0x1FC78.. 0x1FC7D   checksum: multipoint 8   (block 0x14000-0x17F67)
0x1FDB8.. 0x1FDBE   checksum: multipoint 28  (block 0x64000-0x67FFF, holds the CRC values)
0x6705E-0x67061     checksum: CRC32 zone 2
0x7FFE0-0x7FFE6     checksum: main program checksum
```

The only data change is two bytes at `0x14EE9`/`0x14EEA`. They are not a map: the code
reaches them as elements 5 and 6 of a 7-byte array at `0x14EE4`
(`MOVB RLx,[R4+#0x4EE4]`, R4 loaded from a RAM variable, 7 call sites around
`0x46A98`–`0x47512`), and the patch *sets exactly four bits* (`F8→FB` = bits 0,1;
`33→3F` = bits 2,3). That is the fingerprint of a per-fault-path bitmask edit, i.e. the four
secondary-air DTCs (pump relay, valve, flow) are suppressed so the missing pump never sets
the MIL. The pump output driver and the SAI control logic are untouched; the ECU still
"runs" the SAI function, it just no longer reports it. This is the usual commercial
"SAI off" and is adequate for a removed pump, but it is **not** a map-level deactivation.

Re-creating it from the original with the tool gives a file identical to the XTuner one:

```
python3 tools/me7_tool.py patch "ECU/8N0906018AE 0001.Bin" --out /tmp/saioff.bin --at 0x14EE9 --hex "fb 3f"
# -> md5 f0cabb1d52be19665bfbf3b26e7934af  (== XTuner file)
```

### 10.4 Plan for the final firmware

Target: stage 1 + SAI off + immo off + pop-and-bang. Where each part has to be done:

| Item | Where | How |
|---|---|---|
| Stage 1 | flash, calibration maps | buy/obtain the stage 1 file for *this* `8N0906018AE 0001` software; verify with `info` that ID strings and checksums are intact |
| SAI off | flash, 2 bytes @`0x14EE9` | `apply-diff ORIG SAIOFF STAGE1 --out final.bin` ports the two bytes onto the stage 1 file and re-signs every checksum; conflicts (stage 1 touching the same bytes) are reported and abort |
| Immo off | ECU **EEPROM** (95040), not flash | separate read/patch of the EEPROM with ME7EEPROM-class tools; the cluster-side consequences are in section 9 |
| Pop & bang | flash, several maps + code conditions | not a byte patch. On ME7.5 it needs an overrun-fuel-cutoff strategy change (keep injecting on decel with retarded ignition, usually gated by rpm/throttle/temperature). This is a calibration job with the map definitions (A2L/XDF for `8N0906018AE`), not something to derive from these three files. Ask the tuning house for it on top of stage 1, or do it with a proper definition file; the tool's `maps`/`map` commands help locate candidate tables but do not name them |

Whatever the combination, run `checksums` on the final file before flashing and keep the
original read untouched.

---

## 11. Flash Tooling (`tools/me7_tool.py`)

Dependency-free Python 3.8+. All addresses are file offsets, hex accepted (`0x14EE9`).

```bash
# identify the image, summarise all checksums, and print the current overrun/SAI settings
# (values come from ECU/Profiles/<part>.xdf; --xdf PATH to choose another, --no-settings to skip)
python3 tools/me7_tool.py info "ECU/8N0906018AE 0001.Bin"

# every checksum with ranges, stored vs. computed values; non-zero exit on any error
python3 tools/me7_tool.py checksums FILE
# correct them (never overwrites the input unless --in-place)
python3 tools/me7_tool.py fix FILE --out fixed.bin

# annotated diff: each run is labelled 'calibration' / 'program code' / 'checksum: <which>';
# --xref adds the code locations that read changed calibration bytes
python3 tools/me7_tool.py diff ORIG MOD --xref

# port the data changes BASE->MOD onto TARGET (checksum runs skipped, sums rebuilt)
python3 tools/me7_tool.py apply-diff ORIG SAIOFF STAGE1 --out final.bin

# raw patch with automatic checksum correction
python3 tools/me7_tool.py patch ORIG --out out.bin --at 0x14EE9 --hex "fb 3f"

# what covers an offset: region, DPP address, checksums that include it
python3 tools/me7_tool.py where FILE 0x14EE9
# region / checksum map, including the calibration windows without CRC32 cover
python3 tools/me7_tool.py regions FILE [--all]

# code references to calibration bytes (C167 mem / indexed / pointer operands)
python3 tools/me7_tool.py xref FILE 0x14ED0 --end 0x14EEA --quiet

# tables, derived from the lookup-routine call sites (R12/R13 pointers before CALLS)
python3 tools/me7_tool.py maps FILE --min-cells 96          # list
python3 tools/me7_tool.py maps FILE --routines              # which lookup routines exist
python3 tools/me7_tool.py map  FILE 0x14EEB                 # print one (axes auto-resolved)
python3 tools/me7_tool.py map  FILE 0x15DD4 --axes 0x14450 --signed --factor 0.75

python3 tools/me7_tool.py hexdump FILE --start 0x14EC0 --end 0x14F00
python3 tools/me7_tool.py strings FILE --min 8

# regression tests (checksum values, XTuner reproduction, ME7Sum-equivalent fixes)
python3 -m unittest tools/test_me7_tool.py -v
```

Table detection notes: ME7 lookup routines receive the data pointer in R12 and, for
"group-axis" maps, the shared `[nx][x…][ny][y…]` axes block in R13; small 2-D maps are
"embedded" (`[nx][ny][x…][y…][data]`). The `maps` command reconstructs tables from those call
sites, so it only lists what the code actually uses; cell width (8/16 bit) of group-axis maps
is inferred from the data and can be overridden with `--word`. Axis/cell scaling factors are
not known without a definition file – values are printed raw unless `--factor/--offset/--xf/--yf`
are given.
