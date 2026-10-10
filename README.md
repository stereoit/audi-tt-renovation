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
  - [6.1 Dump Overview (`TT_919_orig_252622km.bin`)](#61-dump-overview-tt_919_orig_252622kmbin)
  - [6.2 Odometer (Mileage) Decoding](#62-odometer-mileage-decoding)
  - [6.3 Immobilizer PIN / SKC (Secret Key Code)](#63-immobilizer-pin--skc-secret-key-code)
  - [6.4 Transponder Key Data & Immobilizer Status](#64-transponder-key-data--immobilizer-status)
- [7. Common Failure Modes of Jaeger M73 Clusters](#7-common-failure-modes-of-jaeger-m73-clusters)
- [8. Dump Inspection Helper Script](#8-dump-inspection-helper-script)

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

### 6.1 Dump Overview (`TT_919_orig_252622km.bin`)

Located in `EEPROM/TT_919_orig_252622km.bin`:

- **File size**: `65,536` bytes (64 KB).
- **Active memory areas**:
  - `0x0000 - 0x3C00`: Microcontroller working registers, static configuration, odometer mirrors, and crypto buffers.
  - `0x3800 - 0x3B50`: Internal EEPROM non-volatile data (cluster part number, VIN, transponders, PIN).
  - `0x8000 - 0x10000`: Microcontroller internal operating firmware and diagnostic lookup tables.

#### Identification Strings in the Dump

```text
Offset 0x3A29:  8N1919860C  D29
Offset 0xC003:  V7.66ML   EP_2002
Offset 0xC463:  KOMBI+WEGFAHRS. M73
Offset 0xC47C:  Ident.-Nr. Wegfahrs.: AUZ5Z0...
Offset 0x384A:  ZAJUK3CD011
```

---

### 6.2 Odometer (Mileage) Decoding

In the Magneti Marelli / Jaeger MC68HC912 architecture, mileage is recorded using a combination of **static summary registers** and a **rolling ring buffer** that prevents EEPROM flash wear.

#### Static Mileage Registers

The mileage is stored in **hexadecimal kilometers**:

$$\text{252,622 km} \longrightarrow \mathbf{0x03DACE}$$

```text
Offset 0x0080:  85 00 03 DA CE 00 03 DA CE 00 00 00 A0 00 00 00
                      └────────┘  └────────┘
                     252,622 km  252,622 km (Redundant copy)
```

Directly preceding the kilometer value is the mileage stored in **miles**:

$$\text{252,622 km} \div 1.609344 = 156,972.1 \text{ miles} \approx 156,973 \text{ miles} \longrightarrow \mathbf{0x02654D}$$

```text
Offset 0x007C:  02 65 4D
                └──────┘
              156,973 miles
```

#### Rolling Ring Buffer (Wear-Leveling Odometer)

From offset `0x3928` to `0x39B7`, the cluster maintains a 36-entry circular log updating every 2 kilometers:

```text
Offset 0x3928:  00 03 DA B8  (252,600 km)
Offset 0x392C:  00 03 DA BA  (252,602 km)
Offset 0x3930:  00 03 DA BC  (252,604 km)
Offset 0x3934:  00 03 DA BE  (252,606 km)
Offset 0x3938:  00 03 DA C0  (252,608 km)
...
Offset 0x3954:  00 03 DA CE  (252,622 km) ◄── LATEST LOG ENTRY
Offset 0x3958:  00 03 DA 88  (252,552 km) ◄── Previous cycle wrap-around
```

---

### 6.3 Immobilizer PIN / SKC (Secret Key Code)

The Secret Key Code (SKC) is stored in the EEPROM block immediately preceding the key-count sequence marker `00 [keys] 0F`:

```text
Offsets 0x03ED and 0x3881:
03 2B 00 02 0F 05
└─┬─┘ └──┬──┘
  │      └── Marker: 2 keys adapted (0x02)
  └───────── PIN Code: 0x032B
```

- **Big-Endian representation**: `0x032B` = `811` $\rightarrow$ **VCDS PIN: `00811`**
- **Byte-swapped (Little-Endian) representation**: `0x2B03` = `11011`

*Note: For VCDS login, always pad to 5 digits (e.g. `00811`).*

---

### 6.4 Transponder Key Data & Immobilizer Status

The cluster uses **Megamos Crypto 48 (ID48)** transponder chips.

In `0x3A39–0x3A9F`, the cluster stores 3 identical redundant 16-byte cryptographically adapted key blocks:

```text
Block 1 (0x3A39):  02 BC 13 31 3A 5B 79 99 B4 D4 29 BD AD FC A9 42
Block 2 (0x3A62):  02 13 31 3A 5B 79 99 B4 D4 29 BD AD FC A9 42 41
Block 3 (0x3A8B):  02 13 31 3A 5B 79 99 B4 D4 29 BD AD FC A9 42 41
```

The mirror data at `0x02EB` contains the byte-inverted key sequence `D4 B4 99 79 5B 3A 31 13`.

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

## 8. Dump Inspection Helper Script

To inspect, decode, and verify any bin dump from this cluster, run the following Python script from the repository root:

```python
#!/usr/bin/env python3
"""
Audi TT 8N Jaeger Cluster (8N1919860C) Dump Inspector
"""
import sys
from pathlib import Path

def inspect_dump(bin_path: str):
    data = Path(bin_path).read_bytes()
    print(f"=== Inspecting: {bin_path} ===")
    print(f"File Size: {len(data)} bytes ({len(data)//1024} KB)\n")

    # Part Number
    pn = data[0x3A29:0x3A3B].decode('ascii', errors='replace').strip()
    print(f"Part Number / Software : {pn}")

    # Mileage
    km = int.from_bytes(data[0x0082:0x0085], 'big')
    miles = int.from_bytes(data[0x007C:0x007F], 'big')
    print(f"Odometer Reading (KM)  : {km:,} km (Hex: {km:06X})")
    print(f"Odometer Reading (MI)  : {miles:,} mi (Hex: {miles:06X})")

    # PIN / SKC
    pin_be = (data[0x3881] << 8) | data[0x3882]
    pin_le = (data[0x3882] << 8) | data[0x3881]
    keys = data[0x3884]
    print(f"Adapted Keys           : {keys}")
    print(f"Immo PIN (Big-Endian)  : {pin_be:05d} (0x{pin_be:04X})")
    print(f"Immo PIN (Little-End)  : {pin_le:05d} (0x{pin_le:04X})")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "EEPROM/TT_919_orig_252622km.bin"
    inspect_dump(path)
```

Run with:
```bash
python3 -c "
from pathlib import Path
data = Path('EEPROM/TT_919_orig_252622km.bin').read_bytes()
km = int.from_bytes(data[0x0082:0x0085], 'big')
pin = (data[0x3881] << 8) | data[0x3882]
print(f'Part: {data[0x3A29:0x3A3B].decode(errors=\"ignore\").strip()}')
print(f'KM: {km:,} km | PIN: {pin:05d}')
"
```
