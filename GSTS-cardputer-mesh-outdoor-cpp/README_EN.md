# Cardputer Mesh Outdoor App — User Guide (English)

This guide covers only **System Operations** and **System Settings**. Hardware: M5Stack Cardputer Mesh Kit (K152) with integrated GNSS (ATGM336H) and LoRa (SX1262 / Meshtastic).

---

## 1. System Operations

### 1. Boot Splash Screen
On boot the **GSTS** splash animation plays first: the four letters fade in one by one (about the first 0.5 s), then hold while hardware initializes, and finally a top-to-bottom curtain wipe transitions into the main UI. Total duration is about 2 seconds.

### 2. Main (Idle) Screen
Shows current GNSS fix and system status:
- **Latitude / Longitude**: labels on the left, values on the right, arranged horizontally.
- **Altitude / Speed / HDOP**: shown on a single horizontal line.
- Top bar: recording status (REC), satellite count (SAT), SD card status, LoRa status (LR), battery percentage.
- While searching for a fix it shows "正在搜星…" (Searching for satellites…).

### 3. Key Operations

| Key | Function | Notes |
|-----|----------|-------|
| **R** | Start / Stop track recording | Press again to stop; filename is based on local time (UTC+8). |
| **Fn + L** | Arm SOS | Hold Fn and press L to enter the "armed" state (a red banner appears at the top). |
| **S** (double-press) | Trigger SOS | Must be **double-pressed within 2 s after Fn+L arming**; a single S does nothing (anti-misclick). |
| **ESC / ` (backtick)** | Cancel SOS | Stops the distress broadcast and disarms; track recording continues. |
| **T** | Toggle Track View | Switches the pure-vector track map (no map tiles loaded). |
| **Any key** | Wake screen | After screen-off, any key wakes it (the key itself is not acted upon). |

### 4. Screens & States

- **Idle screen**: location and system status (see above).
- **Recording screen**: shows track point count, distance, current speed, altitude, plus the record interval and GPX file path.
- **Track View screen**: draws the recorded track as vector lines with start (green) / end (red) markers and a north indicator at top-right; bottom shows point count, distance, and extent (meters). With fewer than 2 points it shows "轨迹点不足" (Not enough track points).
- **SOS screen**: red flashing background showing "正在救援频道循环广播…" (Broadcasting on rescue channel…), sent count, next-send countdown, current coordinates and satellite count; press ESC to cancel.

### 5. Power Saving (Auto Screen-Off)
- In non-SOS modes, **30 s without a key press** turns the backlight off (GNSS / recording / LoRa keep running); any key wakes it.
- **SOS mode forces the screen to stay on** at all times.

### 6. SOS Broadcast Content
Broadcasts in a loop on the rescue channel (Meshtastic primary channel, empty PSK / unencrypted), every 15 s by default:
- With a fix: `TEXT` message and `POSITION` map-pin packets are sent alternately (a stock Meshtastic node can display the location directly).
- Without a fix: only the TEXT distress message is sent.

### 7. Track Recording Output
The track is written to the microSD card in real time as **GPX 1.1**, e.g. `/track_YYYYMMDD_HHMM.gpx`. The file stays a valid GPX after every point is appended, so it can be imported directly into mapping software.

---

## 2. System Settings (config.h)

All tunable parameters live in `src/config.h`. Recompile and reflash after changing.

### Track & GNSS
| Parameter | Default | Description |
|-----------|---------|-------------|
| `RECORD_INTERVAL_S` | `5` | Seconds between recorded track points. |
| `GNSS_BAUD` | `115200` | GNSS serial baud rate (try 9600 if no data). |
| `GNSS_RX_PIN` / `GNSS_TX_PIN` | `15` / `13` | ESP32 ↔ GNSS serial pins. |

### SPI / LoRa Hardware Pins
| Parameter | Default | Description |
|-----------|---------|-------------|
| `PIN_SPI_SCK` / `PIN_SPI_MISO` / `PIN_SPI_MOSI` | `40` / `39` / `14` | Shared SPI bus pins (LoRa + SD). |
| `PIN_SD_CS` | `12` | SD card chip select. |
| `PIN_LORA_NSS` / `PIN_LORA_DIO1` / `PIN_LORA_RST` / `PIN_LORA_BUSY` | `5` / `4` / `3` / `6` | SX1262 control pins. |
| `IOE_I2C_ADDR` | `0x43` | Antenna-switch IO expander address (P0 high = antenna on). |

### LoRa / Meshtastic (CN Region)
> `LORA_FREQ` must match your stock Meshtastic node (see `meshtastic --info` Freq). CN formula: `470.0 + slot*0.125` MHz.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `LORA_FREQ` | `470.125f` | Frequency (MHz, default slot=1; change to match). |
| `LORA_BW` | `250.0f` | Bandwidth (kHz, CN LONG_FAST). |
| `LORA_SF` | `11` | Spreading factor. |
| `LORA_CR` | `5` | Coding rate (4/5). |
| `LORA_POWER` | `17` | TX power (dBm, CN max 19). |
| `LORA_PREAMBLE` | `16` | Preamble length (Meshtastic fixed 16). |
| `LORA_SYNCWORD` | `0x2B` | Sync word (Meshtastic, SX126x expands to 0x24B4). |

### Identity / SOS
| Parameter | Default | Description |
|-----------|---------|-------------|
| `MY_NODE_ID` | `0x11223344UL` | Custom 32-bit node ID (non-zero). |
| `CHANNEL_INDEX` | `0` | Channel index (0 = primary channel, empty PSK rescue channel). |
| `SOS_INTERVAL_S` | `15` | SOS re-broadcast interval (seconds). |
| `SOS_HOP_LIMIT` | `5` | Max relay hop limit. |
| `SOS_TEXT` | `"SOS! Need rescue 需要救援"` | SOS text message content. |

### SOS Anti-Misclick (Two-Step Trigger)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `SOS_ARM_TIMEOUT_MS` | `5000` | Auto re-lock timeout after Fn+L arming (ms). |
| `SOS_DOUBLE_MS` | `800` | Max gap between the two S presses (ms); expires → recount. |

### Power Saving (Auto Screen-Off)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `SCREEN_BRIGHTNESS` | `80` | Normal backlight brightness (0–255). |
| `SCREEN_OFF_MS` | `30000` | Idle time before screen-off (ms); SOS mode forces always-on. | |
