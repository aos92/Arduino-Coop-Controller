# Arduino Coop Controller – CC2540 / HM-10 BLE 

An automated **Smart Coop Controller** system powered by an **Arduino Mega**, equipped with an HM-10 / CC2540 Bluetooth Low Energy (BLE) module. This system is designed to automate essential functions inside an animal enclosure (such as poultry or other livestock) based on solar schedules (*TimeLord*), temperature and humidity sensors, as well as remote manual control via a smartphone over BLE.

---

## 🛠️ Key Features

1. **Automatic Coop Door**
   - Automatically opens and closes the coop door based on sunrise and sunset times using the **TimeLord** library (calculated via latitude/longitude coordinates).
   - Equipped with limit switches (stop pins) for upper and lower travel limits and manual buttons.
2. **Smart Lighting**
   - Regulates supplemental lighting based on required daily light hours (*photoperiod control*) to maximize productivity.
3. **Automatic Feeder**
   - Schedules periodic feeding times (morning, afternoon, evening, night) or triggers manually with programmed durations.
4. **Automatic Watering**
   - Controls drinking water supply using a solenoid valve combined with a float switch.
5. **Climate Control / Fan System**
   - Monitors ambient coop temperature using a DS18B20 sensor (OneWire) and automatically activates fans if temperatures exceed safe limits.
6. **Bluetooth Low Energy (BLE) Connectivity**
   - Supports wireless communication using the HM-10 module to monitor system status and send control commands directly from a smartphone app.

---

## 📌 Hardware Wiring Configuration

| Component | Arduino Mega Pin | Description |
| :--- | :--- | :--- |
| **HM-10 TX** | Pin 19 (RX1) | Hardware Serial Line 1 |
| **HM-10 RX** | Pin 18 (TX1) | Hardware Serial Line 1 *(Use a 3.3V voltage divider)* |
| **HM-10 VCC** | 3.3V | BLE module power supply |
| **HM-10 GND** | GND | Common Ground |
| **Light Toggle Button** | Pin 2 | Manual light control button |
| **Water Button** | Pin 3 | Manual water control button |
| **Door Relay Close** | Pin 4 | Door motor relay (Close) |
| **Door Relay Open** | Pin 5 | Door motor relay (Open) |
| **Air Temp Sensor** | Pin 6 | DS18B20 temperature sensor (OneWire) |
| **Light Relay** | Pin 8 | Coop lighting relay |
| **Feeder Relay** | Pin 9 | Feeder motor relay |
| **Fan Relay** | Pin 10 | Ventilation fan relay |
| **Water Solenoid Relay**| Pin 11 | Water solenoid valve relay |
| **Door Open Switch** | A0 | Door open limit switch |
| **Door Close Switch** | A1 | Door close limit switch |
| **Feed Button** | A2 | Manual feeder button |
| **Water Float Switch** | A3 | Water level float switch |
| **Door Close Stop** | A6 | Door close analog/stop sensor |
| **Door Open Stop** | A7 | Door open analog/stop sensor |

---

## 📡 BLE Command Format

The module receives commands through serial BLE communication prefixed with a question mark (`?`):

* `?water=1` / `?water=0` : Manually turn the water system on or off.
* `?light=1` / `?light=0` / `?light=toggle` : Control coop lighting.
* `?feed` : Manually trigger the feeding system.
* `?door=open` / `?door=close` : Open or close the coop door.
* `?fan=1` / `?fan=0` / `?fan=toggle` : Control the ventilation fan.
* `?status` : Request real-time status of all devices in JSON format.
* `?bleinfo` : Display MAC address, name, RSSI, and battery info of the BLE module.
* `?digitalread=pin` : Read the digital status of a specific pin.
* `?analogread=pin` : Read the analog value of a specific pin.
* `?digitalwrite=pin;v` : Set digital value (`v = 0/1`) on a specific pin.
* `?pinmode=pin;mode` : Configure pin mode (`INPUT/OUTPUT`).

---

## 📦 Required Libraries

Make sure you have installed the following libraries in your Arduino IDE:
- `Wire.h` (Built-in)
- `OneWire.h`
- `TimeLord`
- `Time` / `TimeLib`
- `TimeAlarms`
- `DS1307RTC.h` (or appropriate RTC driver)
- `HM_10_BLE.h`

---

## 🚀 Getting Started

1. Wire the components according to the pin mapping table above.
2. Upload this program code to the **Arduino Mega** using Arduino IDE or ArduinoDroid.
3. Connect your smartphone to the BLE device with the default name `COOP_BLE`.
4. Monitor and control the coop via BLE commands or let the system run automatically based on RTC schedules and sensors.
