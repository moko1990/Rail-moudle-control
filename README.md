# Rail-moudle-control
# Rail Control and Resistance Meter with Arduino from Python Interface

This project integrates a **Python GUI (PySide6)** with an **Arduino Mega 2560** to control a **linear rail system** and measure the resistance of a polymer sample in real time.  
The GUI communicates with the Arduino via **serial port**, sending control commands and receiving sensor data for live plotting.

---

## 🔹 Features
- **Motor Control:** Move a stepper motor along a linear rail with adjustable speed (Fast, Smooth, Slow).
- **Polymer Resistance Measurement:** Measure polymer sample resistance via **ADS1110 ADC** and display results.
- **Real-Time Plots:** GUI provides 4 visualization tabs:
  1. Voltage vs Time  
  2. Resistance vs Time  
  3. Voltage vs Displacement  
  4. Resistance vs Displacement  
- **Reference Voltage Control:** Output voltage via **MCP4725 DAC**.  
- **OLED Display:** Show system data on SH1106 OLED screen.  
- **User Interface:** Built with **PySide6** (Qt for Python).  

---

## 🔹 Hardware Setup

**Controller:** Arduino Mega 2560  
**Communication:** Serial, 250000 baud  

| Module / Component       | Arduino Mega Pin(s) |
|--------------------------|----------------------|
| Stepper Motor Driver     | DIR=35, STEP=34, ENABLE=30, M0=31, M1=32, M2=33 |
| Limit Switches           | START=3, DEST=2 |
| Buzzer                   | 38 |
| Buttons                  | ORIGIN=44, STOP=45, LEFT=46, RIGHT=47 |
| Multiplexer CD74HC4067   | S0=9, S1=12, S2=7, S3=6, EN=8 |
| OLED SH1106 (SPI)        | DC=10, CS=53, RESET=11 |
| ADC ADS1110              | I2C (Addr=0x48) |
| DAC MCP4725              | I2C (Addr=0x60) |

---

## 🔹 Software Requirements
- **Arduino IDE** (for `.ino` file upload)  
- **Python 3.9+**  
- Python libraries:  
  ```bash
  pip install PySide6 pyserial matplotlib
