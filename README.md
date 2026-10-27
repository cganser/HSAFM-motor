# HSAFM-motor

This repository contains the software and build documentation for a Raspberry Pi 5-based motor controller for high-speed atomic force microscopy (HS-AFM). It supports motor approach and retraction using hardware PWM, acquires analog signals with Digilent DAQ HATs, and includes assembly instructions and 3D-printable case files.

## Assembly and hardware

- [Read the assembly instructions](documentation/assembly-instructions.md)
- [Raspberry Pi setup, MCC installation, and I/O assignments](documentation/raspberry-pi-setup-and-io.md)
- [Browse assembly photos and diagrams](documentation/assembly-images/)
- [Browse 3D printable models and parts list](documentation/3D_printables/README.md)

## Parts to purchase

### Main components

- 1 × [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/) — Broadcom BCM2712 64-bit quad-core Arm Cortex-A76 CPU at 2.4 GHz; available with 1, 2, 4, 8, or 16 GB RAM.
- 1 × [Raspberry Pi 5 Active Cooler](https://www.raspberrypi.com/products/active-cooler/) — clip-on anodised aluminium heatsink with temperature-controlled blower; powered from the Pi's 5 V fan header and supports PWM speed control with tachometer feedback.
- 1 × [7-inch touchscreen for Raspberry Pi 5](https://www.dfrobot.com/product-2193.html) — DSI interface, 800 × 480 resolution, 5-point capacitive touch; powered by the Raspberry Pi.
- 1 × Raspberry Pi 5 power cable
- 1 × [Digilent MCC118 AD board](https://digilent.com/shop/mcc-118-128-voltage-measurement-daq-hat-for-raspberry-pi/?srsltid=AYCX9vLv1VeI8d_CGI8nQOB5uJKWOgXHKdIYVjeo68UJ_gsXPLztD-P5) — 8 single-ended voltage inputs, ±10 V range, 12-bit resolution, up to 100 kS/s.
- 1 × [Digilent MCC152 DA board](https://digilent.com/shop/mcc-152-voltage-output-and-dio-daq-hat-for-raspberry-pi/?srsltid=AYCX9vKHoXEmsueneZcxIjn8YsESX0VM1GmY_GtYRbXAkFuEdi52xR4H) — two 12-bit analog outputs (0–5 V, up to 5 kS/s) and eight digital I/O lines. Force-curve acquisition uses these outputs; the current application also initializes the MCC152 at startup, so it is required to run the application as written.
- 1 × Cherry MX style keyboard switch (Raspberry Pi 5 main switch)
- 1 × [Buzzer](https://jp.rs-online.com/web/p/piezo-buzzers/0102773) — RS PRO continuous-tone, externally driven buzzer; 2 kHz, 70 dB, 3–25 Vp-p, through-hole PCB pins.
- 1 × [SN74HC125 buffer IC](https://www.ti.com/product/SN74HC125) — four independent non-inverting buffers with three-state outputs; 2–6 V supply range.
- 4 × screws to connect the touchscreen to the case (check the touchscreen datasheet for the required screw type and size)

### Connectors

- 12 × BNC connectors
- 1 × 15-pin female D-sub connector
- 1 × 15-pin male D-sub connector

### Storage — choose one option

- 1 × 128 GB microSD card, **or**
- 1 × [256 GB Raspberry Pi SSD](https://www.raspberrypi.com/products/ssd/) — NVMe 1.4, M.2 2230 form factor, PCIe Gen 3 compliant — and 1 × [Waveshare PCIe to M.2 Mini adapter](https://www.waveshare.com/pcie-to-m.2-mini.htm) — for Raspberry Pi 5; supports NVMe M.2 2230/2242 drives and PCIe Gen 2/3.

### Optional cooling

- 1 × [case fan](https://jp.rs-online.com/web/p/axial-fans/1442032) (recommended, but optional) — 5 V DC axial fan, 30 × 30 × 10.2 mm, 80 mA, 9,000 rpm.

> **Note:** Wires are not included in this parts list. Refer to the [assembly instructions](documentation/assembly-instructions.md) for wiring details.

## Run on Raspberry Pi 5

Install the Digilent DAQ HAT library and `RPi.GPIO` using the [Raspberry Pi setup guide](documentation/raspberry-pi-setup-and-io.md). The setup script creates a virtual environment that can also see these system-installed hardware libraries, then installs the Python packages listed in `requirements.txt`.

```bash
./scripts/setup_venv.sh
./scripts/run_motor_control.sh
```

The launcher runs `motor_control.py` with the project's virtual environment and sets the repository as the working directory, so `settings.dat` is found consistently.

## License

Software is licensed under GPL-3.0-or-later, hardware design files under
CERN-OHL-S-2.0, and project documentation and media under CC BY 4.0. See
[LICENSE](LICENSE) and [LICENSES.md](LICENSES.md) for details.
