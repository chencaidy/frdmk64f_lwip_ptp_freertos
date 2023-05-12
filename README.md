# IEEE1588v2 PTP time server for FRDM-K64F

## Overview

![logo](docs/images/K64F.png)

## Feature

### Precision Time Protocol

* Support 3-Layer UDP IEEE1588v2 protocol
* Support master clock and slave clock
* Support slaveonly mode

### ICMP-TS

* Support ICMP echo and ICMP timestamp protocol
* Support hping3 or clockdiff check time diff in milliseconds

### 1PPS Output

* Support programmable pulsewidth and phase
* Pulsewidth support 10us to 900ms with 20ns step
* Phase support 0 to 2pi
* Output GPIO is PTC16

### NMEA GPRMC Output

* Support programmable phase
* Fake RMC location at null island
* Baudrate is 115200 8N1
* Output GPIO is PTC4

## Progress

| Feature | Support |
|:--------|:-------|
| HW Timestamp | YES |
| LwIP Stack | YES |
| ICMP Timestamp Reply | YES |
| PTPd | YES |
| GPS PPS Input | WIP |
| 1PPS Simulation | YES |
| NMEA RMC Simulation | YES |
| Web Control Panel | WIP |
