# Duplex LiFi-Based PC-to-PC Communication

## Project Overview

This project aims to establish communication between two computers using Light Fidelity (LiFi). It uses Visible Light Communication (VLC), where data is transmitted through light signals instead of conventional wireless radio-frequency signals.

The system uses Arduino boards, LED transmitters and LDR-based receivers to send and receive data. A Python application is also being developed to support file sharing between the connected computers.

The main focus is to implement bidirectional communication and study how reliably data can be transferred through a light-based communication channel.

## Objectives

- Establish communication between two computers using LiFi.
- Implement data transmission and reception using LEDs and LDRs.
- Develop a duplex communication system.
- Interface Arduino hardware with Python applications.
- Implement file transfer between connected computers.
- Test the system under different transmission conditions.

## Hardware Components

- Arduino boards
- LEDs for data transmission
- LDRs for light detection
- Resistors
- Connecting wires
- Two computers

## Software and Tools

- Python
- Arduino IDE
- C/C++ for Arduino programming
- Serial communication
- Git and GitHub

## Working Principle

The transmitter converts digital data into signals that control the LED. The LED switches according to the transmitted data, producing light variations that represent the information.

At the receiver, the LDR detects these variations in light intensity. The received signals are processed by the Arduino and converted back into digital data.

Both computers have transmitting and receiving sections, allowing data to travel in both directions.

## Repository Structure

```text
LiFi-PC-Communication/
│
├── lifi_share_app.py
├── lifi_trx.ino
└── README.md
```

## Current Progress

- Developed the initial Arduino transmitter code.
- Started implementing the Python-based file-sharing application.
- Established serial communication between the computer and Arduino.
- Working on integrating the transmitter and receiver for duplex communication.

## Future Work

- Complete the bidirectional communication implementation.
- Improve the transmission speed and reliability.
- Test file transfer with different file sizes.
- Measure the effective data rate and bit error rate.
- Improve the receiver circuit and signal detection.

## Team

Developed by a five-member team under the GitHub organization:

[Duplex-communication-using-LiFi](https://github.com/Duplex-communication-using-LiFi)

## Project Status

Under Development
