# Guardian AI Smart Pet Feeder

Guardian AI is an IoT pet-feeding system that combines local visual recognition, physical PIR motion confirmation and deterministic Arduino control.

Automatic feeding requires two independent conditions:

1. the ESP32-CAM frame must match a registered pet through the local CLIP recognition server;
2. Arduino must detect fresh PIR motion within the following ten-second authorization window.

The owner can also dispense the configured portion manually from the application or with the physical button. Arduino enforces portion timing, the adjustable cooldown and the final servo state regardless of the command source.

## Video demonstration

[Watch the Guardian AI Smart Pet Feeder demonstration](https://arianadaboc.github.io/smart-pet-feeder/demo.html)

## Project files

- [`guardian-ai-system/`](guardian-ai-system/) — React application, Convex backend, local Next.js AI server and firmware;
- [`documentation/Guardian-AI-Project-Documentation-EN.pdf`](documentation/Guardian-AI-Project-Documentation-EN.pdf) — final seven-page English technical documentation;
- the original prototype materials are archived separately in a private repository.

## Main components

- Arduino feeder controller;
- ESP8266 NodeMCU Wi-Fi/UART bridge;
- AI-Thinker ESP32-CAM;
- PIR and DHT11 sensors;
- servo motor, 16×2 I²C LCD, status LEDs and buzzer;
- React, TypeScript, Vite, Convex and Clerk;
- local Next.js server using CLIP embeddings through Transformers.js.

## Key features

- local visual identity matching against photos of registered pets;
- two-step automatic feeding based on AI authorization followed by fresh PIR motion;
- live ESP32-CAM monitoring and recognition status in the web application;
- support for multiple registered pets and reference-photo management;
- configurable food portions, cooldown duration and temperature limits;
- manual feeding from the application or directly from the physical button;
- real-time device status, temperature, humidity and food-level telemetry;
- feeding history, recognition events, notifications and CSV export;
- health insights based on feeding activity and consumption trends;
- guided dispenser calibration with the resulting flow rate stored in EEPROM;
- automatic Wi-Fi reconnection and a local queue for events created while offline;
- audible, visual and LCD feedback for feeding, cooldown and error states.

## Feeding modes

| Mode | Trigger | Safety behavior |
| --- | --- | --- |
| Smart automatic | Registered pet recognized by AI | Arduino waits for new PIR motion before dispensing |
| Manual application | Owner presses **Feed Now** | Arduino applies the configured portion and cooldown |
| Physical control | Owner presses the feeder button | Works locally while preserving portion and cooldown rules |

## How the product works

Guardian AI is a single smart pet-feeding product. Its web interface, local AI recognition, cloud synchronization and embedded controllers work together as one system to identify the pet, confirm its presence and dispense food safely.

The ESP32-CAM provides the live image, while the local recognition service compares the current frame with the saved pet references. A successful match creates a short authorization window. Arduino then requires a new PIR detection, opens the dispenser for the calculated portion duration, closes the servo and starts the configured cooldown. The ESP8266 bridge keeps the application synchronized with the physical feeder and queues telemetry when the network is unavailable.

## Safety principle

AI errors, missing camera frames, recognition timeouts, missing PIR confirmation and an active cooldown always block automatic feeding. The ESP32-CAM cannot directly command the servo.
