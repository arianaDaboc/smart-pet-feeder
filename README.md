# Guardian AI Smart Pet Feeder

Guardian AI is an IoT pet-feeding system that combines local visual recognition, physical PIR motion confirmation and deterministic Arduino control.

Automatic feeding requires two independent conditions:

1. the ESP32-CAM frame must match a registered pet through the local CLIP recognition server;
2. Arduino must detect fresh PIR motion within the following ten-second authorization window.

The owner can also dispense the configured portion manually from the application or with the physical button. Arduino enforces portion timing, the adjustable cooldown and the final servo state regardless of the command source.

## Project files

- [`guardian-ai-system/`](guardian-ai-system/) — React application, Convex backend, local Next.js AI server and firmware;
- [`documentation/Guardian-AI-Project-Documentation-EN.pdf`](documentation/Guardian-AI-Project-Documentation-EN.pdf) — final seven-page English technical documentation;
- [`video/guardian_ai_video.mp4`](video/guardian_ai_video.mp4) — project demonstration video;
- the original prototype materials are archived separately in a private repository.

## Main components

- Arduino feeder controller;
- ESP8266 NodeMCU Wi-Fi/UART bridge;
- AI-Thinker ESP32-CAM;
- PIR and DHT11 sensors;
- servo motor, 16×2 I²C LCD, status LEDs and buzzer;
- React, TypeScript, Vite, Convex and Clerk;
- local Next.js server using CLIP embeddings through Transformers.js.

## Integrated product

Guardian AI is a single smart pet-feeding product. Its web interface, local AI recognition, cloud synchronization and embedded controllers work together as one system to identify the pet, confirm its presence and dispense food safely.

## Safety principle

AI errors, missing camera frames, recognition timeouts, missing PIR confirmation and an active cooldown always block automatic feeding. The ESP32-CAM cannot directly command the servo.

---

**Author:** Daboc Ariana Daria
