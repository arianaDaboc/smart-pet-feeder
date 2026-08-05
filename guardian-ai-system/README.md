# Guardian AI

**Intelligent recognition. Safer feeding. Complete control.**

Guardian AI is an IoT smart feeder that combines local CLIP-based pet recognition, physical PIR motion confirmation, configurable portion control and cooldown protection.

## System overview

- ESP32-CAM captures the live image.
- A local Next.js server generates and compares CLIP embeddings.
- The web application authorizes only registered pets.
- ESP8266 bridges the application, Convex and Arduino over Wi-Fi/UART.
- Arduino controls the PIR sensor, servo, LCD, indicators and safety state machine.
- Automatic feeding requires both AI authorization and PIR confirmation within approximately 10 seconds.

## Documentation

The complete Romanian technical documentation is available in [DOCUMENTATION.md](DOCUMENTATION.md). It covers:

- architecture and hardware wiring;
- automatic, application and physical-button feeding modes;
- local AI enrollment and recognition;
- HTTP APIs and the serial protocol;
- installation and configuration;
- security and fail-safe behavior;
- acceptance testing and troubleshooting;
- limitations and future development.

## Quick start

```powershell
npm install
npm run dev
```

Start the local AI service separately:

```powershell
cd local-ai-server
npm install
npm run dev
```

The local AI server listens on port `3000`. Keep it running on a PC, mini-PC or Raspberry Pi connected to the same local network.

## Important safety note

Automatic feeding must remain fail-safe: an AI error, camera failure, missing PIR confirmation or active cooldown must never result in automatic dispensing. Nutritional estimates are informational and must be reviewed by a veterinarian before changing an animal's diet.
