# roboPio

6-Achsen-Roboterarm mit kabelloser Steuerung via ESP-NOW.

**Dokumentation: [robo-pio.vercel.app](https://robo-pio.vercel.app/)**

---

## Überblick

Ein ESP32-S3 (Sender/Controller-Box) liest sechs Potentiometer und schickt die Werte drahtlos an einen ESP32 (Receiver/Roboter). Der Receiver rechnet optional Inverskinematik und schreibt die Gelenkpositionen auf den Servo-Bus.

## Repo-Struktur

```
roboPio/
├── lib/RobotLink/          # Arduino-Bibliothek (Sender + Receiver)
│   ├── src/                # RobotLink.h + RobotLink.cpp
│   └── examples/           # SenderDirect, SenderCylindrical, SenderCartesian
├── src/receiver/main.cpp   # Receiver-Firmware (PlatformIO)
├── docs/                   # Dokumentationswebsite (Astro/Starlight)
└── platformio.ini
```

## Schnellstart

**Sender (Arduino IDE)**

1. `lib/RobotLink/` als ZIP-Bibliothek in der Arduino IDE installieren (`Sketch → Include Library → Add .ZIP Library…`).
2. Abhängigkeiten im Library Manager installieren: `FastLED`, `Smoothed`.
3. Beispiel öffnen: `File → Examples → RobotLink → SenderDirect`.
4. Board: ESP32-S3 Dev Module — hochladen.

**Receiver (PlatformIO)**

```bash
git clone https://github.com/YOUR_USER/roboPio
cd roboPio
pio run -e receiver -t upload
```

## Ansteuerungsmodi

| Modus | CoordMode | Beschreibung |
|---|---|---|
| 1 | `COORD_DIRECT` | Jeder Poti steuert direkt einen Servo |
| 2 | `COORD_CYLINDRICAL` | Zylindrische Koordinaten (Rotation, Elevation, Reichweite) |
| 3 | `COORD_CARTESIAN` | Kartesische Koordinaten (X, Y, Z + Werkzeugneigung) |

Moduswechsel: **zweimal RST** innerhalb von 400 ms. Die LED zeigt den aktiven Modus (Weiß / Rot / Grün / Blau / Magenta).

Alle Details, Schaltpläne und Beispielcode: **[robo-pio.vercel.app](https://robo-pio.vercel.app/)**
