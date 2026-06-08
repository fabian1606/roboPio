# RobotLink

ESP-NOW-Wrapper für die 6-Achs-Robotersteuerung auf ESP32 / ESP32-S3. Ein
Sender (Controller mit Potis) liest die Eingabe und schickt alle sechs
Werte in einem einzigen Frame; ein Receiver auf dem Roboter empfängt sie,
rechnet optional zylindrische Inverskinematik und schreibt die Positionen
über `SCServo` auf den Servo-Bus.

## Rollenverteilung

| Rolle | Tooling | Sketches |
|---|---|---|
| **Sender** (Controller-Box) | Arduino IDE — ZIP installieren | `SenderDirect`, `SenderCylindrical`, `SenderCartesian` (in Arbeit) |
| **Receiver** (Roboter mit Servos) | PlatformIO im roboPio-Repo | `src/receiver/main.cpp` |

Die Library selbst (`lib/RobotLink/`) ist Arduino-IDE-1.5-kompatibel und
wird vom roboPio-Projekt zugleich als lokale PIO-Library verwendet.

---

## Sender: Installation in der Arduino IDE

1. ZIP herunterladen: **[Releases](https://github.com/fabian1606/roboPio/releases/latest)** →
   `RobotLink-x.y.z.zip` herunterladen.
2. In der IDE: `Sketch → Include Library → Add .ZIP Library…` und das ZIP
   auswählen.
3. Falls die IDE „Install missing dependencies" anbietet → mit Ja bestätigen.
   Das installiert `FastLED`, `Smoothed` und `SCServo` aus dem Library Manager.
4. ESP32-Board-Support: `Boards Manager → "esp32" by Espressif Systems`
   installieren (falls nicht vorhanden).
5. `File → Examples → RobotLink → SenderDirect` öffnen (oder
   `SenderCylindrical`), Board ESP32-S3-Dev-Modul wählen, hochladen.
6. Sender und Receiver müssen im **selben Gruppenkanal (1–5)** sein, sonst
   werden Frames verworfen. Kanal-Wechsel via **Doppel-Reset** (zwei RST-Drücke
   innerhalb 400 ms) oder Mode-Button auf GPIO0. Der Koordinatenmodus
   (`COORD_DIRECT` / `COORD_CYLINDRICAL` / `COORD_CARTESIAN`) ist davon
   unabhängig und wird per `setCoordMode()` gesetzt.

## Receiver: Build und Flashen via PlatformIO

```
git clone https://github.com/fabian1606/roboPio
cd roboPio
pio run -e receiver -t upload
```

Das ist der einzige PIO-Env im Projekt. Dependencies kommen aus
`platformio.ini`, der Sketch liegt in [`src/receiver/main.cpp`](../../src/receiver/main.cpp).

---

## Sender-Beispiele

| Beispiel | CoordMode | Beschreibung |
|---|---|---|
| `SenderDirect` | `COORD_DIRECT` | Jeder Poti → ein Servo, 1:1 durchgereicht. |
| `SenderCylindrical` | `COORD_CYLINDRICAL` | Potis liefern Basisrotation / Elevation / Radius / Greifer; IK rechnet der Receiver. |
| `SenderCartesian` | `COORD_CARTESIAN` *(in Arbeit)* | XYZ + Tool-Pitch (noch nicht produktiv — `COORD_CARTESIAN` fehlt zurzeit in der Lib). |

---

## API-Kurzreferenz

```cpp
#include <RobotLink.h>

// — Sender —
const int basePot = 4;   // GPIO-Pin des Potis für Achse 0

robotLink.setCoordMode(COORD_CYLINDRICAL);  // optional, Default: COORD_DIRECT
robotLink.setAxisSmoothing(0, 10);          // pro Achse, 1 (kaum) … 50 (stark)
robotLink.beginSender();                    // ESP-NOW + LED + Doppel-Reset

// im loop():
robotLink.setAxisValue(0, analogRead(basePot));
robotLink.sendAllAxes();

// — Receiver —
robotLink.setKinematics(KINEMATICS);         // optional, für COORD_CYLINDRICAL
robotLink.beginReceiver(onFrame);            // onFrame(receiverID, values[6])
robotLink.update();                          // im loop()
```

Vollständige API: siehe [`src/RobotLink.h`](src/RobotLink.h).

---

## Plattformen

- ESP32-S3 (z. B. ESP32-S3-DevKitC-1) — Sender.
- ESP32 (z. B. Waveshare „Servo Driver with ESP32") — Receiver.

Andere Architekturen werden nicht unterstützt (`esp_now.h` / `Preferences.h`
sind ESP32-spezifisch).

---

## ZIP für Verteilung bauen und als Release veröffentlichen

```bash
cd lib
find RobotLink -name ".DS_Store" -delete
zip -r ../RobotLink-1.0.0.zip RobotLink
```

Wichtig: oberster Eintrag im ZIP muss der Ordner `RobotLink/` sein,
sonst lehnt die Arduino IDE den Import ab.

Anschließend ein GitHub Release anlegen:

```bash
gh release create lib/v1.0.0 ../RobotLink-1.0.0.zip \
  --title "RobotLink 1.0.0" \
  --notes "Erste stabile Version der RobotLink-Library."
```

Der direkte Download-Link lautet dann:
`https://github.com/<user>/roboPio/releases/download/lib/v1.0.0/RobotLink-1.0.0.zip`
