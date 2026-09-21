# DGAC Beacon (Remote ID) — ESP32-C3

ESP32-C3 firmware that transmits a DGAC beacon (802.11 WiFi beacon frame, channel 6) using the GPS data of a Betaflight flight controller read over MSP.

- Position, altitude, speed, heading and home point (captured on arming)
- Transmits every 3 s or every 30 m travelled, only after a GPS fix
- Status LED (GPIO8): solid while the beacon is transmitting

## Configuration
In `balise_dgac/balise_dgac.ino`:
- `ID_FR`: **replace the dummy identifier with your own DGAC identifier** (exactly 30 characters).
- MSP: RX = GPIO20, TX = GPIO21, 115200 baud.

## Build
Arduino IDE or `arduino-cli`, board `esp32:esp32:esp32c3`.

---

## Français

Firmware ESP32-C3 qui émet une balise DGAC (trame WiFi beacon 802.11, canal 6) à partir des données GPS d'un contrôleur de vol Betaflight lu en MSP.

- Position, altitude, vitesse, cap et point home (pris à l'armement)
- Émission toutes les 3 s ou tous les 30 m parcourus, uniquement après un fix GPS
- LED de statut (GPIO8) : fixe quand la balise émet

## Configuration
Dans `balise_dgac/balise_dgac.ino` :
- `ID_FR` : **remplace l'identifiant bidon par ton propre identifiant DGAC** (30 caractères exactement).
- MSP : RX = GPIO20, TX = GPIO21, 115200 baud.

## Compilation
Arduino IDE ou `arduino-cli`, carte `esp32:esp32:esp32c3`.
