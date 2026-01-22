# alphaloc

AlphaLoc is an ESP32 application which allows embedding GPS location in Sony Alpha camera photos.

- It parses NMEA sentences it receives via UART from a GPS module, e.g. GY-GPS6MV2.
- It connects to a Sony Alpha camera via BLE and sends the aquired location.
- It allows configuration via BLE server or WiFi (both acting as an access point or as a client):

  <img width="635" height="922" alt="image" src="https://github.com/user-attachments/assets/5a3e1a87-8896-4ce5-b7ce-edba67558b7e" />
