# Überblick über das Benutzerhandbuch

!!! warning "Wichtig"

    Bevor Sie den Roboter mit Flexiv TDK verwenden, müssen Sie alle mit dem Roboter gelieferten Dokumente sowie dieses Handbuch sorgfältig lesen und alle darin beschriebenen Sicherheitshinweise strikt befolgen.

Dieses Handbuch führt Sie durch die Installation, Konfiguration und Nutzung des Flexiv TDK (Teleoperation Development Kit) zum Aufbau von Teleoperationsanwendungen über LAN oder WAN.

## Zielgruppe

- Anwendungsentwickler, die Teleoperationsfunktionen integrieren
- Robotikingenieure, die Flexiv-Systeme einsetzen
- Forscher, die eigene Roboter-zu-Roboter-Teleoperations-Workflows aufbauen

## Voraussetzungen

### Roboter
- Mindestens zwei Roboter der Enlight-Serie für TDK v2.x (Roboter der Rizon-Serie mit konfiguriertem FT-Sensor für TDK v1.x)

### Netzwerkgeräte
- Netzwerkgeräte (z. B. Ethernet-Switch/Router) mit ausreichender Bandbreite und Latenz
- CAT-6- oder CAT-7-Ethernet-Kabel

### Benutzer-PC
- Ubuntu 22.04+ (x86_64 oder aarch64): C++ und Python 3.10/3.12/3.14
- macOS 14+ (arm64): C++ und Python 3.10/3.12
- C++-Toolchain: GCC ≥ 9.4 (Linux) oder Apple Clang ≥ 15 (macOS), CMake ≥ 3.16.3
- Netzwerkzugang zum Roboter und zum Internet (für WAN)

## Einstieg

- Schließen Sie zuerst die [Systemeinrichtung](./system-setup.md) ab
- Neu im SDK? Beginnen Sie mit dem [Schnellstart (Python)](quick-start-python.md) oder [Schnellstart (C++)](quick-start-cpp.md).
- Benötigen Sie harte Echtzeitleistung? Siehe [Echtzeit-Kernel](real-time-kernel.md).
- Einsatz über WAN? Siehe [Zeitsynchronisation (WAN)](time-sync.md).
- Suchen Sie die API-Dokumentation? Siehe [API-Dokumentation](../api/index.md).
