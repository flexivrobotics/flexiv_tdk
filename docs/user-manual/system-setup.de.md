# Roboter-Einrichtung und Netzwerkkonfiguration

!!! warning "Achtung"

    Stellen Sie vor dem Fortfahren sicher, dass der Roboter sicher auf einer stabilen Basis montiert ist und bei schnellen Bewegungen mit abrupten Stopps nicht umkippen kann.

TDK unterstützt sowohl LAN- (Local Area Network) als auch WAN-Deployments (Wide Area Network). Der folgende Abschnitt beschreibt am Beispiel einer LAN-Einrichtung die Netzwerkkonfiguration für die Roboter und den Benutzer-PC.

## Schritt 1: Roboter einschalten und Servo aktivieren

Folgen Sie der mitgelieferten Schnellstartanleitung, um die Hardware einzurichten und die beiden Roboter zu starten. Verbinden Sie die Roboter nach dem Hochfahren mit dem UI-Tablet (Flexiv Elements). Alle Leuchtringe der Roboter sollten tiefblau leuchten, wenn sie eingeschaltet und der Servo aktiviert ist.

## Schritt 2: Remote-Modus aktivieren - Ethernet

Gehen Sie auf Flexiv Elements zu Einstellungen > Remote-Modus.
![settings_remote_mode](../assets/settings_remote_mode.png)
Um den Remote-Modus zu aktivieren, schalten Sie den Schalter ein und wählen Sie dann Ethernet aus der Dropdown-Liste "Modus auswählen".
![remote_mode_ethernet](../assets/remote_mode_ethernet.png)

Prüfen Sie die derzeit angewendeten Ethernet-Kommunikationsprotokolle. Falls keine Lizenz installiert ist, können Sie die TDK-Lizenz unter Einstellungen > Lizenz installieren.
![ethernet_license](../assets/ethernet_license.png)

## Schritt 3: Netzwerk für Leader- und Follower-Roboter konfigurieren

Die Netzwerkkonfiguration für den Leader- und den Follower-Roboter erfolgt über ``Flexiv Elements->Einstellungen->Roboter-Steuerbox``.
Weitere Details finden Sie im Flexiv-Elements-Benutzerhandbuch.
![control_box_net_cfg](../assets/control_box_network.png)

Für LAN-Teleoperation setzen Sie den Leader-Roboter, den Follower-Roboter und den Benutzer-PC in dasselbe Netzsegment mit unterschiedlichen IP-Adressen (z. B. Leader-Roboter: 192.168.2.110, Follower-Roboter: 192.168.2.111 und Benutzer-PC: 192.168.2.112, jeweils mit der Subnetzmaske 255.255.255.0).

## Schritt 4: Roboter neu starten und Ping-Test

Starten Sie die Roboter nach der Netzwerk- und Remote-Modus-Konfiguration neu. Verbinden Sie den Benutzer-PC und beide Roboter mit demselben Ethernet-Switch. Pingen Sie beide Steuerboxen an, um sicherzustellen, dass alle Geräte korrekt verbunden sind.

## Schritt 5: Werkzeugkalibrierung

Stellen Sie anhand des Kapitels "Werkzeug kalibrieren" im Flexiv-Elements-Benutzerhandbuch sicher, dass die Trägheitsparameter der Werkzeuge an beiden Robotern kalibriert sind und die konfigurierten Werkzeuge tatsächlich die aktuell verwendeten sind. Setzen Sie die TCP-Position des Leader-Roboters an die Stelle, an der die menschliche Hand tatsächlich greift.
