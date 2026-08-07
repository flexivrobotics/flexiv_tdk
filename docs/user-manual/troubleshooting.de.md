# Fehlerbehebung

## Build-Probleme

- **CMake kann Flexiv TDK nicht finden**: Stellen Sie sicher, dass `-DCMAKE_PREFIX_PATH=~/tdk_install` auf das Installationspräfix zeigt.
- **Fehlende Abhängigkeiten**: Führen Sie `thirdparty/build_and_install_dependencies.sh` erneut aus und stellen Sie sicher, dass das Installationsverzeichnis existiert.

## Laufzeitprobleme

- **Shared Library nicht gefunden**: Setzen Sie `LD_LIBRARY_PATH=~/tdk_install/lib`, bevor Sie Binärdateien ausführen.
- **Berechtigungsprobleme mit Echtzeitprioritäten**: Fügen Sie Ihren Benutzer in `/etc/security/limits.conf` für `rtprio` und `nice` hinzu.

## WAN-Teleoperation

- **Hohe Latenz / instabile Bewegung**: Überprüfen Sie die Netzwerkstabilität und prüfen Sie die Zeitsynchronisation erneut.
- **Zeitsynchronisation ungenau**: Verwenden Sie `chronyc tracking` und erwägen Sie, `chronyc burst` und `chronyc makestep` erneut auszuführen.

## Leader-Roboter driftet

- Überprüfen Sie die Kalibrierung der Trägheitsparameter des Leader-Roboter-Werkzeugs und bestätigen Sie, dass das konfigurierte Werkzeug tatsächlich das aktuell verwendete ist.
- Überprüfen Sie, ob die TCP-Position des Leader-Roboters an der Stelle gesetzt ist, an der die menschliche Hand tatsächlich greift.
- Beim Starten der TDK-Beispiele wird anhand der tatsächlichen Eingabeparameter entschieden, ob der Sensor kalibriert wird. Wenn die Sensorkalibrierung bei der Initialisierung gewählt wird, dauert die Kalibrierung einige Zeit – berühren Sie das Gerät während dieses Vorgangs nicht.

## Hilfe erhalten

- [GitHub Issues](https://github.com/flexivrobotics/flexiv_tdk/issues)
