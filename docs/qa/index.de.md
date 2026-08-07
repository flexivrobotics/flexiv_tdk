# Häufige Fragen (FAQ)

## F: Welche Plattformen werden unterstützt?
**A:** Ubuntu 22.04+ auf x86_64 und aarch64. C++ und Python werden mit GCC ≥ 9.4 und CMake ≥ 3.16.3 unterstützt.

## F: Benötige ich einen Echtzeit-Kernel?
**A:** Nicht unbedingt, aber ein Low-Latency- oder RT-Kernel verbessert Reaktionsfähigkeit und Stabilität für die Teleoperation. Siehe [Echtzeit-Kernel](../user-manual/real-time-kernel.md).

## F: Wie synchronisiere ich die Zeit für WAN-Teleoperation?
**A:** Verwenden Sie Chrony, um die Systemuhren beider Seiten zu synchronisieren. Siehe [Zeitsynchronisation (WAN)](../user-manual/time-sync.md).

## F: Wo finde ich die API-Referenz?
**A:** Die Doxygen-API-Referenz ist auf GitHub Pages unter `api/doxygen/index.html` veröffentlicht. Siehe [API-Referenz](../../api/doxygen/index.html).

## F: Wo bekomme ich Hilfe?
**A:** Wenden Sie sich an Ihren Vertriebsmanager, öffnen Sie ein Issue auf GitHub oder kontaktieren Sie Flexiv über https://www.flexiv.com/contact.
