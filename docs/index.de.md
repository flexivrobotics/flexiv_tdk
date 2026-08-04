# Flexiv TDK Dokumentation

Willkommen auf der Dokumentationsseite des **Flexiv TDK (Teleoperation Development Kit)**. Diese Seite bietet ein Benutzerhandbuch, eine API-Referenz und FAQ, die Ihnen beim Einrichten und Integrieren von TDK helfen.

## Was ist Flexiv TDK?

Flexiv TDK ist ein SDK zum Erstellen benutzerdefinierter Roboter-zu-Roboter- oder Gerät-zu-Roboter-Teleoperationsanwendungen mit den adaptiven Robotern von Flexiv. Es ermöglicht synchronisierte, kraftgeführte Bewegungen mit **hochpräzisem Wahrnehmungs-Feedback** und unterstützt sowohl **LAN**- (Local Area Network) als auch **WAN**-Verbindungen (Internet).

TDK bietet ein breites Spektrum an Anwendungsszenarien in Bereichen wie Embodied-AI-Datenerfassung, Strahlenmedizin und Experimente mit Gefahrstoffen.

🎬 **[Flexiv's TDK | Teleoperation Made Simple](https://www.youtube.com/watch?v=H0e9FSZIa14)**
*(Zum Abspielen auf das Bild unten klicken)*
<p align="center">
  <a href="https://www.youtube.com/watch?v=H0e9FSZIa14" target="_blank">
    <img src="https://img.youtube.com/vi/H0e9FSZIa14/hqdefault.jpg" alt="TDK Demo 1" width="350" style="margin-right:10px;" />
  </a>
  <a href="https://www.youtube.com/watch?v=udkddqxth5Q" target="_blank">
    <img src="https://img.youtube.com/vi/udkddqxth5Q/hqdefault.jpg" alt="TDK Demo 2" width="350" />
  </a>
</p>

## Benchmarks für kontaktreiche Manipulation

Flexiv TDK wurde in die [Manipulation Net Peg-in-Hole-Bestenliste](https://manipulation-net.org/leaderboards/peg_in_hole.html) aufgenommen. [Manipulation Net](https://manipulation-net.org) ist ein öffentlicher Benchmark für robotische Manipulation in der realen Welt – in großem Maßstab, mit jedem Roboter, jederzeit und überall. Dies liefert einen externen Nachweis für die kontaktreiche Manipulationsfähigkeit, die für TDK-Anwendungsfälle relevant ist, einschließlich nachgiebiger Einsetzvorgänge, Ausrichtung und kraftbasierter Teleoperations-Workflows.

## Hauptmerkmale

- **Hochpräzises Wahrnehmungs-Feedback**: 100 % taktile Feedback-Transparenz gewährleistet die Präzision der menschlichen Bedienung.
- **Bessere physische Mensch-Roboter-Interaktion**: Der Leader-Roboter kann jederzeit neu positioniert oder ausgerichtet werden. Beim Wiedereinkoppeln wird nur die Relativbewegung abgebildet – keine absoluten Positionsbeschränkungen.
- **Selektive kartesische Beschränkungen**: Beschränken Sie die Bewegung auf bestimmte Richtungen für eine schnellere, präzisere Aufgabenausführung.
- **Robuster Kraft-/Momentenschutz**: Verhindert Schäden am Roboter und Werkstück und gewährleistet intrinsische Sicherheit während des Kontakts.
- **LAN/WAN-Unterstützung**: Flexible Netzwerkkonfigurationen für LAN und WAN.

## Schnellzugriff

- [Benutzerhandbuch](user-manual/overview.md)
- [API-Referenz](../api/doxygen/index.html)
- [FAQ](qa/index.md)
- [GitHub-Repository](https://github.com/flexivrobotics/flexiv_tdk)

## Dokumentationsstruktur

- **Benutzerhandbuch**: Einrichtung, Installation, Beispiele und Bedienhinweise.
- **API-Referenz**: Von Doxygen generierte C++-API-Dokumentation.
- **FAQ**: Häufige Fragen und Tipps zur Fehlerbehebung.

> Hinweis: Die API-Referenz wird mit Doxygen generiert und zusammen mit dieser Seite auf GitHub Pages veröffentlicht.
