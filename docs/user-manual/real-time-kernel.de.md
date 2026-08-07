# Echtzeit-Kernel-Optionen

Nicht alle Anwendungsfälle erfordern ein Echtzeit-Betriebssystem. Lesen Sie weiter, um zu entscheiden, ob Sie eines benötigen.

## ⚙️ Kernel-Optionen für Echtzeitleistung

Ubuntu bietet mehrere Kernel-Varianten für unterschiedliche Workloads:

| Kernel-Typ          | Beschreibung                                                  | Typischer Anwendungsfall                       |
| ------------------- | ------------------------------------------------------------- | ---------------------------------------------- |
| `generic`           | Standard-Kernel: ausgewogene Leistung und Energieverwaltung   | Allgemeine Desktop-/Server-Nutzung             |
| `lowlatency`        | Reduzierte Interrupt-Latenz; bessere Scheduling-Reaktion      | Robotik, Audiobearbeitung, Soft-Echtzeit       |
| `rt` (`PREEMPT_RT`) | Vollständig präemptibel; harte Echtzeit-Deterministik         | Industriesteuerung, missionskritische Systeme  |

---

## ⚠️ Wichtiger Haftungsausschluss

Das Upgrade auf einen **Low-Latency-** oder **Echtzeit-Kernel (RT)** kann:
- Proprietäre Treiber beschädigen (z. B. NVIDIA, Wi-Fi-Module)
- Systeminstabilität oder Bootfehler verursachen

**Sie übernehmen die volle Verantwortung** für alle Probleme, die durch Kernel-Änderungen entstehen.
✅ **Sichern Sie immer Ihr System**, bevor Sie fortfahren.

---


## Low-Latency- oder PREEMPT_RT-Kernel für Ubuntu/x86-64 installieren

### Option 1: Low-Latency-Kernel

1. **Kernel installieren**:
   Für den Hardware-Enablement-Stack (HWE) (prüfen mit `uname -r`; z. B. Ubuntu 22.04 mit Kernel 6.x):

   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency-hwe-22.04  # "22.04" durch Ihre Version ersetzen
   ```
   Für den ursprünglichen Kernel (5.15):
   ```bash
   sudo apt update && sudo apt install --install-recommends linux-lowlatency
   ```

2. **GRUB so einstellen, dass Low-Latency bevorzugt wird**:
   ```bash
    echo 'GRUB_FLAVOUR_ORDER="lowlatency"' | sudo tee -a /etc/default/grub
    sudo update-grub
   ```

3. **Neu starten und überprüfen**:
    ```bash
    sudo reboot
    uname -r  # Sollte "...-lowlatency" anzeigen
    ```

  🔄 Um zu ``generic`` zurückzukehren, ändern Sie ``GRUB_FLAVOUR_ORDER`` auf ``"generic"`` und führen Sie ``sudo update-grub`` aus.

### Option 2: PREEMPT_RT-Kernel

Die auf dieser Seite aufgeführten Ubuntu-Releases unterstützen den Echtzeit-Kernel nativ; er kann mit wenigen Befehlen aktiviert werden. Vollständiges Tutorial unter https://ubuntu.com/real-time .


> ℹ️ Ubuntu-22.04/24.04-Benutzer können den RT-Kernel über ein **kostenloses Ubuntu-Pro-Abonnement** aktivieren. Das Abonnement ist für den persönlichen Gebrauch kostenlos.

> ℹ️ Für Nvidia Jetson (**aarch64**) lesen Sie bitte die offizielle Nvidia-Dokumentation.

---
