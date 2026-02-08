# Opticomm – Optische Datenübertragung einfach erklärt

## Einführung

Opticomm ist ein Experiment zur optischen Datenübertragung. Das Projekt zeigt, wie digitale Informationen nicht über Kabel oder Funk, sondern über **sichtbares Licht** übertragen werden können. Konkret wird ein kleiner Laser verwendet, der schnell ein- und ausgeschaltet wird, um Daten zu senden. Ein Lichtsensor misst diese Helligkeitsänderungen und eine Software rekonstruiert daraus die ursprünglichen Daten.

Das Projekt wurde als Desktop-Versuch aufgebaut und richtet sich nicht an industrielle Anwendungen, sondern an das **Verständnis grundlegender Prinzipien moderner Kommunikationstechnik**: Signalübertragung, Synchronisation, Rauschunterdrückung und digitale Codierung.

---

## Grundidee: Daten als Licht

Digitale Daten bestehen aus Bits – also Nullen und Einsen. In Opticomm werden diese Bits folgendermaßen dargestellt:

* **Laser an** → Bit = 1
* **Laser aus** → Bit = 0

Der Laser wird in festen Zeitabständen ein- oder ausgeschaltet. Der Empfänger misst kontinuierlich die Helligkeit. Steigt die gemessene Helligkeit plötzlich stark an, bedeutet das: Der Laser wurde eingeschaltet. Fällt sie stark ab, wurde er ausgeschaltet.

Das Prinzip ist einfach, die Umsetzung jedoch nicht trivial, da der Sensor nicht nur den Laser, sondern auch Umgebungslicht (z. B. eine Schreibtischlampe oder Sonnenlicht) misst.

---

## Versuchsaufbau

Der gesamte Aufbau befindet sich auf einem Schreibtisch:

* Zwei Arduino Uno
* Ein 5 mW Laser-Modul
* Ein analoger Grove-Lichtsensor
* Abstand zwischen Laser und Sensor: ca. **15 cm**
* Umgebungslicht: Tischlampe, gelegentlich Sonnenlicht durch ein Fenster

Ein Arduino steuert den Laser (Sender), der andere liest den Lichtsensor aus (Empfänger). Beide Arduinos sind per USB mit einem Computer verbunden.

Wichtig: Die Arduinos führen **keine komplexe Logik** aus. Sie dienen nur als Schnittstelle zwischen Hardware und Computer. Die eigentliche Verarbeitung geschieht vollständig auf dem PC.

---

## Rollen der einzelnen Komponenten

### Sender (Laser-Arduino + PC)

Der Sender erhält vom Computer einfache Befehle:

* `1` → Laser einschalten
* `0` → Laser ausschalten

Die PC-Software erzeugt aus Text oder Dateien eine Folge von Bits und sendet diese Bits in festen Zeitabständen an den Arduino. Der Arduino setzt diese Bits direkt in den Laserzustand um.

### Empfänger (Sensor-Arduino + PC)

Der Empfänger misst die Umgebungshelligkeit sehr häufig (etwa **1230 Messungen pro Sekunde**). Jede Messung besteht aus:

* einem Zeitstempel (in Mikrosekunden)
* einem Helligkeitswert (0–1023)

Diese Rohdaten werden an den PC geschickt. Dort entscheidet die Software, ob die gemessene Änderung durch den Laser verursacht wurde oder nur Rauschen ist.

---

## Das Übertragungsformat

Damit der Empfänger weiß, **wann eine echte Nachricht beginnt**, wird jedes Datenpaket gleich aufgebaut:

1. **Preamble** – eine feste Bitfolge (z. B. 1010101011100010)
2. **Längenfeld** – 32 Bit, die angeben, wie viele Datenbits folgen
3. **Payload** – die eigentlichen Nutzdaten

Die Preamble ist besonders wichtig: Sie verhindert, dass zufällige Helligkeitsänderungen (z. B. jemand schaltet das Licht ein) als Nachricht interpretiert werden.

---

## Zentrale Herausforderung: Rauschen

Der Lichtsensor misst nicht nur den Laser, sondern alles Licht im Raum. Dazu kommt:

* Sensorrauschen
* langsame Änderungen des Umgebungslichts
* Verzögerungen im Sensor selbst

Ein einfacher Schwellwert („wenn heller als X, dann Laser an“) würde daher nicht zuverlässig funktionieren.

Opticomm löst dieses Problem mit **adaptiven gleitenden Mittelwerten**.

---

## Wie der Empfänger das Signal erkennt

### Schritt 1: Trend bestimmen

Zuerst wird ein gleitender Mittelwert der Helligkeit berechnet. Dieser beschreibt das **langsame Grundniveau** des Umgebungslichts.

### Schritt 2: Schnelle Änderungen erkennen

Die Differenz zwischen aktuellem Messwert und diesem Mittelwert zeigt, ob es gerade eine schnelle Änderung gibt – genau das passiert, wenn der Laser ein- oder ausgeschaltet wird.

### Schritt 3: Rauschpegel abschätzen

Ein zweiter gleitender Mittelwert schätzt, wie groß typische zufällige Änderungen sind. Dieser Wert dient als **adaptiver Rauschpegel**.

### Schritt 4: Entscheidung

Nur wenn die gemessene Änderung deutlich größer ist als das übliche Rauschen, wird sie als echtes Laser-Ereignis gewertet.

So passt sich das System automatisch an unterschiedliche Lichtbedingungen an.

---

## Zeitliche Synchronisation

Sobald eine Umschaltung des Lasers erkannt wird, weiß der Empfänger:

> Jetzt beginnt ein neues Bit.

Der Empfänger wartet dann **eine halbe Bitdauer** und liest den Laserzustand genau in der Mitte jedes Bits aus. Dadurch ist das System tolerant gegenüber kleinen Zeitabweichungen und Sensorverzögerungen.

Die Bitdauer beträgt in Opticomm **50 ms**, was einer Datenrate von etwa **20 Bit pro Sekunde** entspricht.

---

## Auswertung und Speicherung

Sobald alle Bits des Payloads empfangen wurden:

* werden sie wieder zu Bytes zusammengesetzt
* als Binärdatei gespeichert

Zusätzlich werden alle Messdaten in einer Logdatei gespeichert. Diese kann später grafisch ausgewertet werden.

Typische Diagramme zeigen:

* den Roh-Helligkeitsverlauf
* erkannte Laser-Umschaltungen
* das Verhältnis von Signal zu Rauschen

Diese Visualisierungen machen die Funktionsweise besonders anschaulich.

---

## Ergebnisse

* Stabile Übertragung auf kurze Distanz (15 cm)
* Erfolgreiche Übertragung kurzer Textnachrichten
* Funktioniert trotz wechselndem Umgebungslicht

Es wurden keine formalen Fehlerraten gemessen, aber in der Praxis ist die Übertragung meist erfolgreich.

---

## Grenzen des Systems

* Geringe Datenrate (bedingt durch Sensor und Signalform)
* Keine Fehlerkorrektur oder Prüfsummen
* Empfindlich gegenüber sehr starkem oder schnell wechselndem Umgebungslicht

Diese Einschränkungen sind bewusst in Kauf genommen worden, um das System verständlich und überschaubar zu halten.

---

## Fazit

Opticomm zeigt, dass optische Kommunikation mit sehr einfachen Mitteln möglich ist. Das Projekt verbindet Hardware, Software und Signalverarbeitung zu einem funktionierenden Gesamtsystem.

Besonders wichtig ist dabei nicht der Laser selbst, sondern die **intelligente Auswertung der Messdaten**. Durch adaptive Filter und klare Protokollstrukturen wird aus einem verrauschten Helligkeitssignal eine digitale Nachricht.

Damit demonstriert Opticomm grundlegende Konzepte moderner Kommunikationstechnik in einer anschaulichen, experimentellen Form.
