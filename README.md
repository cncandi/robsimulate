# KUKA KR8 R1420 HW – KRL Simulator

Web-basierter 3D Kinematik-Simulator für KUKA KR8 R1420 HW.

## Struktur
```
index.html          Hauptseite
css/simulator.css   Styles
js/simulator.js     Gesamte Simulator-Logik (FK, IK, KRL-Parser, Three.js)
stl/
  a1..a6.stl        Roboter-Achsen STL-Modelle
  podest.stl        Podest
  tool1_tcp.stl     Werkzeug (Ursprung = A6-Mitte)
```

## Lokaler Start
Webserver erforderlich (fetch-API), z.B.:
```
python3 -m http.server 8080
```
dann http://localhost:8080

## TCP
X 364.5 · Y 0 · Z 46.5 · A 0 · B -90 · C 0

Entwickelt von [cnc-technik.de](https://cnc-technik.de)


## Grafikqualität (js/graphics.js)
Drei Stufen, umschaltbar über Ribbon „Grafik“ (Kamera), Menü Ansicht oder URL `?quality=standard|high|max`:
- **Standard**: bisherige Darstellung (Phong, ohne Schatten), für schwache Rechner.
- **Hoch** (voreingestellt): PBR-Material (MeshPhysicalMaterial, Lack mit Klarlack, graue Teile metallisch), Studio-Umgebung (RoomEnvironment, keine HDR-Datei), weiche Schatten, ACES-Tonemapping, sRGB-Ausgabe.
- **Maximal**: zusätzlich Ambient Occlusion (SSAO) und FXAA.
In allen Stufen werden STL-Normalen mit Knickwinkel 32° geglättet. Linien, Zielpunkte und Beschriftungen behalten ihre Farben.
`js/graphics.js` ist identisch mit RobSimul Simple; three.js-r128-Erweiterungen liegen in `js/vendor/three-r128/`.
