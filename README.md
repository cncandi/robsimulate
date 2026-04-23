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
