# TARÁNTULA — Interfaz de control

El programa de control (`TARANTULA_UI`) puede manejarse desde dos interfaces que usan exactamente el mismo
código de control (`Tarantula`, `Leg`, `GaitController`, `WaveshareInterface`…):

- **Interfaz web** (por defecto): se abre en el navegador, con el CAD real de la araña en 3D moviéndose en
  tiempo real.
- **Interfaz Qt clásica** (`MainWindow`): la ventana de escritorio de siempre.

## Compilar

Igual que siempre con Qt Creator (kit *Desktop Qt 6 MinGW 64-bit*) o con CMake. La interfaz web sólo
necesita el módulo **Qt Network**, que viene con cualquier instalación de Qt 6; no hace falta instalar nada más.

> `include/Eigen/` no está en el repo (ver `.gitignore`): hay que tenerla en local para compilar.

## Arrancar

```bash
TARANTULA_UI.exe                 # interfaz web en http://localhost:8080 (abre el navegador)
TARANTULA_UI.exe --qt            # interfaz Qt clásica (MainWindow)
TARANTULA_UI.exe --port 8090     # interfaz web en otro puerto
TARANTULA_UI.exe --no-browser    # no abrir el navegador automáticamente
TARANTULA_UI.exe --sim           # modo simulación (ver abajo)
```

En Qt Creator los argumentos se ponen en *Proyectos → Ejecutar → Argumentos de la línea de comandos*.

Notas:

- El ejecutable busca la carpeta `web/` junto a él o en las carpetas superiores (así funciona desde
  `build/...` sin copiar nada). Si se lanza desde otro sitio, el directorio de trabajo debe contener `web/`.
- Si el puerto está ocupado, prueba automáticamente los 9 siguientes (8081, 8082…); la URL real sale en la consola.
- El servidor sólo escucha en `127.0.0.1`: no es accesible desde otros equipos de la red.

### La interfaz web

- **Barra superior**: puerto COM (lista los puertos que existen), estado **USB** (adaptador Waveshare abierto),
  **CAN** (algún motor responde), **Backend** (la página está conectada al `.exe`) y encender/apagar todos.
- **Izquierda**: levantarse/acostarse, tipo de marcha y D-pad (también con las flechas del teclado), postura
  del cuerpo (X, Y, Z, roll, pitch, yaw) y reset de pose.
- **Centro**: el robot en 3D. *Comandado* muestra la consigna que se envía a cada motor; *Real* muestra la
  posición que devuelven los encoders. Clic en una pata para seleccionarla en el IK.
- **Derecha**: control IK de pata (desplegable), comando de articulaciones a las 4 patas, motor individual
  (encender / apagar / enviar), estado de los 12 motores y reset del log CAN (`can_log.csv`).

Seguridad:

- Sólo manda **la pestaña abierta más recientemente**; las demás quedan en *solo lectura* (se puede recuperar
  el control pulsando en la pastilla de estado).
- Si la pestaña que manda se cierra o pierde la conexión, se suelta el D-pad y **se detiene la marcha**.
  Lo mismo si la ventana del navegador pierde el foco mientras se mantiene una flecha pulsada.
- La marcha sólo arranca desde la pose de reposo erguida: si no lo está, se hace un reset de pose
  (levantarse) y la marcha empieza sola al llegar, igual que en la interfaz Qt.

### Modo simulación (`--sim`)

Sirve para probar la interfaz y ver moverse el gemelo digital **sin la araña conectada**. Los motores que no
responden se tratan como encendidos y el programa calcula su trayectoria trapezoidal (sin enviar nada), así
que levantarse, la postura, la marcha, el IK, etc. mueven el modelo *Comandado*. Los motores que sí estén
conectados funcionan con normalidad. En el visor aparece la etiqueta **SIMULACIÓN**.

**No usar `--sim` con el robot conectado.**

## Estructura

```
src/WebServer.cpp, include/WebServer.h   servidor HTTP + WebSocket (Qt Network) y órdenes al robot
web/index.html, web/css/, web/js/        interfaz (app.js: controles y conexión; spider3d.js: visor 3D)
web/models/tarantula.glb                 modelo 3D generado a partir del CAD
web/vendor/three/                        three.js r170 (copia local, funciona sin internet)
tools/step2glb/                          script para regenerar el modelo 3D desde el STEP
```

El protocolo es JSON por WebSocket (`ws://localhost:8080/ws`): la página envía órdenes
(`{"cmd": "standUp"}`, `{"cmd": "bodyPose", "x": 0, ...}`…) y el programa difunde el estado del robot a 30 Hz.
La lista completa de órdenes está en `WebServer::handleCommand`.

## Regenerar el modelo 3D si cambia el CAD

El visor usa `web/models/tarantula.glb`, generado a partir del ensamblaje exportado como **STEP** desde Fusion 360.

1. En Fusion 360: *Archivo → Exportar → STEP* del ensamblaje completo (cuerpo + las 4 patas, en la pose
   de referencia: patas estiradas en horizontal y tibia vertical hacia abajo).
2. Con [Node.js](https://nodejs.org) instalado:

   ```bash
   cd tools/step2glb
   npm install
   node convert.mjs "ruta/al/ENSAMBLAJE TARANTULA.step" ../../web/models/tarantula.glb
   ```

3. Recargar la página.

El script imprime las medidas que lee del CAD (`R_HIP`, `L1`, `L2`); deben coincidir con las de `Leg.h`.

Qué hace `convert.mjs`:

- Lee el STEP con [occt-import-js](https://github.com/kovacsv/occt-import-js) (OpenCascade).
- Toma la **primera pata** del ensamblaje y reparte sus 29 piezas entre **coxa**, **fémur** y **tibia** con la
  tabla `LINK_OF_PART`. Si se añaden, quitan o reordenan piezas de la pata en el CAD, **hay que actualizar
  esa tabla**: si una pieza queda en el eslabón equivocado, se verá "despegada" al mover esa articulación.
- Coloca cada eslabón en el sistema de su articulación usando el centro de los motores como pivote, de modo
  que el visor puede montar la cadena con `R_HIP`, `L1`, `L2` y `L3` igual que `Leg::forwardKinematics`.
- Simplifica las piezas más pesadas (los motores) para que el visor vaya fluido.
