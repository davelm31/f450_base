# F450 Description Setup

Este repositorio contiene los archivos necesarios para la simulación del modelo F450. Sigue estos pasos para generar correctamente el modelo y actualizar el mundo en Gazebo.

---

## 🔥 Paso 1: Eliminar archivos antiguos

Antes de generar nuevos archivos, elimina los antiguos para evitar conflictos:

```bash
rm urdf/f450b.urdf
rm models/f450b/model.sdf
```

---

## 🔧 Paso 2: Compilar el workspace

Desde la raíz de tu workspace (`ros2_ws`), ejecuta:

```bash
colcon build
source install/setup.bash
```

---

## 🧱 Paso 3: Generar URDF y SDF

Navega al directorio del paquete `f450_description`:

```bash
cd ~/ros2_ws/src/f450_description
```

Ejecuta los siguientes comandos para generar el modelo:

```bash
xacro urdf/f450.xacro > urdf/f450.urdf
gz sdf -p urdf/f450.urdf > models/f450/model.sdf
```

> Asegúrate de tener instalado `xacro` y `gz` (Gazebo Tools).

---

## 🌍 Paso 4: Cambiar ruta del modelo en el mundo

Edita el archivo `worlds/empty_world.world` y reemplaza la antigua referencia del modelo F450 por la nueva. Asegúrate de que se vea así:

```xml
<include>
  <uri>model://f450</uri>
</include>
```

---

## ✅ Verificación

- Asegúrate de que el modelo aparezca correctamente en Gazebo.
- Verifica que `models/f450/model.sdf` sea el archivo más reciente.
- Si es necesario, exporta la variable de entorno `GAZEBO_MODEL_PATH`:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/f450_description/models
```

---

## 🗂️ Estructura esperada

```
f450_description/
├── urdf/
│   ├── f450.xacro
│   └── f450.urdf  ← generado
├── models/
│   └── f450/
│       └── model.sdf  ← generado
├── worlds/
│   └── empty_world.world  ← actualizado
```

---

## 📌 Recomendación

Usa control de versiones (Git) para evitar sobrescribir archivos accidentalmente. Agrega el siguiente contenido a tu `.gitignore` si no deseas versionar los archivos generados:

```
urdf/f450.urdf
models/f450/model.sdf
```
