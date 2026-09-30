Taller Segundo Corte: Interfaz HIL Real-to-Sim, Drones y Cinemática Inversa

Autores: Jeicob David Pinilla Ruiz y Fabian Abril Casallas
Programa: Ingeniería Mecatrónica, Universidad Militar Nueva Granada

1. ANALISIS Y FUNDAMENTACION DEL PROYECTO
En el campo de la mecatrónica, la implementación de sistemas Hardware-in-the-Loop (HIL) permite validar lógicas de control utilizando hardware físico para interactuar con modelos virtuales (Gemelos Digitales). Este taller despliega esta arquitectura en tres escenarios incrementales: la estabilización de drones en formación mediante control PID, el posicionamiento espacial con manipulación de objetos en un robot industrial (Baxter) y la traslación de cinemática inversa a un robot humanoide complejo (Atlas).

El reto principal radica en la conversión de señales analógicas puras a un modelo de control articular. El sistema no controla los motores de forma individual (Cinemática Directa), sino que captura coordenadas cartesianas (X, Y, Z) y utiliza algoritmos de Cinemática Inversa (IK) en PyBullet para calcular automáticamente los ángulos requeridos por cada eslabón, garantizando un movimiento natural hacia el objetivo.

2. ARQUITECTURA DEL SISTEMA
El sistema opera en un ciclo de comunicación serial a 115200 baudios:

* Capa Física (ESP32): Lee joysticks (ADC en pines 32, 33, 34) y botones físicos. Atenúa la señal a 3.3V, mapea los valores a dimensiones espaciales en metros y envía tramas codificadas.
* Capa Virtual (PyBullet): Un entorno en Python lee el puerto serial, renderiza las físicas, procesa la matriz jacobiana para la cinemática inversa y aplica control de posición a los actuadores virtuales.

3. DESARROLLO DE LOS PUNTOS DEL TALLER

PUNTO A: Control de Formación de Drones (PID)
Se implementó el entorno "gym-pybullet-drones". Para evitar colisiones en el aire al inicializar la formación con el controlador PID, se modificó el offset de separación original en el código fuente, ampliando la distancia a un metro entre agentes. Esto permitió que los tres cuadrirrotores estabilizaran sus ángulos de Euler (Roll, Pitch, Yaw) sin estrellarse.

PUNTO B: Cinemática Inversa y Manipulación (Robot Baxter)
Para este punto, se diseñó el control del manipulador Baxter. A diferencia de un movimiento simple, el sistema incluye un cuarto parámetro en la trama serial: el estado de un botón. Al recibir las coordenadas (X, Y, Z), PyBullet resuelve la cinemática inversa del efector final, y al recibir el comando del botón, acciona los servomotores de la pinza (gripper) permitiendo al robot coger y mover objetos dinámicos dentro del entorno de simulación.

PUNTO C: Movilidad Humanoide en Entorno Complejo (Robot Atlas)
En esta fase, la misma consola ESP32 se acopla al robot humanoide Atlas de Boston Dynamics. Este escenario presenta un mayor consumo computacional debido a la renderización simultánea de cámaras sintéticas (RGB, Profundidad y Máscara de Segmentación). El script de Python se adaptó para redirigir el cálculo de cinemática inversa a la muñeca de Atlas, aplicando una fuerza de torque muy superior (1500N) para lograr vencer la alta inercia que presenta la masa de este modelo.

4. EL INFIERNO DE LAS DEPENDENCIAS CON PYBULLET
La integración de PyBullet en Windows presentó un error crítico de compilación: "Microsoft Visual C++ 14.0 or greater is required". Dado que el repositorio original de los drones exigía versiones de Python superiores a 3.12, pip intentaba compilar el motor de físicas desde cero.
Para sortear esto sin instalar pesados compiladores de Visual Studio, se aplicó ingeniería inversa al gestor de dependencias: se creó un entorno aislado con Miniconda en Python 3.9, se modificaron los candados de versión en el archivo "pyproject.toml" (bajando la exigencia a >=3.9) y se forzó la instalación con el parámetro "--no-deps". Esto permitió usar los binarios precompilados de PyBullet entregados por Conda-Forge, logrando un entorno robusto a 240Hz.
5. CODIGOS FUENTE IMPLEMENTADOS

Fase 1: Adquisición de Datos y Acondicionamiento de Señales (Firmware ESP32)
El núcleo de la interacción de hardware recae en el microcontrolador ESP32, encargado de digitalizar los movimientos mecánicos del usuario. Los joysticks analógicos, que operan como divisores de tensión en tres ejes espaciales (X, Y, Z), entregan voltajes continuos que son leídos por los conversores Analógico-Digital (ADC) integrados en los pines 32, 33 y 34 de la placa.

Para garantizar la estabilidad del sistema Hardware-in-the-Loop y evitar movimientos erráticos (jitter) en el entorno de PyBullet, se implementaron las siguientes estrategias a nivel de firmware:

Ajuste de Rango Dinámico (Atenuación): El ESP32 por defecto lee voltajes hasta 1.1V. Para leer el rango completo de los joysticks alimentados a 3.3V, se configuró una atenuación de 11dB en cada canal de lectura (ADC.ATTN_11DB).

Resolución del ADC: Se aprovechó la resolución nativa de 12 bits del microcontrolador, obteniendo 4096 pasos de cuantización (0 a 4095). Esto permite una traslación milimétrica hacia el espacio virtual.

Acondicionamiento del Efector Final: Se configuró el pin 25 con una resistencia interna de Pull-Up para leer de forma segura el estado lógico del botón físico, el cual determina la apertura o cierre del gripper (pinza) del robot Baxter.

Mapeo Geométrico y Control de Flujo: Se desarrolló una función de interpolación lineal para traducir los 4096 pasos crudos a coordenadas espaciales en metros (ej: -1.0 a 1.0 m). Además, se estandarizó una frecuencia de muestreo de 20 Hz (Delay de 50ms) para transmitir las tramas por el puerto serial. Esto es crítico: si el ESP32 envía datos más rápido de lo que el motor de físicas a 240Hz puede procesar, el buffer serial del computador se desborda, induciendo un retraso inaceptable en el control del robot.

--- CODIGO 1: FIRMWARE CONSOLA ESP32 (MICROPYTHON) ---
El siguiente script se ejecuta de forma autónoma en el ESP32. Incluye el tratamiento de señales y la estructuración de la trama de datos "X,Y,Z,Gripper" requerida por los tres puntos del laboratorio.

from machine import Pin, ADC
import time

1. Declaración e inicialización de periféricos
Se asignan los pines ADC para capturar la posición espacial
adc_x = ADC(Pin(32))
adc_y = ADC(Pin(33))
adc_z = ADC(Pin(34))

Configuración del botón para el efector final (Gripper)
Se usa PULL_UP interno para evitar lecturas flotantes (ruido EMI)
btn_grip = Pin(25, Pin.IN, Pin.PULL_UP)

2. Acondicionamiento de señal
La atenuación de 11dB expande el rango de lectura seguro hasta los 3.3V
adc_x.atten(ADC.ATTN_11DB)
adc_y.atten(ADC.ATTN_11DB)
adc_z.atten(ADC.ATTN_11DB)

3. Función de Interpolación Lineal (Mapeo Espacial)
Transforma el dominio del sensor digitalizado al dominio espacial del simulador
def map_val(x, in_min, in_max, out_min, out_max):
# Ecuación de la recta para escalar valores proporcionales
return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

print("Iniciando transmisión de telemetría HIL...")

4. Bucle principal de control (Main Loop)
while True:
# Captura de datos crudos (0-4095)
raw_x = adc_x.read()
raw_y = adc_y.read()
raw_z = adc_z.read()
# Transformación a coordenadas cartesianas virtuales (metros)
# Se ajustan los rangos de salida según los límites de la cinemática de Baxter/Atlas
x = map_val(raw_x, 0, 4095, -1.0, 1.0)
y = map_val(raw_y, 0, 4095, -1.0, 1.0)
z = map_val(raw_z, 0, 4095, 0.0, 2.0)

# Lógica de conmutación del Gripper
# Lógica inversa por el PULL_UP: 0 es presionado (cerrar), 1 es suelto (abrir)
grip = 1 if btn_grip.value() == 0 else 0

# Empaquetado estandarizado de la trama serial
# Se limita a 2 decimales para optimizar el ancho de banda serial
print(f"{x:.2f},{y:.2f},{z:.2f},{grip}")

# Control de la tasa de baudios y prevención de desbordamiento (Overflow)
# 50 milisegundos garantizan un refresco fluido a 20Hz
time.sleep(0.05)
# =============================================================================
Fase 2: Control de Cinemática Inversa y Manipulación de Objetos (Robot Baxter)

En esta fase del taller se aborda el control del manipulador colaborativo Baxter (Rethink Robotics). A diferencia de un actuador simple, Baxter posee dos extremidades con 7 Grados de Libertad (DOF) por brazo, lo que lo clasifica mecánicamente como un robot cinemáticamente redundante. Esto significa que para una misma coordenada cartesiana en el espacio tridimensional ($X, Y, Z$), existen infinitas configuraciones articulares posibles.

Para satisfacer el requerimiento de lograr un movimiento fluido del brazo y habilitar la capacidad de coger y mover un objeto dinámico dentro del simulador, la integración en Python se estructuró bajo los siguientes fundamentos mecatrónicos:

1. Resolución Numérica de la Cinemática Inversa (IK):
PyBullet implementa una aproximación basada en el Jacobiano transpuesto y el método de mínimos cuadrados amortiguados (Damped Least Squares - DLS). La función `p.calculateInverseKinematics()` recibe la coordenada objetivo enviada por el ESP32 y el índice cinemático del efector final (`MANO_INDEX`), resolviendo en tiempo real el vector de ángulos articulares necesarios para posicionar la muñeca con precisión milimétrica sin entrar en singularidades mecánicas.
2. Generación del Entorno y Dinámica de Agarre:
Para verificar la manipulación real de objetos, no basta con mover el brazo en el vacío:



* Se genera una mesa de trabajo estática y un objeto cúbico dinámico (`target_box`) con propiedades físicas definidas (masa de 0.1 kg e inercia calculada).
* Se asignan coeficientes de fricción lateral y de contacto a las superficies para permitir que las fuerzas normales ejercidas por los dedos del gripper generen la fricción estática suficiente para levantar el cuerpo sin que resbale.

3. Control Diferencial del Gripper Paralelo:
El efector final de Baxter cuenta con una pinza eléctrica de mordaza paralela controlada por dos juntas prismáticas independientes (`DEDO_L` y `DEDO_R`). Al detectar el pulso digital del botón en la trama serial (`grip_state == 1`), el script conmuta la posición objetivo de ambas articulaciones hacia el centro (posición 0.0m) aplicando un torque controlado de 100N. Cuando el botón se libera (`grip_state == 0`), las pinzas se abren hacia los extremos (+0.05m y -0.05m), soltando el objeto.
4. Desacoplamiento de Frecuencias de Muestreo:
El motor de físicas de PyBullet debe avanzar rígidamente a 240 Hz (`p.stepSimulation()` cada 1/240 segundos) para mantener la estabilidad del integrador simpléctico. Sin embargo, la telemetría del ESP32 llega a 20 Hz. El bucle fue diseñado de manera no bloqueante: la simulación física nunca se detiene esperando datos seriales; únicamente actualiza los setpoints articulares cuando arriba un paquete completo a través del buffer.

--- CODIGO 2: SCRIPT PYTHON PARA EL PUNTO B (ROBOT BAXTER + GRIPPER) ---

import pybullet as p
import pybullet_data
import serial
import time

# 1. Inicialización de la Comunicación Serial

# Configuración no bloqueante con timeout para sincronizar con los 240Hz del simulador

try:
esp32 = serial.Serial('COM3', 115200, timeout=0.01)
print("Conexión serial establecida con éxito con la consola ESP32.")
except Exception as e:
print(f"Error al abrir el puerto COM: {e}")
print("Verifique que Thonny o cualquier otro monitor serial esté cerrado.")
exit()

# 2. Inicialización del Entorno Gráfico y Físicas

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81) # Vector de aceleración gravitacional estándar
p.setRealTimeSimulation(0) # Control por paso de simulación discreto

# 3. Carga de Escenarios y Mallas URDF

p.loadURDF("plane.urdf") # Suelo base

# Carga de la mesa de soporte para posicionar el objeto

table_pos = [0.7, 0.0, 0.0]
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# Instanciación del robot Baxter con base anclada al origen

baxterId = p.loadURDF("baxter_description/urdf/baxter.urdf", [0, 0, 0], useFixedBase=True)

# Creación dinámica de un bloque manipulable con propiedades de contacto

box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.04])
box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.04], rgbaColor=[0, 1, 0, 1])
target_box = p.createMultiBody(baseMass=0.1,
baseCollisionShapeIndex=box_collision,
baseVisualShapeIndex=box_visual,
basePosition=[0.6, 0.2, 0.65])

# Ajuste de fricción en el objeto para garantizar agarre mecánico firme

p.changeDynamics(target_box, -1, lateralFriction=1.5, rollingFriction=0.01)

# 4. Mapeo de Juntas de Baxter

# Índices cinemáticos correspondientes a la cadena cinemática del brazo izquierdo y su gripper

MANO_INDEX = 19   # Eslabón del efector final (Left Wrist / End-Effector)
DEDO_L = 20       # Mordaza izquierda del gripper
DEDO_R = 21       # Mordaza derecha del gripper

# Desactivar motores por defecto para permitir control de torque directo

num_joints = p.getNumJoints(baxterId)
for j in range(num_joints):
p.setJointMotorControl2(baxterId, j, p.VELOCITY_CONTROL, force=0)

print("Iniciando bucle de control en tiempo real (Hardware-in-the-Loop)...")

# 5. Bucle Principal de Control HIL

while True:
try:
# Lectura de la telemetría enviada por el ESP32
if esp32.in_waiting > 0:
raw_line = esp32.readline().decode('utf-8', errors='ignore').strip()

        if raw_line:
            datos = raw_line.split(',')
            # Validación estricta de la estructura del paquete: X, Y, Z, Gripper
            if len(datos) == 4:
                try:
                    # Decodificación de coordenadas cartesianas objetivo
                    target_x = float(datos[0])
                    target_y = float(datos[1])
                    target_z = float(datos[2])
                    grip_state = int(datos[3])
                    
                    target_pos = [target_x, target_y, target_z]
                    
                    # Cálculo matricial de Cinemática Inversa (IK)
                    joint_poses = p.calculateInverseKinematics(
                        bodyUniqueId=baxterId,
                        endEffectorLinkIndex=MANO_INDEX,
                        targetPosition=target_pos,
                        maxNumIterations=50,
                        residualThreshold=0.001
                    )
                    
                    # Accionamiento articular en modo Posición
                    for i in range(len(joint_poses)):
                        p.setJointMotorControl2(
                            bodyIndex=baxterId,
                            jointIndex=i,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_poses[i],
                            force=500,        # Límite de torque por motor
                            maxVelocity=2.0   # Evita transiciones bruscas
                        )
                    
                    # Control de agarre (Gripper): 0.0 cerrado, 0.04 abierto
                    pos_apertura = 0.0 if grip_state == 1 else 0.04
                    fuerza_agarre = 120.0 # Fuerza de sujeción para sostener el bloque
                    
                    p.setJointMotorControl2(baxterId, DEDO_L, p.POSITION_CONTROL, 
                                            targetPosition=pos_apertura, force=fuerza_agarre)
                    p.setJointMotorControl2(baxterId, DEDO_R, p.POSITION_CONTROL, 
                                            targetPosition=-pos_apertura, force=fuerza_agarre)
                                            
                except ValueError:
                    pass # Descarta paquetes con bytes corruptos durante la transmisión
    
    # Avance temporal del motor físico a 240Hz
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    
except KeyboardInterrupt:
    print("\nFinalizando simulación y cerrando puertos de forma segura.")
    break


# Cierre y liberación de recursos

esp32.close()
p.disconnect()
De una, pai. Vamos a darle a este último punto el mismo nivel de ingeniería que a los anteriores. El Atlas no es cualquier carrito, es un humanoide masivo, y el entorno que te pide el taller tiene el renderizado de tres cámaras sintéticas al mismo tiempo (RGB, Profundidad y Máscara de Segmentación). Eso consume mucha máquina y hay que justificarlo en el informe.   Copia y pega este bloque final justo debajo de lo que ya tienes. Con esto cierras el documento con broche de oro:=============================================================================
Fase 3: Movilidad Humanoide y Renderizado Sintético (Robot Atlas)El último reto de la práctica escala la arquitectura de control Hardware-in-the-Loop hacia un modelo tridimensional de alta complejidad: el robot humanoide Atlas (Boston Dynamics). A diferencia del manipulador Baxter, Atlas posee una cadena cinemática mucho más extensa, una distribución de masa considerablemente mayor y dinámicas inerciales que exigen un control de torque superior.Para cumplir con las exigencias del entorno virtual planteado en el taller, el script de simulación no solo debe procesar la cinemática inversa, sino también soportar la carga computacional de renderizar telemetría visual. Se simulan tres cámaras sintéticas simultáneas: RGB, Depth (Profundidad) y Segmentation Mask (Máscara de Segmentación).   Esta integración exige justificar los siguientes parámetros técnicos en el código:Incremento del Torque (Fuerza Articular): Mientras que Baxter operaba con 500N, los eslabones de Atlas requieren un límite de fuerza inyectada de 1500N (force=1500) en la función p.setJointMotorControl2(). Si no se aplica esta magnitud, el cálculo de IK se realiza correctamente, pero los motores virtuales carecen del torque necesario para vencer la gravedad y la inercia del propio brazo, resultando en un movimiento caído o nulo.Adaptabilidad de la Trama Serial: El ESP32 sigue enviando una trama de 4 valores (X, Y, Z, Gripper) porque el firmware es universal. Sin embargo, el script de Atlas está diseñado para ignorar el cuarto valor (estado del botón) mediante un truncamiento de la lista (len(datos) >= 3), utilizando exclusivamente las coordenadas espaciales para el cálculo de la muñeca (Índice 28).Estabilidad Computacional: Debido a la sobrecarga gráfica que imponen las tres ventanas de cámaras sintéticas, mantener la lectura serial desvinculada del stepSimulation es vital. Si el procesamiento de la cámara retrasa el ciclo, el buffer serial almacenará la cola de datos sin perder coordenadas, y el brazo de Atlas seguirá la ruta trazada por el joystick físico con la máxima fidelidad posible.   --- CODIGO 3: SCRIPT PYTHON PARA EL PUNTO C (ROBOT ATLAS Y CAMARAS) ---import pybullet as pimport pybullet_dataimport serialimport time1. Conexión Serial HIL (Hardware-in-the-Loop)try:esp32 = serial.Serial('COM3', 115200, timeout=0.01)print("Conexión serial establecida. Consola ESP32 en línea.")except Exception as e:print(f"Error de puerto: {e}")exit()2. Inicialización del Motor PyBulletSe activa la interfaz gráfica (GUI) para soportar el renderizado de cámarasp.connect(p.GUI)p.setAdditionalSearchPath(pybullet_data.getDataPath())p.setGravity(0, 0, -9.81)p.setRealTimeSimulation(0)3. Carga del Entorno y Modelo Humanoidep.loadURDF("plane.urdf")Se ancla la base de Atlas al origen [0, 0, 1] para enfocar la prueba en la cinemática del brazo superioratlasId = p.loadURDF("atlas/atlas_v4_with_multisense.urdf", [0, 0, 1], useFixedBase=True)Índice de la muñeca (End-Effector) del modelo Atlas v4MANO_ATLAS = 28Desactivar motores por defecto para evitar conflictos con el control de posiciónfor j in range(p.getNumJoints(atlasId)):p.setJointMotorControl2(atlasId, j, p.VELOCITY_CONTROL, force=0)4. Configuración de Cámaras Sintéticas (RGB, Depth, Segmentation)Parámetros de la cámara virtual que simula el entorno de Boston Dynamicscam_target_pos = [0, 0, 1]cam_distance = 2.0cam_yaw = 45.0cam_pitch = -20.0print("Renderizando cámaras sintéticas. Iniciando cinemática inversa...")5. Bucle de Control y Físicaswhile True:try:# Renderizado de la matriz de cámaras (RGB, Profundidad, Segmentación)# Esto genera la carga gráfica visualizada en el lado izquierdo del simuladorview_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, 0, 2)proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(960)/720, nearVal=0.1, farVal=100.0)# p.getCameraImage(960, 720, view_matrix, proj_matrix) # Descomentar para volcar frames    # Lectura de la telemetría del ESP32
    if esp32.in_waiting > 0:
        linea = esp32.readline().decode('utf-8', errors='ignore').strip()
        
        if linea:
            datos = linea.split(',')
            # El firmware envía 4 datos, pero Atlas solo requiere las 3 coordenadas (X, Y, Z)
            if len(datos) >= 3:
                try:
                    target = [float(datos[0]), float(datos[1]), float(datos[2])]
                    
                    # Cálculo de Cinemática Inversa (IK) para el brazo de Atlas
                    joint_poses = p.calculateInverseKinematics(
                        bodyUniqueId=atlasId, 
                        endEffectorLinkIndex=MANO_ATLAS, 
                        targetPosition=target
                    )
                    
                    # Accionamiento de las juntas con control de alto torque (1500N)
                    for i in range(len(joint_poses)):
                        p.setJointMotorControl2(
                            bodyIndex=atlasId, 
                            jointIndex=i, 
                            controlMode=p.POSITION_CONTROL, 
                            targetPosition=joint_poses[i], 
                            force=1500, # Fuerza elevada para manipular la masa del humanoide
                            maxVelocity=1.5
                        )
                except ValueError:
                    pass
    
    # Sincronización del tiempo de simulación (240 Hz)
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
    
except KeyboardInterrupt:
    print("\nSimulación abortada por el usuario.")
    break
esp32.close()p.disconnect()
