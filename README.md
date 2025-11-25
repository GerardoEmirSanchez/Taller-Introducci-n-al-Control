# Taller de Control PID para Robótica

## Descripción del Taller

Este repositorio contiene el material completo del Taller de Control PID para Robótica, donde se exploran los fundamentos teóricos y prácticos del control PID aplicado a sistemas robóticos. El taller incluye implementaciones en Java y MATLAB de diversos sistemas de control con animaciones y análisis detallados.

## Objetivos del Taller
- Comprender los fundamentos del control PID
- Implementar controladores PID en sistemas de segundo orden
- Aplicar técnicas de identificación de sistemas
- Diseñar controladores para sistemas robóticos
- Analizar el desempeño de diferentes configuraciones PID

## Estructura del Repositorio
Taller-Control-PID-Robotica/

│
├── Documentación/  
│   └── Introducción_al_Control.pdf  
│  
├── Codigos Taller Introducción al Control TEC/  
│    
│   ├── MATLAB/  
│   │   ├── Codigo_01_Control_PID.m  
│   │   ├── Codigo_02_Control_PID_sintonizacion.m  
│   │   ├── Codigo_03_Control_Identificar_modelo.m  
│   │   ├── Codigo_04_Control_PID_ejemplo_elevador.m  
│   │   ├── Codigo_05_Control_PID_ejemplo_manipulador.m  
│   │   └── Codigo_extra_01_Control_PID_sobrepicos.m  
│   │  
│   ├── Java/  
│   │   ├── C_01_Control_PID.java  
│   │   ├── C_02_Control_PID_sintonizacion.java  
│   │   ├── C_03_Control_Identificar_modelo.java  
│   │   ├── C_04_ControlElevadorPID.java  
│   │   ├── C_05_ControlBrazoRobotico2GDL.java  
│   │   ├── C_extra_01_ControlSinSobrepico.java  
│  
└── README.md  






## Sistemas Implementados

### 1. **Control de Temperatura - Sistema de 2do Orden**
- Modelado de sistema térmico de segundo orden
- Comparación: Lazo abierto vs Control P vs Control PID
- Análisis de métricas de desempeño (ISE, tiempo de establecimiento, sobrepico)

### 2. **Identificación de Sistemas + Control PID**
- Generación de datos con señal de excitación rica en frecuencias
- Identificación por mínimos cuadrados (ARX)
- Conversión a tiempo continuo mediante transformación bilineal
- Validación del modelo identificado

### 3. **Control de Posición de Elevador**
- Sistema mecánico de segundo orden con límites de actuador
- Animación en tiempo real del movimiento del elevador
- Control PID con anti-windup
- Gráficos de posición, error y fuerza del motor

### 4. **Brazo Robótico - 2 Grados de Libertad**
- Modelo dinámico completo con gravedad
- Control PID independiente para cada articulación
- Animación del movimiento del brazo
- Análisis de torques y trayectorias

### 5. **Control Sin Sobrepico - Respuesta Gradual**
- Configuraciones PID para eliminar oscilaciones
- Tres estrategias: Muy Suave, Balanceada, Rápida Sin Sobrepico
- Guía práctica para ajustar parámetros PID
- Comparación de desempeño entre configuraciones

## 🛠️ Requisitos del Sistema

### Para MATLAB:
- MATLAB R2020a o superior
- Toolboxes: No requeridos (código independiente)

### Para Java:
- JDK 8 o superior
- No se requieren librerías externas

## 🚀 Instrucciones de Ejecución

### Ejecución en MATLAB:
1. Abrir MATLAB
2. Navegar a la carpeta de códigos MATLAB
3. Ejecutar el archivo deseado:
   ```matlab
   run('Control_Temperatura_Sistema_2do_Orden.m')

### Ejecución en Java:
1. Compilar los archivos Java:
     ```java
      javac NombreDelArchivo.java
2. Ejecutar la clase principal:
   ```java
      java NombreDelArchivo

## Métricas de Desempeño Analizadas
En todos los sistemas se evalúan:
- Error Estacionario: Precisión en estado estable
- Tiempo de Establecimiento: Velocidad de respuesta
- Sobrepico: Magnitud de las oscilaciones
- ISE (Integral Square Error): Error cuadrático integral
- Consumo Energético: Eficiencia del controlador

## Parámetros Ajustables
Los estudiantes pueden modificar:
- Constantes PID: Kp, Ki, Kd
- Condiciones iniciales del sistema
- Referencias deseadas
- Límites de los actuadores
- Tiempos de simulación

## Características Interactivas
### Animaciones Incluidas:
- Movimiento de elevador entre pisos
- Brazo robótico de 2 GDL en tiempo real
- Trayectorias del efector final
- Actualización en tiempo real de métricas

### Gráficos Interactivos:
- Múltiples pestañas de visualización
- Zoom en regiones de interés
- Leyendas dinámicas
- Comparación de configuraciones

🔍 Ejemplos de Experimentación
Para entender el efecto de cada término PID:
- Proporcional (Kp):

   ```java
      // Prueba con Kp muy alto (causa oscilaciones)
      Kp = 10.0, Ki = 0.0, Kd = 0.0 ´´´´ 

      // Prueba con Kp muy bajo (respuesta lenta)  
      Kp = 0.5, Ki = 0.0, Kd = 0.0

- Integral (Ki):

   ```java
      // Para eliminar error estacionario
      Kp = 2.0, Ki = 1.0, Kd = 0.0
      
      // Ki muy alto causa inestabilidad
      Kp = 2.0, Ki = 5.0, Kd = 0.0

- Derivativo (Kd):

   ```java
      // Para amortiguar oscilaciones
      Kp = 3.0, Ki = 1.0, Kd = 4.0

   
## Contribuciones
Este material fue desarrollado por MSc. Gerardo Emir Sánchez Valdés para fines educativos. Las contribuciones y mejoras son bienvenidas.

## Licencia
Material educativo para uso académico. Se permite la reproducción con fines educativos citando la fuente.


