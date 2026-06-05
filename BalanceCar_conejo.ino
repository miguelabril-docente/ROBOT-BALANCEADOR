/*
==============================================================================
 PROGRAMA: Robot Auto-balanceado (VERSIÓN CONTROL LQR - MODELO YAMAMOTO)
 DESCRIPCIÓN: Reemplaza por completo el control PID por una ley de control
              de Espacio de Estados Óptima (LQR) basada en optimización.
              - Todo el cálculo se ejecuta en la interrupción crítica de 5ms.
==============================================================================
*/

#include <MsTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/*********************** PINES DEL HARDWARE ***********************/
// Driver de Motores TB6612
#define TB6612_STBY 8
#define TB6612_PWMA 10
#define TB6612_PWMB 9
#define TB6612_AIN1 12
#define TB6612_AIN2 13
#define TB6612_BIN1 7
#define TB6612_BIN2 6

// Encoders de los Motores
#define MOTOR1 2
#define MOTOR2 4

/********************** OBJETOS Y LIBRERÍAS **********************/
MPU6050 mpu;
BalanceCar balancecar;
KalmanFilter kalmanfilter;

/*********************** VARIABLES DE CONTROL ***********************/
// Contadores de pulsos (Encoders)
volatile long count_right = 0;
volatile long count_left = 0;
int rpluse = 0, lpluse = 0;
int sumam;

// Parámetros del Filtro Kalman
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5, C_0 = 1;
float timeChange = 5; // Intervalo en ms
float dt = timeChange * 0.001;
float K1 = 0.05;

// Peso del acelerómetro
float angle0 = -18.0; // Ángulo mecánico de equilibrio en grados (Cero)

// =========================================================================
// VARIABLES Y GANANCIAS DEL CONTROLADOR LQR (OBTENIDAS DE MATLAB)
// Vector de estados x = [Posición; Velocidad; Ángulo; Vel_Angular]
// =========================================================================
long posicion_acumulada = 0; // Historial total de pulsos para la posición (x1)

// Tus ganancias óptimas calculadas por MATLAB:
double k1 = 3.000;  // Ganancia para Posición lineal (Pulsos)
double k2 = 1.5;   // Ganancia para Velocidad lineal (Pulsos/ciclo)
double k3 = -19.0482; // Ganancia para Ángulo (Radianes)
double k4 = -1.5855;  // Ganancia para Velocidad angular (Rad/s)

// Constante de escalamiento/conversión para acoplar el PWM físico
double k_escalado_pwm = 30.0; 
// =========================================================================

// Variables del sensor MPU
int16_t ax, ay, az, gx, gy, gz;

// Variables de dirección (En 0 para mantener estabilidad estática en el sitio)
int front = 0, back = 0;
int turnl = 0, turnr = 0;
int spinl = 0, spinr = 0;

// Variables internas del controlador
int turncount = 0;
float turnoutput = 0;
double Outputs = 0;


/*********************************************************
 * SETUP: Configuración inicial del robot
 *********************************************************/
void setup()
{
    Pin_Config();
    Pin_Init();
    
    Wire.begin();
    Serial.begin(115200); // Mantenemos velocidad alta para diagnóstico
    mpu.initialize();
    delay(1500); // Esperar a que el giroscopio se estabilice
    
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;


    posicion_acumulada = 0; // <-- AGREGA ESTA LÍNEA AQUÍ
    sumam = 0;              // <-- AGREGA ESTA LÍNEA AQUÍ


    // Configurar interrupciones de encoders
    attachInterrupt(0, Code_left, CHANGE); // Pin 2
    attachPinChangeInterrupt(MOTOR2);      // Pin 4

    // Configurar interrupción del Timer2 (Ejecuta Timer2Isr cada 5ms)
    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}


/*********************************************************
 * LOOP PRINCIPAL: Monitoreo o diagnóstico
 *********************************************************/
void loop()
{
    // El lazo de control se ejecuta de manera asíncrona en la interrupción.
    // Puedes usar el loop si deseas imprimir variables para telemetría.
}


/*********************************************************
 * Timer2Isr: CEREBRO DEL ROBOT CON LQR (Cada 5ms)
 *********************************************************/
void Timer2Isr()
{
    sei(); // Habilita interrupciones globales

    // --- CAPA 1: ADQUISICIÓN DE DATOS Y FILTRADO ---
    countpluse(); // Lee 'sumam' (velocidad) e incrementa los pulsos
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
    
    // Filtro de Kalman: Ángulo en grados
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    // Integración de la odometría para el estado x1 (Posición acumulada)
    posicion_acumulada += (lpluse + rpluse);

    // --- CAPA 2: CONVERSIÓN DE UNIDADES FÍSICAS (IDIOMA MATLAB) ---
    // El modelo de Yamamoto requiere el ángulo y su velocidad en RADIANES
    double angulo_radianes = (kalmanfilter.angle + angle0) * (3.141592 / 180.0);
    double vel_angular_rads = kalmanfilter.Gyro_x * (3.141592 / 180.0);

    // Los estados de los encoders se pasan escalados como variables relativas
    double posicion_estado = (double)posicion_acumulada * 0.001; 
    double velocidad_estado = (double)sumam * 0.001;

    // --- CAPA 3: ECUACIÓN DE CONTROL LQR DIRECTA ---
    // Ley de control LQR: u = -(k1*x1 + k2*x2 + k3*x3 + k4*x4)
    double u_esfuerzo = -( (k1 * posicion_estado) + 
                           (k2 * velocidad_estado) + 
                           (k3 * angulo_radianes) + 
                           (k4 * vel_angular_rads) );

    // Convertimos el esfuerzo de control abstracto en voltaje/PWM real para tus motores
    Outputs = u_esfuerzo * k_escalado_pwm;

    // --- CAPA 4: SATURACIÓN DE SEGURIDAD INTERNA (PWM máx 255) ---
    if (Outputs > 255)  Outputs = 255;
    if (Outputs < -255) Outputs = -255;

    // Condición de apagado de seguridad: si el robot se inclina más de 40 grados, apaga motores
    if (abs(kalmanfilter.angle) > 30) {
        Outputs = 0;
        posicion_acumulada = 0; // Reinicia la odometría al caerse
    }

    // --- CAPA 5: SALIDA PWM DIRECTA A LOS MOTORES ---
    turnoutput = 0; // Sin giros por ahora para validar estabilidad pura
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, 0, 0, 0, 0, 0, 0, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);
}


/*********************************************************
 * FUNCIONES CORE DE CAPTURA DE ENCODERS
 *********************************************************/

// Interrupciones de hardware para los encoders
ISR(PCINT2_vect) { count_right++; }
void Code_left() { count_left++; }

// Lectura y cálculo de pulsos de motor
void countpluse()
{
    lpluse = count_left;
    rpluse = count_right;

    count_left = 0;
    count_right = 0;

    // Sentidos de giro de los pulsos según el signo del PWM
    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) {
        rpluse = -rpluse;
        lpluse = -lpluse;
    } else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)) {
        rpluse = rpluse;
        lpluse = lpluse;
    } else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) {
        rpluse = rpluse;
        lpluse = -lpluse;
    } else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)) {
        rpluse = -rpluse;
        lpluse = lpluse;
    }

    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}


/*********************************************************
 * CONFIGURACIÓN DE PINES DE RELEVANCIA HARDWARE
 *********************************************************/

void Pin_Config()
{
    pinMode(TB6612_STBY, OUTPUT);
    pinMode(TB6612_PWMA, OUTPUT);
    pinMode(TB6612_PWMB, OUTPUT);
    pinMode(TB6612_AIN1, OUTPUT);
    pinMode(TB6612_AIN2, OUTPUT);
    pinMode(TB6612_BIN1, OUTPUT);
    pinMode(TB6612_BIN2, OUTPUT);

    pinMode(MOTOR1, INPUT);
    pinMode(MOTOR2, INPUT);
}

void Pin_Init()
{
    digitalWrite(TB6612_STBY, HIGH);
    digitalWrite(TB6612_PWMA, LOW);
    digitalWrite(TB6612_PWMB, LOW);
    digitalWrite(TB6612_AIN1, LOW);
    digitalWrite(TB6612_AIN2, HIGH);
    digitalWrite(TB6612_BIN1, HIGH);
    digitalWrite(TB6612_BIN2, LOW);
}

void attachPinChangeInterrupt(int pin)
{
    pinMode(pin, INPUT_PULLUP);
    cli();
    PCMSK2 |= bit(PCINT20); 
    PCIFR |= bit(PCIF2);
    PCICR |= bit(PCIE2);
    sei();
}

