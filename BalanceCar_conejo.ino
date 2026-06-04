/*
==============================================================================
 PROGRAMA: Robot Auto-balanceado (VERSIÓN LIMPIA - LIBRERÍAS)
 DESCRIPCIÓN: Control PID de equilibrio, velocidad y giro.
              - Eliminado: Bluetooth, Ultrasónico, LEDs, Buzzer.
              - Todo el control se ejecuta en la interrupción de 5ms.
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
float K1 = 0.05; // Peso del acelerómetro
float angle0 = 1.0; // Ángulo mecánico de equilibrio (Cero)

// -------------- PARÁMETROS PID (MODIFICA ESTOS PARA AFINAR) --------------
// 1. PID de Ángulo (Equilibrio principal)
double kp = 40, ki = 0.0, kd = 0.58;

// 2. PID de Velocidad (Evita que el coche se desplace sin control)
double kp_speed = 4, ki_speed = 0.1058, kd_speed = 0.0;

// 3. PID de Giro (Mantiene la dirección recta)
double kp_turn = 28, ki_turn = 0, kd_turn = 0.09;
// -------------------------------------------------------------------------

// Variables del sensor MPU
int16_t ax, ay, az, gx, gy, gz;

// Variables de dirección (Todas en 0 para mantener el robot quieto)
int front = 0, back = 0;
int turnl = 0, turnr = 0;
int spinl = 0, spinr = 0;

// Variables internas del controlador
double setp0 = 0;
int turncount = 0;
float turnoutput = 0;
double Outputs = 0;
int speedcc = 0;


/*********************************************************
 * SETUP: Configuración inicial del robot
 *********************************************************/
void setup()
{
    Pin_Config();
    Pin_Init();
    
    Wire.begin();
    Serial.begin(9600);
    
    mpu.initialize();
    delay(1500); // Esperar a que el giroscopio se estabilice
    
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;

    // Configurar interrupciones de encoders
    attachInterrupt(0, Code_left, CHANGE); // Pin 2
    attachPinChangeInterrupt(MOTOR2);      // Pin 4

    // Configurar interrupción del Timer2 (Ejecuta Timer2Isr cada 5ms)
    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}


/*********************************************************
 * LOOP PRINCIPAL
 * Queda vacío. Todo el procesamiento se hace en la 
 * interrupción Timer2Isr para garantizar el tiempo exacto.
 *********************************************************/
void loop()
{
    // Puedes poner un Serial.print aquí si necesitas monitorear algo en la PC
}


/*********************************************************
 * Timer2Isr: CEREBRO DEL ROBOT (Se ejecuta cada 5ms)
 *********************************************************/
void Timer2Isr()
{
    sei(); // Habilita interrupciones globales para no perder pasos de encoder

    // --- CAPA 1: ADQUISICIÓN DE DATOS ---
    countpluse(); // Lee los pulsos de los motores
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee el IMU
    
    // Filtro de Kalman: Calcula el ángulo real limpio de ruido
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    // --- CAPA 2: CONTROL DE ÁNGULO (Rápido - 5ms) ---
    angleout();

    // --- CAPA 3: CONTROL DE VELOCIDAD (Lento - 40ms) ---
    speedcc++;
    if (speedcc >= 8)
    {
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }

    // --- CAPA 4: CONTROL DE GIRO (Medio - 20ms) ---
    turncount++;
    if (turncount > 4)
    {
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);
        turncount = 0;
    }
    balancecar.posture++;

    // --- CAPA 5: SALIDA PWM A LOS MOTORES ---
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);
}


/*********************************************************
 * FUNCIONES DE CONTROL CORE
 *********************************************************/

// Función PD para el ángulo
void angleout()
{
    // Calcula la fuerza necesaria usando Proporcional (Ángulo) y Derivativo (Velocidad de caída)
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;
}

// Interrupciones de los encoders
ISR(PCINT2_vect) { count_right++; }
void Code_left() { count_left++; }

// Lectura y cálculo de odometría (pulsos)
void countpluse()
{
    lpluse = count_left;
    rpluse = count_right;

    count_left = 0;
    count_right = 0;

    // Corrección de dirección de los pulsos según el sentido de giro (PWM)
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

    // Detección de "coche levantado" para apagar motores
    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    // Acumulación para el PID de velocidad
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}


/*********************************************************
 * CONFIGURACIÓN DE PINES Y HARDWARE
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
    PCMSK2 |= bit(PCINT20); // Pin D4 a interrupción PCINT20
    PCIFR |= bit(PCIF2);
    PCICR |= bit(PCIE2);
    sei();
}




