/*
==============================================================================
 PROGRAMA: Robot Auto-balanceado (VERSIÓN TELEMETRÍA - MATLAB)
 DESCRIPCIÓN: Recibe constantes Kp, Ki, Kd desde MATLAB y transmite 
              el ángulo y la acción de control en tiempo real.
==============================================================================
*/

#include <MsTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define TB6612_STBY 8
#define TB6612_PWMA 10
#define TB6612_PWMB 9
#define TB6612_AIN1 12
#define TB6612_AIN2 13
#define TB6612_BIN1 7
#define TB6612_BIN2 6
#define MOTOR1 2
#define MOTOR2 4

MPU6050 mpu;
BalanceCar balancecar;
KalmanFilter kalmanfilter;

volatile long count_right = 0;
volatile long count_left = 0;
int rpluse = 0, lpluse = 0;
int sumam;

float Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1;
float dt = 0.005; 
float K1 = 0.05; 
float angle0 = 1.0; 

// Variables PID dinámicas (Ya no son constantes, MATLAB las modificará)
double kp = 40.0, ki = 0.0, kd = 0.58;

double kp_speed = 4, ki_speed = 0.1058, kd_speed = 0.0;
double kp_turn = 28, ki_turn = 0, kd_turn = 0.09;

int16_t ax, ay, az, gx, gy, gz;
int front = 0, back = 0, turnl = 0, turnr = 0, spinl = 0, spinr = 0;
double setp0 = 0;
int turncount = 0;
float turnoutput = 0;
double Outputs = 0;
int speedcc = 0;

void setup() {
    Pin_Config();
    Pin_Init();
    Wire.begin();
    Serial.begin(9600); // Puerto COM a 9600 baudios
    
    mpu.initialize();
    delay(1500);
    
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;

    attachInterrupt(0, Code_left, CHANGE);
    attachPinChangeInterrupt(MOTOR2);

    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}

void loop() {
    // 1. ESCUCHAR A MATLAB: Recibir nuevas constantes Kp, Ki, Kd
    if (Serial.available() > 0) {
        // Espera el formato enviado por MATLAB: Kp,Ki,Kd\n
        kp = Serial.parseFloat();
        ki = Serial.parseFloat();
        kd = Serial.parseFloat();
        
        // Limpiar cualquier residuo en el buffer serial
        while(Serial.available() > 0) Serial.read();
    }

    // 2. ENVIAR DATOS A MATLAB: Transmisión periódica cada 40ms (25Hz)
    static unsigned long lastTx = 0;
    if (millis() - lastTx >= 40) {
        lastTx = millis();
        
        // Formato: Angulo,Señal_De_Control
        Serial.print(kalmanfilter.angle);
        Serial.print(",");
        Serial.println(balancecar.angleoutput); 
    }
}

void Timer2Isr() {
    sei(); 
    countpluse(); 
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    angleout(); // Aquí se aplica el kp, ki y kd actualizados

    speedcc++;
    if (speedcc >= 8) {
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }

    turncount++;
    if (turncount > 4) {
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);
        turncount = 0;
    }
    balancecar.posture++;

    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);
}

void angleout() {
    // Lógica PD de ángulo usando las variables modificables
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;
}

ISR(PCINT2_vect) { count_right++; }
void Code_left() { count_left++; }

void countpluse() {
    lpluse = count_left; rpluse = count_right;
    count_left = 0; count_right = 0;

    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) { rpluse = -rpluse; lpluse = -lpluse; }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)) { rpluse = rpluse; lpluse = lpluse; }
    else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) { rpluse = rpluse; lpluse = -lpluse; }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)) { rpluse = -rpluse; lpluse = lpluse; }

    balancecar.stopr += rpluse; balancecar.stopl += lpluse;
    balancecar.pulseright += rpluse; balancecar.pulseleft += lpluse;
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}

void Pin_Config() {
    pinMode(TB6612_STBY, OUTPUT); pinMode(TB6612_PWMA, OUTPUT); pinMode(TB6612_PWMB, OUTPUT);
    pinMode(TB6612_AIN1, OUTPUT); pinMode(TB6612_AIN2, OUTPUT);
    pinMode(TB6612_BIN1, OUTPUT); pinMode(TB6612_BIN2, OUTPUT);
    pinMode(MOTOR1, INPUT); pinMode(MOTOR2, INPUT);
}

void Pin_Init() {
    digitalWrite(TB6612_STBY, HIGH);
    digitalWrite(TB6612_PWMA, LOW); digitalWrite(TB6612_PWMB, LOW);
    digitalWrite(TB6612_AIN1, LOW); digitalWrite(TB6612_AIN2, HIGH);
    digitalWrite(TB6612_BIN1, HIGH); digitalWrite(TB6612_BIN2, LOW);
}

void attachPinChangeInterrupt(int pin) {
    pinMode(pin, INPUT_PULLUP);
    cli(); PCMSK2 |= bit(PCINT20); PCIFR |= bit(PCIF2); PCICR |= bit(PCIE2); sei();
}

