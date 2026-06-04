
#include <MsTimer2.h>
//The speed PID control is realized by counting the speed measuring code disk
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/***********************Pin definition***********************/
// RGB color light control pin
#define RLED A0
#define GLED A1
#define BLED A2
// The buzzer control pin
#define BUZZER 11
// TB6612 芯片控制引脚
#define TB6612_STBY 8
#define TB6612_PWMA 10
#define TB6612_PWMB 9
#define TB6612_AIN1 12
#define TB6612_AIN2 13
#define TB6612_BIN1 7
#define TB6612_BIN2 6
// 电机编码器控制引脚
#define MOTOR1 2
#define MOTOR2 4
// 超声波模块控制引脚
#define TRIG_PIN 3
#define ECHO_PIN 5
/*************************END**************************/

/**********************Instantiating an object**********************/
MPU6050 mpu; //Example Instantiate an MPU6050 object named mpu
BalanceCar balancecar;
KalmanFilter kalmanfilter;
/*************************END**************************/

/***********************Enumeration variable***********************/
enum COLOR
{
    RED = 0, // red
    GREEN,   // green
    BLUE,    // blue
    YELLOW,  // yellow
    PURPLE,  // purple
    CYAN,    // cyan
    WHITE,   // white
    ALL_OFF  // off(black)
};
/*************************END**************************/

/***********************Variable definitions***********************/
byte RX_package[11] = {0};
byte TX_package[4] = {0xA5, 0, 0, 0x5A}; // Packet header(0xA5) + original data (n*byte) + inspection(1byte) + Package the tail(0x5A)

int UT_distance = 0;
int detTime = 0;

int Serialcount = 0;
char x_axis = 0;   // Store variables along the X-axis
char y_axis = 0;   // Store variables on the Y axis
byte klaxon = 0;   // The default storage rate is1 ~ 255
byte S_Button = 0; // Store a clockwise rotation variable
byte N_Button = 0; // Store counterclockwise rotation variables

byte model_var = 0;

// Pulse calculation
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;

// Kalman_Filte
float Q_angle = 0.001, Q_gyro = 0.005; // Angular data confidence, angular velocity data confidence
float R_angle = 0.5, C_0 = 1;
float timeChange = 5;                  // Filter sampling interval in milliseconds
float dt = timeChange * 0.001;         // Note: Dt is the filter sampling time

// Angle data
float Q;
float Angle_ax;                       // The Angle of tilt calculated by acceleration
float Angle_ay;
float K1 = 0.05;                      // The weight of the accelerometer
float angle0 = 1.0;                   // Angle of mechanical balance
int slong;

double kp = 40, ki = 0.0, kd = 0.58;  // Parameters that you need to modify
//          p:5.5            i:0.1098           d:0.0
double kp_speed = 4, ki_speed = 0.1058, kd_speed = 0.0; // Parameters that you need to modify
//          p:10            i:0          d:0.09
double kp_turn = 28, ki_turn = 0, kd_turn = 0.09; // Rotary PID setting

int16_t ax, ay, az, gx, gy, gz;

int front = 0; // Forward variables
int back = 0;  // Back variables
int turnl = 0; // Turn left sign
int turnr = 0; // Turn right
int spinl = 0; // Rotate the left flag
int spinr = 0; // Rotate the flag right

// Steering PID parameters
double setp0 = 0, dpwm = 0, dl = 0;                         // Angle balance, PWM poor, dead zone,PWM1,PWM2

// Turn and rotate parameters
int turncount = 0;                                          // Calculate the steering intervention time
float turnoutput = 0;

double Setpoint;               // Angle DIP set point, input, output
double Setpoints, Outputs = 0; // speed DIP set point, input, output

int speedcc = 0;
volatile long count_right = 0; // The volatile LON type is used to ensure that the external interrupt pulse meter values are valid when used in other functions
volatile long count_left = 0;
/*************************END**************************/

/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Timer2Isr()
{
    sei();          // Enable global variables // Habilita interrupciones globales para no perder pulsos de encoders
    
    // --- CAPA 0: SEGURIDAD Y FEEDBACK SONORO ---
    if (klaxon > 0) // 喇叭开关
    {
        digitalWrite(BUZZER, !digitalRead(BUZZER));
    }
    else
    {
        digitalWrite(BUZZER, LOW); // Oscila el pin para generar sonido
    }
    
    // --- CAPA 1: ADQUISICIÓN DE DATOS (SENSORES) ---
    countpluse(); // Lee cuántos pasos han avanzado los motores en estos 5ms                                                                         // 脉冲叠加子函数
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Obtiene aceleración y giro crudos                                        // IIC获取MPU6050六轴数据 ax ay az gx gy gz
    // Filtro de Kalman: Combina acelerómetro y giroscopio para obtener el ángulo real "limpio"
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); //获取angle 角度和卡曼滤波
    
    // --- CAPA 2: CONTROL DE ÁNGULO (CAPA INTERNA / RÁPIDA) ---
    // Se ejecuta cada 5ms porque si el coche cae, debe reaccionar de inmediato

    angleout();   // Calcula el PD (Proporcional-Derivativo) para mantener el equilibrio vertical       // 角度环 PD控制
    // --- CAPA 3: CONTROL DE VELOCIDAD (CAPA EXTERNA / LENTA) ---
    speedcc++;
    if (speedcc >= 8) // into the speed loop control // Cada 8 ciclos (5ms * 8 = 40ms)
    {
        // El control de velocidad es más lento para no desestabilizar el ángulo.
        // Su objetivo es que el coche no se desplace continuamente en una dirección
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }
    // --- CAPA 4: CONTROL DE GIRO (DIRECCIÓN) ---
    turncount++;
    if (turncount > 4) // 40ms into the rotation control // Cada 20ms aproximadamente
    {
        // Calcula la diferencia de potencia entre motores para rotar o girar
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z); //旋转子函数
        turncount = 0;
    }
    balancecar.posture++;
    // --- CAPA 5: MEZCLA FINAL Y SALIDA PWM ---
    // Aquí se suman todas las capas: Ángulo + Velocidad + Giro
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB); //小车总PWM输出
    
    // --- CAPA 6: DISTANCIA (ULTRASONIDO) ---
    detTime++;
    if (detTime >= 4) // Cada 20ms mide la distancia para evitar obstáculos o seguir objeto
    {
        detTime = 0;
        UT_distance = getDistanceCentimeter();
    }
}


/*********************************************************
 * Función: getDistanceTime()
 * Objetivo: Medir el tiempo de vuelo del sonido (ida y vuelta).
 * Retorna: Tiempo en microsegundos (us).
 *********************************************************/

int getDistanceTime()
{
	long sum = 0; // Variable para acumular las lecturas (útil si se hacen varias
	long _duration; // Variable para almacenar el tiempo de respuesta del eco
	
    
    // El ciclo for está configurado de 0 a 1, por lo que se ejecuta UNA sola vez.
    // Podrías cambiar "i < 1" por "i < 5" para promediar lecturas y ganar precisión

    for (int i=0;i<1;i++)
	{
		
        // 1. Asegurar que el pin TRIG esté limpio (LOW) antes de empezar
        digitalWrite(TRIG_PIN, LOW);
		delayMicroseconds(2);
        // 2. Generar el pulso de disparo (Trigger):
        // Se envía un pulso HIGH de 10 microsegundos para que el sensor lance 8 pulsos ultrasónicos
		digitalWrite(TRIG_PIN, HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIG_PIN, LOW);

        // 3. Medir el tiempo de espera:
        // pulseIn mide cuánto tiempo el pin ECHO está en HIGH (el tiempo que tarda el sonido en volver).
        // El "10000" es un timeout: si en 10ms no vuelve el sonido, se rinde (evita que el código se bloquee)

		_duration = pulseIn(ECHO_PIN, HIGH, 10000);
		sum=sum+_duration;
    
    // 4. Manejo de error (Fuera de rango):
        // Si sum es 0, significa que el sensor no recibió eco (el objeto está muy lejos o no hay nada).
        // Se le asigna 7366 porque es el equivalente a una distancia máxima segura (~127 cm) 
        // para que el coche no tome decisiones erróneas con un valor de cero
        if (sum == 0)
        {
            sum = 7366;
        }
	}
    // Retorna el promedio de las lecturas (en este caso, sum dividido por 1
	return(int(sum/1));
}



/*********************************************************
 * Función: getDistanceCentimeter()
 * Objetivo: Convertir el tiempo de vuelo del sonido en distancia real.
 * Retorna: Distancia al objeto en centímetros (cm).
 *********************************************************/

int getDistanceCentimeter()
{
	// El cálculo se basa en la velocidad del sonido: ~343 m/s o 29.1 microsegundos por cm.
    
    // 1. (getDistanceTime() / 29): 
    // Dividimos el tiempo total entre 29 para obtener la distancia en cm.
    
    // 2. (/ 2): 
    // El sonido viaja desde el sensor hasta el objeto Y REGRESA. 
    // Dividimos entre 2 porque solo nos interesa la distancia de ida
    
    return (getDistanceTime()/29/2);
}




ISR(PCINT2_vect)
{
    count_right++;
} //Right speed dial count
void Code_left()
{
    count_left++;
} //Left speed gauge count

void setup()
{
    Pin_Config();                   // Module pin configuration
    Pin_Init();                     // Module pin initialization
    Wire.begin();                   // Join the I2C bus sequence
    Serial.begin(9600);             // Initialize the baud rate of the serial port to 9600
    mpu.initialize();               // Initialize the MPU6050
    delay(1500);                    // Wait for the system to stabilize
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    // 5ms timed interrupt Settings use timer2
    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}

void loop()
{
    attachInterrupt(0, Code_left, CHANGE); // Enable external interrupt 0     
    attachPinChangeInterrupt(MOTOR2);      // Pin D4 is interrupted externally
    RX_Information();                      // Receive Bluetooth data
    TX_Information(UT_distance);
    switch (model_var)
    {
    case 1:
        mode2();
        break;
    case 2:
        mode3();
        break;
    default:
        mode1();
        break;
    }
}

/*********************************************************
Mode2 ()
Function Function: Obstacle avoidance mode
Function parameters: None
The function returns: none
*********************************************************/
void mode2()
{
    if (UT_distance < 20)
    {
        ResetCarState();
        front = 50;
        RGB(RED);
    }
    else if (UT_distance < 30)
    {
        RGB(BLUE);
        turnr = 1;
        delay(400);
    }
    else
    {
        RGB(GREEN);
        ResetCarState();
        back = -50;
    }
}

/*********************************************************
Mode3 ()
Function Function: follow mode
Function parameters: None
The function returns: none
*********************************************************/
void mode3()
{
    if (UT_distance < 15)
    {
        ResetCarState();
        front = 50;
        delay(20);
        RGB(RED);
    }
    else if (UT_distance < 30 && UT_distance > 15)
    {
        RGB(BLUE);
        ResetCarState();
    }
    else if (UT_distance > 30 && UT_distance < 55)
    {
        RGB(GREEN);
        ResetCarState();
        back = -50;
    }
    else
    {
        RGB(BLUE);
        ResetCarState();
    }
}

/*********************************************************
Function name: RX_Information()
Function Function: Receives data packets through Bluetooth
Function parameters: None
The function returns: none
*********************************************************/
void TX_Information(byte dat)
{
    if (dat > 127)
        dat = 127;
    TX_package[1] = dat;
    TX_package[2] = TX_package[1]; // Check the sum
    Serial.write(TX_package, 4);   // Send the packet
}


/*********************************************************
 * Función: angleout()
 * Objetivo: Calcular la potencia necesaria para el equilibrio.
 * Algoritmo: Control PD (Proporcional - Derivativo).
 *********************************************************/
void angleout()
{
    
    // Esta es la ecuación maestra del equilibrio:
    // angle0: Es el ángulo donde el coche está físicamente recto (el "cero" mecánico).
    // kalmanfilter.angle: El ángulo actual calculado por el filtro.
    // kalmanfilter.Gyro_x: La velocidad a la que se está cayendo (grados/segundo)
    
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x; // PD 角度环控制
    
    /* * EXPLICACIÓN DE LOS TÉRMINOS:
     * * 1. PARTE PROPORCIONAL (kp * ángulo):
     * Es la fuerza principal. Si el coche se inclina 5 grados, los motores 
     * reaccionan con una fuerza proporcional a esos 5 grados. 
     * Si solo tuviéramos esto, el coche oscilaría como un péndulo sin parar.
     * * 2. PARTE DERIVATIVA (kd * Gyro_x):
     * Es el "amortiguador". El giroscopio mide qué tan RÁPIDO se cae el coche.
     * Si el coche se cae muy rápido, el KD añade una fuerza extra para frenar 
     * esa caída antes de que sea demasiado tarde.
     */

}

void countpluse()
{
    
    // 1. Captura de datos instantánea:
    // Se copian los valores de los contadores incrementados por las interrupciones
    lpluse = count_left;
    rpluse = count_right;

    // 2. Reinicio:
    // Ponemos a cero los contadores para empezar a contar los próximos 5ms desde cero.
    count_left = 0;
    count_right = 0;

    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)) //小车运动方向判断 后退时（PWM即电机电压为负） 脉冲数为负数
    {
        rpluse = -rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)) //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
    {
        rpluse = rpluse;
        lpluse = lpluse;
    }
    else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)) //小车运动方向判断 右旋时 左脉冲数为负数 右脉冲数为正数
    {
        rpluse = rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)) //小车运动方向判断 左旋转 右脉冲数为负数 左脉冲数为正数
    {
        rpluse = -rpluse;
        lpluse = lpluse;
    }

    // 提起判断


// --- 1. DETECCIÓN DE "COCHE LEVANTADO" (STOP DETERMINATION) ---
    // Se acumulan los pulsos en estas variables específicas ('stopr' y 'stopl').
    // La librería 'balancecar' usa estos valores para detectar si las ruedas
    // están girando "en el aire" (locas). Si detecta un giro excesivo sin que 
    // cambie el ángulo, entiende que alguien levantó el coche y apaga los motores


    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    // --- 2. ACUMULACIÓN PARA EL CONTROL DE VELOCIDAD (PI CONTROL) ---
    // Aquí sumamos los pulsos de los últimos 5ms al total acumulado.
    // 'pulseright' y 'pulseleft' son las variables que el PID de velocidad 
    // revisará cada 40ms para intentar que vuelvan a cero (mantener la posición)

    // 每5ms进入中断时，脉冲数叠加
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    
    // --- 3. CÁLCULO DE LA VELOCIDAD TOTAL (SUMAM) ---
    // sumam representa la velocidad promedio combinada de ambos motores.
    // Se multiplica por 4 para aumentar la resolución (escala) del dato, 
    // permitiendo que el PID de velocidad reaccione con más precisión ante 
    // cambios muy pequeños en el movimiento de las ruedas

    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}

/*********************************************************
函数名称: mode1()
函数功能: 通过蓝牙控制小车方向与转速
函数参数: 无
函数返回: 无
*********************************************************/
void mode1()
{
    RGB(GREEN);
    if (N_Button > 0) // 逆时针旋转
    {
        spinl = 1;
    }
    else if (S_Button > 0) // 顺时针旋转
    {
        spinr = 1;
    }
    else if (x_axis >= -30 && x_axis <= 30 && y_axis > 30) // 向前运动
    {
        ResetCarState();
        back = -80;
    }
    else if (x_axis >= -30 && x_axis <= 30 && y_axis < -30) // 向后运动
    {
        ResetCarState();
        front = 80;
    }
    else if (y_axis >= -30 && y_axis <= 30 && x_axis < -30) // 左转
    {
        turnr = 1;
    }
    else if (y_axis >= -30 && y_axis <= 30 && x_axis > 30) // 右转
    {
        turnl = 1;
    }
    else if (y_axis > -30 && y_axis < 30 && x_axis < 30 && x_axis > -30 && S_Button == 0 && N_Button == 0)
    {
        ResetCarState();
    }

    else // 停止
    {
        RGB(RED);
    }
}

void ResetCarState()
{
    turnl = 0;
    turnr = 0;
    front = 0;
    back = 0;
    spinl = 0;
    spinr = 0;
    turnoutput = 0;
}

/*********************************************************
函数名称: RX_Information()
函数功能: 通过蓝牙接收数据包
函数参数: 无
函数返回: 无
*********************************************************/
/*********************************************************
 * Función: RX_Information()
 * Objetivo: Recibir e interpretar los comandos enviados por Bluetooth.
 * Protocolo: Paquete de 11 bytes.
 *********************************************************/
 
void RX_Information(void)
{
    if (Serial.available() > 0)
    {
        delay(1); // 延时 1MS
        if (Serial.readBytes(RX_package, 11))
        {
            if (RX_package[0] == 0xA5 && RX_package[10] == 0x5A) // 只验证了包头与包尾，暂未验证检验码
            {
                Serialcount = 0;
                x_axis = RX_package[1];   // X轴数值
                y_axis = RX_package[2];   // Y轴数值
                klaxon = RX_package[3];   // 喇叭
                N_Button = RX_package[4]; // 逆时针旋转
                S_Button = RX_package[5]; // 顺时针旋转
                if (RX_package[6] > 0)
                {
                    ResetCarState();
                    RGB(GREEN);
                    model_var = 0;
                }
                else if (RX_package[7] > 0)
                {
                    ResetCarState();
                    RGB(BLUE);
                    model_var = 1;
                }
                else if (RX_package[8] > 0)
                {
                    ResetCarState();
                    RGB(PURPLE);
                    model_var = 2;
                }
            }
            else
            {
                while (Serial.read() >= 0)
                    ; // 清除串口缓存
                Serialcount++;
                return;
            }
        }
    }
    else
    {
        Serialcount++;
        if (Serialcount > 300)
        {
            klaxon = 0;
            x_axis = 0;
            y_axis = 0;
            N_Button = 0;
            S_Button = 0;
        }
    }
}

/*********************************************************
函数名称: Pin_Init()
函数功能: 初始化引脚高低电平
函数参数: 无
函数返回: 无
*********************************************************/
void Pin_Init()
{
    digitalWrite(BUZZER, LOW); // 蜂鸣器控制引脚输出低电平

    digitalWrite(TB6612_STBY, HIGH); // TB6612 使能控制引脚输出高电平
    digitalWrite(TB6612_PWMA, LOW);  // TB6612 PWMA 控制引脚输出低电平
    digitalWrite(TB6612_PWMB, LOW);  // TB6612 PWMB 控制引脚输出低电平
    digitalWrite(TB6612_AIN1, LOW);  // TB6612 AIN1 控制引脚输出低电平
    digitalWrite(TB6612_AIN2, HIGH); // TB6612 AIN2 控制引脚输出高电平
    digitalWrite(TB6612_BIN1, HIGH); // TB6612 BIN1 控制引脚输出高电平
    digitalWrite(TB6612_BIN2, LOW);  // TB6612 BIN2 控制引脚输出低电平
}

/*********************************************************
函数名称: attachPinChangeInterrupt()
函数功能: 配置 D4 引脚为外部中断
函数参数: pin, 4
函数返回: 无
*********************************************************/

/*********************************************************
 * Función: attachPinChangeInterrupt(int pin)
 * Objetivo: Configurar el pin D4 (PCINT20) para que actúe como 
 * una interrupción de cambio de estado.
 *********************************************************/

void attachPinChangeInterrupt(int pin)
{
    
    // 1. Configura el pin como entrada con resistencia Pull-up interna.
    // Esto evita que el pin "flote" y genere lecturas falsas de pulsos
    pinMode(pin, INPUT_PULLUP);
    
    // 2. Desactiva las interrupciones globales temporalmente (cli = Clear Interrupts).
    // Es una medida de seguridad mientras modificamos los registros de hardware
    cli();
    
    // 3. PCMSK2 (Pin Change Mask Register 2):
    // Habilita específicamente el pin PCINT20. En el Arduino Uno, 
    // el pin digital 4 corresponde al PCINT20

    PCMSK2 |= bit(PCINT20);
    // 4. PCIFR (Pin Change Interrupt Flag Register):
    // Limpia cualquier bandera de interrupción previa en el puerto 2 
    // para evitar que se dispare una interrupción por error al activar el sistema
    PCIFR |= bit(PCIF2);
    // 5. PCICR (Pin Change Interrupt Control Register):
    // Activa el grupo de interrupciones PCIE2, que es el que supervisa 
    // los pines del D0 al D7.
    PCICR |= bit(PCIE2);
    // 6. Reactiva las interrupciones globales (sei = Set Enable Interrupts).
    // A partir de este momento, cualquier cambio en D4 ejecutará el ISR(PCINT2_vect)
    sei();
}

/*********************************************************
函数名称: Pin_Config()
函数功能: 配置引脚输入输出模式
函数参数: 无
函数返回: 无
*********************************************************/
void Pin_Config()
{
    pinMode(RLED, OUTPUT); // RGB 彩灯红色控制引脚配置输出
    pinMode(GLED, OUTPUT); // RGB 彩灯绿色控制引脚配置输出
    pinMode(BLED, OUTPUT); // RGB 彩灯蓝色控制引脚配置输出

    pinMode(BUZZER, OUTPUT); // 蜂鸣器控制引脚配置输出

    pinMode(TB6612_STBY, OUTPUT); // TB6612 使能控制引脚配置输出
    pinMode(TB6612_PWMA, OUTPUT); // TB6612 PWMA 控制引脚配置输出
    pinMode(TB6612_PWMB, OUTPUT); // TB6612 PWMB 控制引脚配置输出
    pinMode(TB6612_AIN1, OUTPUT); // TB6612 AIN1 控制引脚配置输出
    pinMode(TB6612_AIN2, OUTPUT); // TB6612 AIN2 控制引脚配置输出
    pinMode(TB6612_BIN1, OUTPUT); // TB6612 BIN1 控制引脚配置输出
    pinMode(TB6612_BIN2, OUTPUT); // TB6612 BIN2 控制引脚配置输出

    pinMode(MOTOR1, INPUT); // 编码电机1 控制引脚配置输入
    pinMode(MOTOR2, INPUT); // 编码电机2 控制引脚配置输入

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

/*********************************************************
函数名称: RGB()
函数功能: 控制 RGB 输出各种颜色
函数参数: color, 各种颜色参考枚举@ COLOR
函数返回: 无
*********************************************************/
void RGB(enum COLOR color)
{
    switch (color)
    {
    case RED:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    case GREEN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case BLUE:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case YELLOW:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case PURPLE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case CYAN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    case WHITE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    default:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    }
}
