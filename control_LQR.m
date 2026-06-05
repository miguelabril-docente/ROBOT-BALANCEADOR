%% MODELADO DE YORIHISA YAMAMOTO PARA ROBOT BALANCEADOR (TWIP)
clear; clc;

% =========================================================================
% 1. INGRESA AQUÍ TUS DATOS MEDIDOS
% =========================================================================
g = 9.81;       % Gravedad (m/s^2)
m = 0.047;      % Masa de UNA rueda (kg) -> ¡Mídela en tu balanza!
R = 0.033;      % Radio de la rueda (metros)
M = 0.542;      % Masa del cuerpo/chasis sin ruedas (kg) -> ¡Mídela en tu balanza!
W = 0.150;      % Ancho del chasis (metros)
H = 0.150;      % Alto del chasis (metros)
D = 0.080;      % Profundidad del chasis (metros)
l = 0.045;      % Distancia del eje al centro de masa (metros)

% --- PARÁMETROS ELÉCTRICOS DE LOS MOTORES (Valores genéricos para motores amarillos/DC) ---
R_m = 4.0;      % Resistencia del motor (Ohms)
K_t = 0.007;    % Constante de torque (Nm/A)
K_b = 0.007;    % Constante contra-electromotriz (V/(rad/s))

% =========================================================================
% 2. CÁLCULO DE MOMENTOS DE INERCIA (Fórmulas teóricas de Yamamoto)
% =========================================================================
J_m = 0.5 * m * R^2;                     % Inercia de la rueda (Cilindro)
J_b = (1/12) * M * (H^2 + D^2);          % Inercia del chasis (Prisma)

% =========================================================================
% 3. ECUACIONES DINÁMICAS DE LAGRANGE (Matriz de Masa y Fuerza)
% =========================================================================
% Estas expresiones resumen el complejo modelo matemático de Yamamoto
E11 = (M + 2*m) * R^2 + 2*J_m;
E12 = M * l * R;
E21 = M * l * R;
E22 = J_b + M * l^2;

Den = E11*E22 - E12*E21; % Determinante del sistema

% Factores de fricción y acoplamiento motor añadidos al sistema
alpha = (2 * K_t * K_b) / R_m;
beta  = (2 * K_t) / R_m;

% =========================================================================
% 4. CONSTRUCCIÓN DE LAS MATRICES A y B (Espacio de Estados)
% =========================================================================
% Vector de estados x = [posición; velocidad; ángulo; velocidad_angular]

A = [0, 1, 0, 0;
     0, (-E22*alpha/R)/Den, (E12*M*g*l)/Den, (E12*alpha)/Den;
     0, 0, 0, 1;
     0, (E21*alpha/R)/Den,  (-E11*M*g*l)/Den, (-E11*alpha)/Den];

B = [0;
     (E22*beta + E12*beta)/Den;
     0;
     (-E21*beta - E11*beta)/Den];

disp('--- Matriz A del sistema (Yamamoto) ---'); disp(A);
disp('--- Matriz B del sistema (Yamamoto) ---'); disp(B);

% =========================================================================
% 5. DISEÑO DEL CONTROL LQR
% =========================================================================
Q = diag([10, 1, 500, 10]); % Pesos: [Posición, Velocidad, Ángulo, Vel_Angular]
R = 1;                       % Peso de la acción de control (Voltaje/PWM)

K = lqr(A, B, Q, R);
disp('--- Vector de ganancias óptimas K para tu Arduino ---'); disp(K);


