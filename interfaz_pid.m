% ==========================================================================
% SCRIPT: interfaz_pid.m
% DESCRIPCIÓN: Envía constantes PID al Robot Balanceador y grafica 
%              la respuesta de ángulo y PWM en tiempo real.
% ==========================================================================
clc; clear; close all;

% --- CONFIGURACIÓN DEL PUERTO COM ---
puertoCOM = 'COM5'; % <-- ¡Modifica esto con el número de tu puerto COM!
baudios = 9600;

% --- NUEVAS CONSTANTES A PROBAR ---
Kp_test = 45.0;
Ki_test = 2.0;
Kd_test = 0.62;

fprintf('Abriendo conexion con el robot en %s...\n', puertoCOM);
s = serialport(puertoCOM, baudios);
configureTerminator(s, "LF");
flush(s);
pause(2); % Esperar inicialización del Arduino

% Enviar constantes en el formato esperado por "Serial.parseFloat()"
datosAEnviar = sprintf('%.2f,%.2f,%.2f\n', Kp_test, Ki_test, Kd_test);
writeline(s, datosAEnviar);
fprintf('¡Constantes enviadas de forma exitosa! Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp_test, Ki_test, Kd_test);

% --- CONFIGURACIÓN DE LA GRÁFICA EN TIEMPO REAL ---
figure('Name', 'Telemetria del Robot Balanceador', 'NumberTitle', 'off');
subplot(2,1,1);
lineaAngulo = animatedline('Color', 'r', 'LineWidth', 1.5);
title('Respuesta del Sistema: Angulo del Robot');
ylabel('Angulo (Grados)'); grid on;

subplot(2,1,2);
lineaControl = animatedline('Color', 'b', 'LineWidth', 1.5);
title('Señal de Control (Accion del PID)');
xlabel('Tiempo / Muestras'); ylabel('Valor PWM / Salida'); grid on;

% --- BUCLE DE ADQUISICIÓN DE DATOS ---
muestras = 0;
maxMuestras = 400; % Duración de la captura de datos (aprox 16 segundos)

fprintf('Graficando en tiempo real. Mueve el robot para ver la respuesta...\n');
while muestras < maxMuestras
    if s.NumBytesAvailable > 0
        datosEntrantes = readline(s);
        valores = str2num(datosEntrantes); %#ok<ST2NM>
        
        if length(valores) == 2
            muestras = muestras + 1;
            angulo = valores(1);
            pwm = valores(2);
            
            % Agregar puntos a las gráficas en tiempo real
            addpoints(lineaAngulo, muestras, angulo);
            addpoints(lineaControl, muestras, pwm);
            drawnow limitrate;
        end
    end
end

% Cerrar puerto de forma segura al finalizar
clear s;
fprintf('Conexion cerrada. Prueba completada.\n');
