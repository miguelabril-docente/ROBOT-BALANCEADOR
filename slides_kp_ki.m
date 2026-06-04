% ==========================================================================
% SCRIPT: interfaz_pid.m (VERSIÓN CON SLIDERS INTERACTIVOS)
% DESCRIPCIÓN: Panel de control visual para modificar Kp, Ki y Kd usando
%              sliders, graficando ángulo y PWM en tiempo real con scroll.
% ==========================================================================
clc; clear; close all;

% --- CONFIGURACIÓN DEL PUERTO COM ---
puertoCOM = 'COM5'; % <-- ¡Modifica con tu puerto real!
baudios = 9600;

fprintf('Abriendo conexion con el robot en %s...\n', puertoCOM);
s = serialport(puertoCOM, baudios);
configureTerminator(s, "LF");
flush(s);
pause(2); % Esperar inicialización de Arduino

% --- VALORES INICIALES PID ---
Kp_init = 40.0;
Ki_init = 0.0;
Kd_init = 0.58;

% Enviar valores iniciales al arrancar
writeline(s, sprintf('%.2f,%.2f,%.2f\n', Kp_init, Ki_init, Kd_init));

% --- CREACIÓN DE LA INTERFAZ GRÁFICA (PANEL DE SLIDERS) ---
figUI = uifigure('Name', 'Panel de Sintonizacion PID', 'Position', [100 100 420 320]);

% Elementos para Kp
uilabel(figUI, 'Position', [40 250 100 22], 'Text', 'Constante Kp', 'FontWeight', 'bold');
lblKp = uilabel(figUI, 'Position', [350 220 50 22], 'Text', num2str(Kp_init));
sldKp = uislider(figUI, 'Position', [40 230 290 3], 'Limits', [0 100], 'Value', Kp_init);

% Elementos para Ki
uilabel(figUI, 'Position', [40 170 100 22], 'Text', 'Constante Ki', 'FontWeight', 'bold');
lblKi = uilabel(figUI, 'Position', [350 140 50 22], 'Text', num2str(Ki_init));
sldKi = uislider(figUI, 'Position', [40 150 290 3], 'Limits', [0 10], 'Value', Ki_init);

% Elementos para Kd
uilabel(figUI, 'Position', [40 90 100 22], 'Text', 'Constante Kd', 'FontWeight', 'bold');
lblKd = uilabel(figUI, 'Position', [350 60 50 22], 'Text', num2str(Kd_init));
sldKd = uislider(figUI, 'Position', [40 110 290 3], 'Limits', [0 5], 'Value', Kd_init);

% Asignar la función de envío automático cuando se suelta el slider
sldKp.ValueChangedFcn = @(src, event) enviarDatosPID(s, sldKp, sldKi, sldKd, lblKp, lblKi, lblKd);
sldKi.ValueChangedFcn = @(src, event) enviarDatosPID(s, sldKp, sldKi, sldKd, lblKp, lblKi, lblKd);
sldKd.ValueChangedFcn = @(src, event) enviarDatosPID(s, sldKp, sldKi, sldKd, lblKp, lblKi, lblKd);

% --- CONFIGURACIÓN DE LA GRÁFICA EN TIEMPO REAL ---
figPlot = figure('Name', 'Telemetria en Tiempo Real', 'NumberTitle', 'off');
subplot(2,1,1);
lineaAngulo = animatedline('Color', 'r', 'LineWidth', 1.5);
title('Respuesta del Sistema: Angulo del Robot');
ylabel('Angulo (Grados)'); grid on;

subplot(2,1,2);
lineaControl = animatedline('Color', 'b', 'LineWidth', 1.5);
title('Señal de Control (Accion del PID / PWM)');
xlabel('Tiempo / Muestras'); ylabel('Valor PWM'); grid on;

% --- BUCLE DE ADQUISICIÓN DE DATOS ---
muestras = 0;
ventanaMuestras = 200; % Cuántos datos mostrar en pantalla antes de hacer scroll

fprintf('Graficando en tiempo real. Mueve los sliders para ajustar el robot...\n');

% El bucle se ejecutará siempre y cuando las ventanas sigan abiertas
while ishandle(figPlot) && isvalid(figUI)
    if s.NumBytesAvailable > 0
        datosEntrantes = readline(s);
        valores = str2num(datosEntrantes); %#ok<ST2NM>
        
        if length(valores) == 2
            muestras = muestras + 1;
            angulo = valores(1);
            pwm = valores(2);
            
            % Añadir puntos a las gráficas
            addpoints(lineaAngulo, muestras, angulo);
            addpoints(lineaControl, muestras, pwm);
            
            % Efecto de OSCILOSCOPIO (Scroll horizontal continuo)
            if muestras > ventanaMuestras
                subplot(2,1,1); xlim([muestras - ventanaMuestras, muestras]);
                subplot(2,1,2); xlim([muestras - ventanaMuestras, muestras]);
            end
            
            drawnow limitrate; % Renderizado veloz de gráficos
        end
    end
    pause(0.001); % Pequeña pausa para no saturar la CPU
end

% Cerrar puerto de forma segura si se cierran las ventanas
clear s;
fprintf('Conexion cerrada correctamente.\n');

% --- FUNCIÓN INTERNA: ENVIAR DATOS POR EL PUERTO SERIAL ---
function enviarDatosPID(serialObj, sldKp, sldKi, sldKd, lblKp, lblKi, lblKd)
    % Leer valores actuales de los controles deslizantes
    Kp_val = sldKp.Value;
    Ki_val = sldKi.Value;
    Kd_val = sldKd.Value;
    
    % Actualizar los textos numéricos al lado de los sliders
    lblKp.Text = sprintf('%.2f', Kp_val);
    lblKi.Text = sprintf('%.2f', Ki_val);
    lblKd.Text = sprintf('%.2f', Kd_val);
    
    % Enviar cadena formateada a Arduino
    cadenaFormateada = sprintf('%.2f,%.2f,%.2f\n', Kp_val, Ki_val, Kd_val);
    writeline(serialObj, cadenaFormateada);
    
    fprintf('>>> COMANDO ENVIADO -> Kp: %.2f | Ki: %.2f | Kd: %.2f\n', Kp_val, Ki_val, Kd_val);
end