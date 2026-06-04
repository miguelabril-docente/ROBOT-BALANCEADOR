% ==========================================================================
% SCRIPT: interfaz_pid.m (VERSIÓN CORREGIDA DE ORDEN DE VARIABLES)
% DESCRIPCIÓN: Panel con sliders, cuadros numéricos y botones para enviar
%              las constantes Kp, Ki y Kd directamente al robot.
% ==========================================================================
clc; clear; close all;

% --- CONFIGURACIÓN DEL PUERTO COM ---
puertoCOM = 'COM5'; % <-- ¡Modifica con tu puerto real si es necesario!
baudios = 9600;

fprintf('Abriendo conexion con el robot en %s...\n', puertoCOM);
s = serialport(puertoCOM, baudios);
configureTerminator(s, "LF");
flush(s);
pause(2); % Esperar inicialización del Arduino

% --- CONSUMIDOR SILENCIOSO DE DATOS ---
configureCallback(s, "terminator", @(src, evt) readline(src));

% --- VALORES INICIALES PID ---
Kp_init = 40.0;
Ki_init = 0.0;
Kd_init = 0.58;

% Enviar valores iniciales al conectar
writeline(s, sprintf('%.2f,%.2f,%.2f\n', Kp_init, Ki_init, Kd_init));

% --- ETAPA 1: CREACIÓN DE TODOS LOS ELEMENTOS GRÁFICOS ---
fig = uifigure('Name', 'Panel de Control PID - Robot Balanceador', ...
                'Position', [100 100 480 280]);

% Elementos de la fila Kp
uilabel(fig, 'Position', [20 210 60 22], 'Text', 'Ganancia Kp:', 'FontWeight', 'bold');
sldKp = uislider(fig, 'Position', [100 220 180 3], 'Limits', [0 100], 'Value', Kp_init);
numKp = uieditfield(fig, 'numeric', 'Position', [300 210 50 22], 'Value', Kp_init);
btnKp = uibutton(fig, 'Position', [370 210 90 22], 'Text', 'Enviar Kp');

% Elementos de la fila Ki
uilabel(fig, 'Position', [20 150 60 22], 'Text', 'Ganancia Ki:', 'FontWeight', 'bold');
sldKi = uislider(fig, 'Position', [100 160 180 3], 'Limits', [0 10], 'Value', Ki_init);
numKi = uieditfield(fig, 'numeric', 'Position', [300 150 50 22], 'Value', Ki_init);
btnKi = uibutton(fig, 'Position', [370 150 90 22], 'Text', 'Enviar Ki');

% Elementos de la fila Kd
uilabel(fig, 'Position', [20 90 60 22], 'Text', 'Ganancia Kd:', 'FontWeight', 'bold');
sldKd = uislider(fig, 'Position', [100 100 180 3], 'Limits', [0 5], 'Value', Kd_init);
numKd = uieditfield(fig, 'numeric', 'Position', [300 90 50 22], 'Value', Kd_init);
btnKd = uibutton(fig, 'Position', [370 90 90 22], 'Text', 'Enviar Kd');

% Botón Maestro (Enviar Todo)
btnAll = uibutton(fig, 'Position', [140 25 200 35], 'Text', 'ENVIAR TODO', ...
                  'FontWeight', 'bold', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'white');


% --- ETAPA 2: ASIGNACIÓN DE CALLBACKS (Ahora que todas las variables existen) ---
fig.CloseRequestFcn = @(src, event) cerrarPrograma(src, s);

% Asignar funciones a los botones individuales y maestro
btnKp.ButtonPushedFcn = @(btn, event) enviarDatosPID(s, numKp, numKi, numKd);
btnKi.ButtonPushedFcn = @(btn, event) enviarDatosPID(s, numKp, numKi, numKd);
btnKd.ButtonPushedFcn = @(btn, event) enviarDatosPID(s, numKp, numKi, numKd);
btnAll.ButtonPushedFcn = @(btn, event) enviarDatosPID(s, numKp, numKi, numKd);

% Asignar sincronizaciones automáticas entre Sliders y Cuadros Numéricos
sldKp.ValueChangedFcn = @(src, event) syncEditField(numKp, src.Value);
numKp.ValueChangedFcn = @(src, event) syncSlider(sldKp, src.Value);

sldKi.ValueChangedFcn = @(src, event) syncEditField(numKi, src.Value);
numKi.ValueChangedFcn = @(src, event) syncSlider(sldKi, src.Value);

sldKd.ValueChangedFcn = @(src, event) syncEditField(numKd, src.Value);
numKd.ValueChangedFcn = @(src, event) syncSlider(sldKd, src.Value);


% --- COMPORTAMIENTO INTERNO DE LAS FUNCIONES ---

function syncEditField(efObj, val)
    efObj.Value = val;
end

% Ajustar límites dinámicos para evitar errores de desborde del slider
function syncSlider(sldObj, val)
    if val < sldObj.Limits(1), val = sldObj.Limits(1); end
    if val > sldObj.Limits(2), val = sldObj.Limits(2); end
    sldObj.Value = val;
end

% Transmisión unificada de parámetros al microcontrolador
function enviarDatosPID(serialObj, numKp, numKi, numKd)
    Kp = numKp.Value;
    Ki = numKi.Value;
    Kd = numKd.Value;
    
    cadenaFormateada = sprintf('%.2f,%.2f,%.2f\n', Kp, Ki, Kd);
    writeline(serialObj, cadenaFormateada);
    
    fprintf('>>> COMANDO TRANSMITIDO -> Kp: %.2f | Ki: %.2f | Kd: %.2f\n', Kp, Ki, Kd);
end

% Liberación controlada del puerto de comunicaciones
function cerrarPrograma(figObj, serialObj)
    clear serialObj; 
    delete(figObj);  
    fprintf('Conexion serial cerrada. Panel finalizado.\n');
end