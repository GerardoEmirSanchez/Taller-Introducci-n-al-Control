%% Control de Posición de Elevador - Sistema de 2do Orden con Control PID y Animación

% En este código se podrá encontrar el comportamiento de un sistema elevador 
% modelado como sistema de 2do orden bajo control PID. Se incluye una 
% animación en tiempo real que muestra el movimiento del elevador entre pisos,
% junto con gráficos de posición, error y fuerza del motor.
% El Controlador PID regula la posición del elevador aplicando fuerzas 
% proporcionales al error, integral para eliminar error estacionario y 
% derivativa para amortiguar las oscilaciones.
% Se recomienda a los alumnos variar las constantes P, I y D para observar 
% diferencias en: tiempo de establecimiento, sobrepico, oscilaciones 
% y suavidad del movimiento.

% Fecha: 24 de noviembre de 2025
% Autor: MSc. Gerardo Emir Sánchez Valdés


clear; clc; close all;

% Parámetros del sistema ELEVADOR
pos_ref = 10;          % Posición deseada [m] (piso destino)
pos_inicial = 0;       % Posición inicial [m] (piso actual)
vel_inicial = 0;       % Velocidad inicial [m/s]

% MODELO: G(s) = K / (τ²s² + 2ζτs + 1)
K = 1;                 % Ganancia [m/N] - Metros por Newton
tau = 2;               % Constante de tiempo [s]
zeta = 0.7;            % Coeficiente de amortiguamiento

fprintf('=== MODELO DEL ELEVADOR ===\n');
fprintf('G(s) = %.1f m/N / (%.1fs² + %.1fs + 1)\n', K, tau^2, 2*zeta*tau);

% PARÁMETROS PID
Kp = 3;               % Ganancia proporcional [N/m]
Ki = 0.5;                % Ganancia integral [N/m·s]
Kd = 4;               % Ganancia derivativa [N·s/m]

% Límites del actuador (fuerza del motor)
u_max = 1000;          % Fuerza máxima [N]
u_min = -1000;         % Fuerza mínima [N]

% Tiempo de simulación
t_sim = 20;
dt = 0.01;
t = 0:dt:t_sim;
N = length(t);

% DISCRETIZACIÓN del sistema de 2do orden
coef_posk = 1/dt^2 + (2*zeta)/(tau*dt) + 1/tau^2;
coef_posk1 = -2/dt^2 - (2*zeta)/(tau*dt);
coef_posk2 = 1/dt^2;

%% Simulación LAZO CERRADO con Control PID
pos_pid = zeros(1, N); 
pos_pid(1) = pos_inicial;
pos_pid(2) = pos_inicial;
u_pid = zeros(1, N);
error_pid = zeros(1, N);
integral = 0;
error_prev = 0;

for k = 3:N
    % Cálculo del error actual
    error_pid(k) = pos_ref - pos_pid(k-1);
    
    % Término Proporcional
    P_term = Kp * error_pid(k);
    
    % Término Integral
    integral = integral + (error_pid(k) + error_prev) * dt / 2;
    I_term = Ki * integral;
    
    % Término Derivativo
    derivative = (error_pid(k) - error_prev) / dt;
    D_term = Kd * derivative;
    
    % Señal de control total
    u_pid(k) = P_term + I_term + D_term;
    u_pid(k) = max(min(u_pid(k), u_max), u_min);

    % Anti-windup
    if u_pid(k) >= u_max || u_pid(k) <= u_min
        integral = integral - (error_pid(k) + error_prev) * dt / 2;
    end
    
    % Actualizar error anterior
    error_prev = error_pid(k);
    
    % Simulación del elevador
    pos_pid(k) = ( -coef_posk1*pos_pid(k-1) - coef_posk2*pos_pid(k-2) + ...
                  (K/tau^2)*u_pid(k) ) / coef_posk;
end

%% ANIMACIÓN DEL ELEVADOR
fprintf('\n=== INICIANDO ANIMACIÓN ===\n');

% Crear figura para animación
figure('Position', [200, 100, 1000, 800]);

% Subplot para animación
subplot(2,2,[1,2]);
axis([-3, 3, -2, pos_ref + 3]);
axis equal;
grid on;
hold on;

% Dibujar el edificio
edificio_altura = pos_ref + 2;
rectangle('Position', [-2.5, 0, 5, edificio_altura], 'FaceColor', [0.8 0.8 0.8], ...
          'EdgeColor', 'k', 'LineWidth', 2);

% Dibujar pisos
for piso = 0:floor(pos_ref)
    plot([-2.5, 2.5], [piso, piso], 'k-', 'LineWidth', 2);
    text(2.7, piso, sprintf('Piso %d', piso), 'FontSize', 10, 'FontWeight', 'bold');
end

% Dibujar piso destino
plot([-2.5, 2.5], [pos_ref, pos_ref], 'r--', 'LineWidth', 3);
text(2.7, pos_ref, 'DESTINO', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'r');

% Crear elevador
elevador_width = 1.5;
elevador_height = 2;
elevador = rectangle('Position', [-elevador_width/2, pos_inicial, elevador_width, elevador_height], ...
                    'FaceColor', [0.2 0.6 1.0], 'EdgeColor', 'b', 'LineWidth', 3, ...
                    'Curvature', 0.1);

% Crear cables
cable_left = plot([-elevador_width/2, -2], [pos_inicial + elevador_height, edificio_altura], ...
                 'k-', 'LineWidth', 2);
cable_right = plot([elevador_width/2, 2], [pos_inicial + elevador_height, edificio_altura], ...
                  'k-', 'LineWidth', 2);

% Texto informativo
info_text = text(-2.8, -1, '', 'FontSize', 12, 'FontWeight', 'bold', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');

title('ANIMACIÓN DEL ELEVADOR - CONTROL PID', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Posición Horizontal');
ylabel('Altura [m]');

% Subplot para posición vs tiempo
subplot(2,2,3);
pos_plot = plot(t(1), pos_pid(1), 'b-', 'LineWidth', 2);
hold on;
ref_line = plot([0 t_sim], [pos_ref pos_ref], 'r--', 'LineWidth', 2);
current_pos_point = plot(t(1), pos_pid(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Tiempo [s]');
ylabel('Posición [m]');
title('Trayectoria del Elevador');
legend('Posición', 'Referencia', 'Posición Actual', 'Location', 'southeast');
grid on;
xlim([0, t_sim]);
ylim([0, pos_ref + 2]);

% Subplot para fuerza vs tiempo
subplot(2,2,4);
force_plot = plot(t(1), u_pid(1), 'r-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Fuerza [N]');
title('Fuerza del Motor');
grid on;
xlim([0, t_sim]);
ylim([u_min, u_max]);

% Factor de aceleración para la animación (1 = tiempo real)
speed_factor = 2;
frame_skip = max(1, round(1/(dt * speed_factor)));

% Bucle de animación
for k = 1:frame_skip:N
    % Actualizar posición del elevador
    current_pos = pos_pid(k);
    set(elevador, 'Position', [-elevador_width/2, current_pos, elevador_width, elevador_height]);
    
    % Actualizar cables
    set(cable_left, 'YData', [current_pos + elevador_height, edificio_altura]);
    set(cable_right, 'YData', [current_pos + elevador_height, edificio_altura]);
    
    % Actualizar texto informativo
    current_time = t(k);
    current_force = u_pid(k);
    current_error = pos_ref - current_pos;
    
    info_str = sprintf('Tiempo: %.1f s\nPosición: %.2f m\nError: %.3f m\nFuerza: %.1f N', ...
                      current_time, current_pos, current_error, current_force);
    set(info_text, 'String', info_str);
    
    % Actualizar gráfico de posición
    set(pos_plot, 'XData', t(1:k), 'YData', pos_pid(1:k));
    set(current_pos_point, 'XData', t(k), 'YData', pos_pid(k));
    
    % Actualizar gráfico de fuerza
    set(force_plot, 'XData', t(1:k), 'YData', u_pid(1:k));
    
    % Forzar actualización de gráficos
    drawnow;
    
    % Pequeña pausa para controlar velocidad
    pause(0.01);
end

%% Gráficos adicionales de análisis
figure('Position', [300, 200, 1200, 600]);

% Comparación de desempeño
subplot(1,3,1);
plot(t, pos_pid, 'b-', 'LineWidth', 3);
hold on;
plot(t, pos_ref*ones(size(t)), 'r--', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Posición [m]');
title('Respuesta del Elevador - Control PID');
legend('Posición Real', 'Posición Deseada', 'Location', 'southeast');
grid on;

subplot(1,3,2);
plot(t, error_pid, 'm-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Error [m]');
title('Error de Posición vs Tiempo');
grid on;

subplot(1,3,3);
plot(t, u_pid, 'g-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Fuerza [N]');
title('Fuerza del Motor vs Tiempo');
grid on;

%% Resultados finales
fprintf('\n=== RESULTADOS DE LA SIMULACIÓN ===\n');
fprintf('Posición inicial: %.1f m\n', pos_inicial);
fprintf('Posición deseada: %.1f m\n', pos_ref);
fprintf('Posición final: %.3f m\n', pos_pid(end));
fprintf('Error estacionario: %.4f m\n', error_pid(end));
fprintf('Tiempo de simulación: %.1f s\n', t_sim);
fprintf('Fuerza máxima aplicada: %.1f N\n', max(abs(u_pid)));

% Calcular métricas de desempeño
tolerance = 0.02 * pos_ref;
settling_time_idx = find(abs(error_pid) < tolerance, 1);
if ~isempty(settling_time_idx)
    settling_time = t(settling_time_idx);
    fprintf('Tiempo de establecimiento: %.2f s\n', settling_time);
else
    fprintf('Tiempo de establecimiento: > %.1f s\n', t_sim);
end

overshoot = max(0, max(pos_pid) - pos_ref);
fprintf('Sobrepico máximo: %.3f m\n', overshoot);

fprintf('\n=== ANIMACIÓN COMPLETADA ===\n');