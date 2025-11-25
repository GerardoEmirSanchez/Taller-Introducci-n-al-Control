%% Control de Brazo Robótico - 2 Grados de Libertad con Control PID

% En este código se podrá encontrar el comportamiento de un brazo robótico 
% de 2 GDL (Grados de Libertad) bajo control PID independiente para cada 
% articulación. Se incluye una animación en tiempo real que muestra el 
% movimiento del brazo desde su configuración inicial hasta la posición 
% deseada, junto con gráficos de ángulos, torques y trayectorias.
% Los controladores PID regulan cada articulación aplicando torques 
% proporcionales al error angular, integral para eliminar error estacionario
% y derivativa para amortiguar las oscilaciones.
% Se recomienda a los alumnos variar las constantes P, I y D para observar 
% diferencias en: precisión de posicionamiento, oscilaciones, tiempo de 
% establecimiento y consumo energético del sistema.

% Fecha: 24 de noviembre de 2025
% Autor: MSc. Gerardo Emir Sánchez Valdés


clear; clc; close all;

% Parámetros del brazo robótico
l1 = 1.0;           % Longitud del primer eslabón [m]
l2 = 0.8;           % Longitud del segundo eslabón [m]
m1 = 2.0;           % Masa del primer eslabón [kg]
m2 = 1.5;           % Masa del segundo eslabón [kg]
g = 9.81;           % Gravedad [m/s²]

% Posiciones deseadas (ángulos)
theta1_ref = pi/2;  % 60° - Articulación 1
theta2_ref = pi/4;  % 45° - Articulación 2

% Posiciones iniciales
theta1_initial = 0; % 0° - Articulación 1
theta2_initial = 0; % 0° - Articulación 2

fprintf('=== BRAZO ROBÓTICO 2 GDL ===\n');
fprintf('Longitud eslabón 1: %.1f m\n', l1);
fprintf('Longitud eslabón 2: %.1f m\n', l2);
fprintf('Posición deseada: θ1=%.1f°, θ2=%.1f°\n', rad2deg(theta1_ref), rad2deg(theta2_ref));

% PARÁMETROS PID PARA CADA ARTICULACIÓN

Kp = 30;
Ki = 0.5;
Kd = 15;

% Articulación 1
Kp1 = Kp; Ki1 = Ki; Kd1 = Kd;
% Articulación 2  
Kp2 = Kp; Ki2 = Ki; Kd2 = Kd;

% Límites de torque
tau_max = 100;      % Torque máximo [Nm]
tau_min = -100;     % Torque mínimo [Nm]

fprintf('\n=== PARÁMETROS PID ===\n');
fprintf('ARTICULACIÓN 1: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp1, Ki1, Kd1);
fprintf('ARTICULACIÓN 2: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp2, Ki2, Kd2);

% Tiempo de simulación
t_sim = 60;
dt = 0.01;
t = 0:dt:t_sim;
N = length(t);

%% Dinámica del brazo robótico (modelo simplificado)
% Matrices de inercia, Coriolis y gravedad (modelo simplificado)
I1 = m1*l1^2/3;     % Momento de inercia eslabón 1
I2 = m2*l2^2/3;     % Momento de inercia eslabón 2

% Variables de simulación
theta1 = zeros(1,N); theta1(1) = theta1_initial;
theta2 = zeros(1,N); theta2(1) = theta2_initial;
omega1 = zeros(1,N); % Velocidad angular articulación 1
omega2 = zeros(1,N); % Velocidad angular articulación 2

% Variables de control
tau1 = zeros(1,N);   % Torque articulación 1
tau2 = zeros(1,N);   % Torque articulación 2
error1 = zeros(1,N); % Error articulación 1
error2 = zeros(1,N); % Error articulación 2

% Variables PID
integral1 = 0; integral2 = 0;
error1_prev = 0; error2_prev = 0;

%% Simulación del sistema
for k = 1:N-1
    % Cálculo de errores
    error1(k) = theta1_ref - theta1(k);
    error2(k) = theta2_ref - theta2(k);
    
    % CONTROL PID ARTICULACIÓN 1
    P1 = Kp1 * error1(k);
    integral1 = integral1 + error1(k) * dt;
    I1_term = Ki1 * integral1;
    derivative1 = (error1(k) - error1_prev) / dt;
    D1 = Kd1 * derivative1;
    tau1(k) = P1 + I1_term + D1;
    
    % CONTROL PID ARTICULACIÓN 2
    P2 = Kp2 * error2(k);
    integral2 = integral2 + error2(k) * dt;
    I2_term = Ki2 * integral2;
    derivative2 = (error2(k) - error2_prev) / dt;
    D2 = Kd2 * derivative2;
    tau2(k) = P2 + I2_term + D2;
    
    % Saturación de torques
    tau1(k) = max(min(tau1(k), tau_max), tau_min);
    tau2(k) = max(min(tau2(k), tau_max), tau_min);
    
    % Anti-windup
    if tau1(k) >= tau_max || tau1(k) <= tau_min
        integral1 = integral1 - error1(k) * dt;
    end
    if tau2(k) >= tau_max || tau2(k) <= tau_min
        integral2 = integral2 - error2(k) * dt;
    end
    
    % DINÁMICA DEL BRAZO (modelo simplificado)
    % Aceleraciones angulares
    alpha1 = (tau1(k) - m2*l1*l2*sin(theta2(k))*omega2(k)^2 - ...
              (m1*g*l1/2 + m2*g*l1)*cos(theta1(k))) / I1;
    alpha2 = (tau2(k) + m2*l1*l2*sin(theta2(k))*omega1(k)^2 - ...
              m2*g*l2/2*cos(theta1(k)+theta2(k))) / I2;
    
    % Integración numérica (Euler)
    omega1(k+1) = omega1(k) + alpha1 * dt;
    omega2(k+1) = omega2(k) + alpha2 * dt;
    theta1(k+1) = theta1(k) + omega1(k) * dt;
    theta2(k+1) = theta2(k) + omega2(k) * dt;
    
    % Actualizar errores anteriores
    error1_prev = error1(k);
    error2_prev = error2(k);
end

% Último cálculo de errores
error1(N) = theta1_ref - theta1(N);
error2(N) = theta2_ref - theta2(N);

%% Cálculo de la posición del efector final
x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);
x2 = x1 + l2 * cos(theta1 + theta2);
y2 = y1 + l2 * sin(theta1 + theta2);

% Posición deseada del efector final
x_ref = l1 * cos(theta1_ref) + l2 * cos(theta1_ref + theta2_ref);
y_ref = l1 * sin(theta1_ref) + l2 * sin(theta1_ref + theta2_ref);

%% ANIMACIÓN DEL BRAZO ROBÓTICO
fprintf('\n=== INICIANDO ANIMACIÓN ===\n');

figure('Position', [100, 50, 1400, 800]);

% Subplot principal para animación
subplot(2,3,[1,2,4,5]);
axis equal;
grid on;
hold on;
axis([-2, 2, -0.5, 2.5]);
title('ANIMACIÓN BRAZO ROBÓTICO 2 GDL - CONTROL PID', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Posición X [m]');
ylabel('Posición Y [m]');

% Dibujar área de trabajo
rectangle('Position', [-2, -0.5, 4, 3], 'FaceColor', [0.95 0.95 0.95], ...
          'EdgeColor', [0.7 0.7 0.7]);

% Punto de referencia (efector final deseado)
target_point = plot(x_ref, y_ref, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
                   'DisplayName', 'Posición Deseada');

% Brazo robótico (inicial)
shoulder = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
elbow = plot(x1(1), y1(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
wrist = plot(x2(1), y2(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

link1 = plot([0, x1(1)], [0, y1(1)], 'b-', 'LineWidth', 6, 'DisplayName', 'Eslabón 1');
link2 = plot([x1(1), x2(1)], [y1(1), y2(1)], 'g-', 'LineWidth', 6, 'DisplayName', 'Eslabón 2');

% Trayectoria del efector final
trajectory = plot(x2(1), y2(1), 'm-', 'LineWidth', 1, 'DisplayName', 'Trayectoria');

% Texto informativo
info_text = text(-1.8, 2.3, '', 'FontSize', 11, 'FontWeight', 'bold', ...
                'BackgroundColor', 'white', 'EdgeColor', 'black');

legend('Location', 'northeast');

% Subplot para ángulos
subplot(2,3,3);
theta1_plot = plot(t(1), rad2deg(theta1(1)), 'b-', 'LineWidth', 2);
hold on;
theta2_plot = plot(t(1), rad2deg(theta2(1)), 'r-', 'LineWidth', 2);
ref1_line = plot([0 t_sim], rad2deg([theta1_ref theta1_ref]), 'b--', 'LineWidth', 1);
ref2_line = plot([0 t_sim], rad2deg([theta2_ref theta2_ref]), 'r--', 'LineWidth', 1);
current_theta1 = plot(t(1), rad2deg(theta1(1)), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
current_theta2 = plot(t(1), rad2deg(theta2(1)), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
xlabel('Tiempo [s]');
ylabel('Ángulo [°]');
title('Ángulos de las Articulaciones');
legend('θ₁', 'θ₂', 'θ₁ ref', 'θ₂ ref', 'Location', 'southeast');
grid on;
xlim([0, t_sim]);
ylim([0, 90]);

% Subplot para torques
subplot(2,3,6);
tau1_plot = plot(t(1), tau1(1), 'b-', 'LineWidth', 2);
hold on;
tau2_plot = plot(t(1), tau2(1), 'r-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Torque [Nm]');
title('Torques de Control');
legend('τ₁', 'τ₂', 'Location', 'southeast');
grid on;
xlim([0, t_sim]);
ylim([tau_min, tau_max]);

% Bucle de animación
speed_factor = 3;
frame_skip = max(1, round(1/(dt * speed_factor)));

for k = 1:frame_skip:N
    % Actualizar posición del brazo
    set(link1, 'XData', [0, x1(k)], 'YData', [0, y1(k)]);
    set(link2, 'XData', [x1(k), x2(k)], 'YData', [y1(k), y2(k)]);
    set(elbow, 'XData', x1(k), 'YData', y1(k));
    set(wrist, 'XData', x2(k), 'YData', y2(k));
    
    % Actualizar trayectoria
    set(trajectory, 'XData', x2(1:k), 'YData', y2(1:k));
    
    % Actualizar información
    info_str = sprintf('Tiempo: %.1f s\nθ₁: %.1f° (ref: %.1f°)\nθ₂: %.1f° (ref: %.1f°)\nPosición: (%.2f, %.2f) m\nError: %.3f m', ...
                      t(k), rad2deg(theta1(k)), rad2deg(theta1_ref), ...
                      rad2deg(theta2(k)), rad2deg(theta2_ref), ...
                      x2(k), y2(k), norm([x_ref-x2(k), y_ref-y2(k)]));
    set(info_text, 'String', info_str);
    
    % Actualizar gráficos de ángulos
    set(theta1_plot, 'XData', t(1:k), 'YData', rad2deg(theta1(1:k)));
    set(theta2_plot, 'XData', t(1:k), 'YData', rad2deg(theta2(1:k)));
    set(current_theta1, 'XData', t(k), 'YData', rad2deg(theta1(k)));
    set(current_theta2, 'XData', t(k), 'YData', rad2deg(theta2(k)));
    
    % Actualizar gráficos de torque
    set(tau1_plot, 'XData', t(1:k), 'YData', tau1(1:k));
    set(tau2_plot, 'XData', t(1:k), 'YData', tau2(1:k));
    
    drawnow;
end

%% Gráficos de análisis
figure('Position', [150, 100, 1200, 800]);

% Errores de las articulaciones
subplot(2,3,1);
plot(t, rad2deg(error1), 'b-', 'LineWidth', 2);
hold on;
plot(t, rad2deg(error2), 'r-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Error [°]');
title('Errores de las Articulaciones');
legend('Error θ₁', 'Error θ₂', 'Location', 'northeast');
grid on;

% Trayectoria del espacio de trabajo
subplot(2,3,2);
plot(x2, y2, 'b-', 'LineWidth', 2);
hold on;
plot(x_ref, y_ref, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
xlabel('Posición X [m]');
ylabel('Posición Y [m]');
title('Trayectoria del Efector Final');
legend('Trayectoria', 'Posición Deseada', 'Base', 'Location', 'southeast');
grid on;
axis equal;

% Velocidades angulares
subplot(2,3,3);
plot(t, rad2deg(omega1), 'b-', 'LineWidth', 2);
hold on;
plot(t, rad2deg(omega2), 'r-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Velocidad Angular [°/s]');
title('Velocidades de las Articulaciones');
legend('ω₁', 'ω₂', 'Location', 'northeast');
grid on;

% Error de posición del efector final
pos_error = sqrt((x2 - x_ref).^2 + (y2 - y_ref).^2);
subplot(2,3,4);
plot(t, pos_error, 'm-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Error [m]');
title('Error de Posición del Efector Final');
grid on;

% Energía del sistema
kinetic_energy = 0.5*I1*omega1.^2 + 0.5*I2*omega2.^2;
potential_energy = m1*g*l1/2*sin(theta1) + m2*g*(l1*sin(theta1) + l2/2*sin(theta1+theta2));
total_energy = kinetic_energy + potential_energy;

subplot(2,3,5);
plot(t, kinetic_energy, 'r-', 'LineWidth', 2);
hold on;
plot(t, potential_energy, 'g-', 'LineWidth', 2);
plot(t, total_energy, 'b-', 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Energía [J]');
title('Energías del Sistema');
legend('Cinética', 'Potencial', 'Total', 'Location', 'southeast');
grid on;

% Comparación de desempeño
subplot(2,3,6);
bar([rad2deg(error1(end)), rad2deg(error2(end)), pos_error(end)]);
set(gca, 'XTickLabel', {'Error θ₁ [°]', 'Error θ₂ [°]', 'Error Pos [m]'});
ylabel('Valor Final');
title('Errores en Estado Estacionario');
grid on;

%% Resultados finales
fprintf('\n=== RESULTADOS DE LA SIMULACIÓN ===\n');
fprintf('CONFIGURACIÓN INICIAL:\n');
fprintf('  θ₁: %.1f° → θ₂: %.1f°\n', rad2deg(theta1_initial), rad2deg(theta2_initial));
fprintf('CONFIGURACIÓN DESEADA:\n');
fprintf('  θ₁: %.1f° → θ₂: %.1f°\n', rad2deg(theta1_ref), rad2deg(theta2_ref));
fprintf('  Posición efector: (%.3f, %.3f) m\n', x_ref, y_ref);
fprintf('\nRESULTADOS FINALES:\n');
fprintf('  θ₁ final: %.3f° (error: %.3f°)\n', rad2deg(theta1(end)), rad2deg(error1(end)));
fprintf('  θ₂ final: %.3f° (error: %.3f°)\n', rad2deg(theta2(end)), rad2deg(error2(end)));
fprintf('  Posición final: (%.3f, %.3f) m\n', x2(end), y2(end));
fprintf('  Error posición: %.4f m\n', pos_error(end));

% Métricas de desempeño
tolerance_angle = 0.5; % 0.5 grados
tolerance_pos = 0.01;  % 1 cm

settle_time_theta1 = t(find(abs(rad2deg(error1)) < tolerance_angle, 1));
settle_time_theta2 = t(find(abs(rad2deg(error2)) < tolerance_angle, 1));
settle_time_pos = t(find(pos_error < tolerance_pos, 1));

fprintf('\nTIEMPOS DE ESTABLECIMIENTO:\n');
fprintf('  θ₁: %.2f s (para < %.1f°)\n', settle_time_theta1, tolerance_angle);
fprintf('  θ₂: %.2f s (para < %.1f°)\n', settle_time_theta2, tolerance_angle);
fprintf('  Posición: %.2f s (para < %.3f m)\n', settle_time_pos, tolerance_pos);

fprintf('\nTORQUES MÁXIMOS:\n');
fprintf('  τ₁ max: %.1f Nm\n', max(abs(tau1)));
fprintf('  τ₂ max: %.1f Nm\n', max(abs(tau2)));