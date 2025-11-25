%% Control de Temperatura - Sistema de 2do Orden con Control PID

% En este código se podrá encontrar el comportamiento del modelo a un
% controlador a lazo abierto, a lazo cerrado Tipo P y a lazo cerrado Tipo
% PID. El Controlador Tipo P se toma directamente de la constante Kp del
% Controlador Tipo PID. Se recomienda a los alumnos variar las constantes
% P, I y D para ver las diferencias

% Fecha: 24 de noviembre de 2025
% Autor: MSc. Gerardo Emir Sánchez Valdés



clear; clc; close all;

% Parámetros del sistema
T_ref = 22;          % Referencia deseada [°C]
T_amb = 15;          % Temperatura ambiente [°C]
T_inicial = 15;      % Temperatura inicial [°C]

% MODELO: G(s) = K / (τ²s² + 2ζτs + 1)
K = 1;               % Ganancia [°C/W] - Grados por Watt
tau = 1;             % Constante de tiempo [s]
zeta = 0.5;          % Coeficiente de amortiguamiento

fprintf('=== MODELO DEL SISTEMA ===\n');
fprintf('G(s) = %.1f°C/W / (%ds² + %.1fs + 1)\n', K, tau^2, 2*zeta*tau);

% PARÁMETROS PID
Kp = 1.5;           % Ganancia proporcional [W/°C]
Ki = 1;             % Ganancia integral [W/°C·s]
Kd = 0;             % Ganancia derivativa [W·s/°C]

% Límites del actuador
u_max = 50;         % Potencia máxima [W]
u_min = 0;          % Potencia mínima [W]

fprintf('\n=== PARÁMETROS PID ===\n');
fprintf('Kp = %.2f [W/°C]\n', Kp);
fprintf('Ki = %.2f [W/°C·s]\n', Ki);
fprintf('Kd = %.2f [W·s/°C]\n', Kd);

% Tiempo de simulación
t_sim = 20;
dt = 0.01;
t = 0:dt:t_sim;
N = length(t);

% DISCRETIZACIÓN del sistema de 2do orden
% Ecuación: τ²(d²T/dt²) + 2ζτ(dT/dt) + T(t) = K·u(t) + T_amb
% Discretización:
% [T(k) - 2T(k-1) + T(k-2)]/Δt² + (2ζ/τ)[T(k) - T(k-1)]/Δt + (1/τ²)T(k) = (K/τ²)u(k) + (1/τ²)T_amb

a1 = 2*zeta*tau;
a2 = tau^2;

% Coeficientes para la forma recurrente
coef_Tk = 1/dt^2 + (2*zeta)/(tau*dt) + 1/tau^2;
coef_Tk1 = -2/dt^2 - (2*zeta)/(tau*dt);
coef_Tk2 = 1/dt^2;

%% 1. LAZO ABIERTO
Gan_open = 2;
u_open = ones(1, N) * (T_ref - T_amb) * Gan_open;  % [W]

% Simulación LAZO ABIERTO con discretización CORRECTA
T_open = zeros(1, N);
T_open(1) = T_inicial;
T_open(2) = T_inicial; % Inicialización para k=2

for k = 3:N
    % Forma recurrente CORRECTA
    T_open(k) = ( -coef_Tk1*T_open(k-1) - coef_Tk2*T_open(k-2) + ...
                 (K/tau^2)*u_open(k) + (1/tau^2)*T_amb ) / coef_Tk;
end

error_open = T_ref - T_open;

%% 2. LAZO CERRADO con Control PID
T_pid = zeros(1, N); T_pid(1) = T_inicial;
T_pid(2) = T_inicial;
u_pid = zeros(1, N);        % Señal de control [W]
error_pid = zeros(1, N);    % Error actual
integral = 0;               % Término integral acumulado
error_prev = 0;             % Error anterior para derivativo

% Variables para almacenar componentes individuales
P_component = zeros(1, N);
I_component = zeros(1, N);
D_component = zeros(1, N);

for k = 3:N  % Comenzar en k=3 porque necesitamos T(k-2)
    % Cálculo del error actual
    error_pid(k) = T_ref - T_pid(k-1);
    
    % Término Proporcional
    P_term = Kp * error_pid(k);
    
    % Término Integral (usando integración trapezoidal)
    integral = integral + (error_pid(k) + error_prev) * dt / 2;
    I_term = Ki * integral;
    
    % Término Derivativo (filtrado para reducir ruido)
    derivative = (error_pid(k) - error_prev) / dt;
    D_term = Kd * derivative;
    
    % Almacenar componentes individuales
    P_component(k) = P_term;
    I_component(k) = I_term;
    D_component(k) = D_term;
    
    % Señal de control total
    u_pid(k) = P_term + I_term + D_term;
    
    % Saturación del actuador
    u_pid(k) = max(min(u_pid(k), u_max), u_min);

    % Anti-windup: Limitar la integral cuando hay saturación
    if u_pid(k) >= u_max || u_pid(k) <= u_min
        integral = integral - (error_pid(k) + error_prev) * dt / 2;
    end
    
    % Actualizar error anterior
    error_prev = error_pid(k);
    
    % Simulación del sistema con discretización CORRECTA
    T_pid(k) = ( -coef_Tk1*T_pid(k-1) - coef_Tk2*T_pid(k-2) + ...
                (K/tau^2)*u_pid(k) + (1/tau^2)*T_amb ) / coef_Tk;
end

%% 3. COMPARACIÓN CON CONTROL P SIMPLE (para referencia)
Kp_simple = Kp;
T_simple = zeros(1, N); T_simple(1) = T_inicial;
T_simple(2) = T_inicial;
u_simple = zeros(1, N);
error_simple = zeros(1, N);

for k = 3:N
    error_simple(k) = T_ref - T_simple(k-1);
    u_simple(k) = Kp_simple * error_simple(k);
    u_simple(k) = max(min(u_simple(k), u_max), u_min);
    
    % Simulación con discretización CORRECTA
    T_simple(k) = ( -coef_Tk1*T_simple(k-1) - coef_Tk2*T_simple(k-2) + ...
                   (K/tau^2)*u_simple(k) + (1/tau^2)*T_amb ) / coef_Tk;
end

%% 4. Gráficos
figure('Position', [100, 100, 1400, 800]);

% Temperaturas
subplot(2,2,1);
plot(t, T_open, 'b-', 'LineWidth', 2, 'DisplayName', 'Lazo Abierto');
hold on;
plot(t, T_simple, 'g-', 'LineWidth', 2, 'DisplayName', 'Control P');
plot(t, T_pid, 'r-', 'LineWidth', 2, 'DisplayName', 'Control PID');
plot(t, T_ref*ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
xlabel('Tiempo [s]'); ylabel('Temperatura [°C]');
title('Respuesta de Temperatura'); 
legend; grid on;


% Error
subplot(2,2,2);
plot(t, error_open, 'b-', 'LineWidth', 2, 'DisplayName', 'Lazo Abierto');
hold on;
plot(t, error_simple, 'g-', 'LineWidth', 2, 'DisplayName', 'Control P');
plot(t, error_pid, 'r-', 'LineWidth', 2, 'DisplayName', 'Control PID');
plot(t, 0*ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
xlabel('Tiempo [s]'); ylabel('Error [°C]');
title('Error de Temperatura'); 
legend; grid on;

% Señal de control total
subplot(2,2,3);
plot(t, u_open, 'b-', 'LineWidth', 2, 'DisplayName', 'Lazo Abierto');
hold on;
plot(t, u_simple, 'g-', 'LineWidth', 2, 'DisplayName', 'Control P');
plot(t, u_pid, 'r-', 'LineWidth', 2, 'DisplayName', 'Control PID');
xlabel('Tiempo [s]'); ylabel('Potencia [W]');
title('Señal de Control Total'); 
legend; grid on;

% Análisis de desempeño
ISE_open = sum(error_open.^2) * dt;
ISE_simple = sum(error_simple.^2) * dt;
ISE_pid = sum(error_pid.^2) * dt;

tolerance = 0.02 * T_ref;
t_settle_idx_open = find(abs(error_open) < tolerance, 1);
t_settle_idx_simple = find(abs(error_simple) < tolerance, 1);
t_settle_idx_pid = find(abs(error_pid) < tolerance, 1);

t_settle_open = t(t_settle_idx_open);
t_settle_simple = t(t_settle_idx_simple);
t_settle_pid = t(t_settle_idx_pid);

overshoot_open = max(0, max(T_open) - T_ref);
overshoot_simple = max(0, max(T_simple) - T_ref);
overshoot_pid = max(0, max(T_pid) - T_ref);


%% 5. Resultados Detallados
fprintf('\n=== RESULTADOS COMPARATIVOS ===\n');
fprintf('MÉTRICA               | LAZO ABIERTO | CONTROL P   | CONTROL PID\n');
fprintf('----------------------------------------------------------------\n');
fprintf('Error Estacionario    | %6.3f °C   | %6.3f °C   | %6.3f °C\n', ...
        error_open(end), error_simple(end), error_pid(end));
fprintf('Tiempo Establecimiento| %6.2f s     | %6.2f s     | %6.2f s\n', ...
        t_settle_open, t_settle_simple, t_settle_pid);
fprintf('Sobrepico Máximo     | %6.3f °C   | %6.3f °C   | %6.3f °C\n', ...
        overshoot_open, overshoot_simple, overshoot_pid);
fprintf('ISE                  | %6.3f      | %6.3f      | %6.3f\n', ...
        ISE_open, ISE_simple, ISE_pid);
fprintf('Potencia Final       | %6.2f W    | %6.2f W    | %6.2f W\n', ...
        u_open(end), u_simple(end), u_pid(end));


fprintf('\n=== RECOMENDACIONES PARA AJUSTAR PID ===\n');
fprintf('• Para reducir sobrepico: DISMINUIR Kp o AUMENTAR Kd\n');
fprintf('• Para acelerar respuesta: AUMENTAR Kp\n');
fprintf('• Para eliminar error estacionario: AUMENTAR Ki\n');
fprintf('• Para reducir oscilaciones: AUMENTAR Kd o DISMINUIR Ki\n');
