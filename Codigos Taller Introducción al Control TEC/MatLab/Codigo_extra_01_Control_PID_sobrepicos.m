%% CONTROL SIN SOBREPICO - RESPUESTA GRADUAL
clear; clc; close all;

% Parámetros del sistema
T_ref = 22; T_amb = 15; T_inicial = 15;
K = 1; tau = 1; zeta_natural = 0.5;

fprintf('=== CONTROL SIN SOBREPICO - RESPUESTA GRADUAL ===\n');

% PARÁMETROS PID PARA RESPUESTA SOBREAMORTIGUADA
% Objetivo: Cero oscilaciones, respuesta suave y gradual

% Configuración 1: Muy suave (muy sobreamortiguado)
Kp_suave = 2.0;
Ki_suave = 0.8; 
Kd_suave = 8.0;

% Configuración 2: Balanceada (moderadamente sobreamortiguado)
Kp_balance = 3.5;
Ki_balance = 1.5;
Kd_balance = 6.0;

% Configuración 3: Algo más rápida pero sin sobrepico
Kp_rapida = 5.0;
Ki_rapida = 2.0;
Kd_rapida = 4.0;

configs = {
    [Kp_suave, Ki_suave, Kd_suave], 'Muy Suave', 'b';
    [Kp_balance, Ki_balance, Kd_balance], 'Balanceada', 'g';
    [Kp_rapida, Ki_rapida, Kd_rapida], 'Rápida Sin Sobrepico', 'r';
};

fprintf('Configuraciones para respuesta sin sobrepico:\n');
for i = 1:size(configs, 1)
    fprintf('%s: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', ...
            configs{i,2}, configs{i,1}(1), configs{i,1}(2), configs{i,1}(3));
end

%% SIMULACIÓN
t_sim = 40; dt = 0.01;
t = 0:dt:t_sim; N = length(t);
u_max = 50; u_min = 0;

% Discretización
coef_Tk = 1/dt^2 + (2*zeta_natural)/(tau*dt) + 1/tau^2;
coef_Tk1 = -2/dt^2 - (2*zeta_natural)/(tau*dt);
coef_Tk2 = 1/dt^2;

figure('Position', [100, 100, 1200, 800]);

for i = 1:size(configs, 1)
    Kp = configs{i, 1}(1);
    Ki = configs{i, 1}(2);
    Kd = configs{i, 1}(3);
    nombre = configs{i, 2};
    color = configs{i, 3};
    
    % Simulación
    T_pid = zeros(1, N); T_pid(1:2) = T_inicial;
    u_pid = zeros(1, N); error_pid = zeros(1, N);
    integral = 0; error_prev = 0;
    
    for k = 3:N
        error_pid(k) = T_ref - T_pid(k-1);
        P_term = Kp * error_pid(k);
        integral = integral + (error_pid(k) + error_prev) * dt / 2;
        I_term = Ki * integral;
        D_term = Kd * (error_pid(k) - error_prev) / dt;
        
        u_pid(k) = P_term + I_term + D_term;
        u_pid(k) = max(min(u_pid(k), u_max), u_min);
        
        % Anti-windup
        if u_pid(k) >= u_max || u_pid(k) <= u_min
            integral = integral - (error_pid(k) + error_prev) * dt / 2;
        end
        
        error_prev = error_pid(k);
        
        T_pid(k) = ( -coef_Tk1*T_pid(k-1) - coef_Tk2*T_pid(k-2) + ...
                    (K/tau^2)*u_pid(k) + (1/tau^2)*T_amb ) / coef_Tk;
    end
    
    % Almacenar resultados
    configs{i, 4} = T_pid;
    configs{i, 5} = u_pid;
    configs{i, 6} = error_pid;
    
    % Calcular métricas
    sobrespico = max(0, max(T_pid) - T_ref);
    t_settle = calcular_tiempo_establecimiento(T_pid, t, T_ref);
    error_ss = abs(T_pid(end) - T_ref);
    
    fprintf('\n%s:\n', nombre);
    fprintf('  Sobrepico: %.2f°C\n', sobrespico);
    fprintf('  Tiempo establecimiento: %.1f s\n', t_settle);
    fprintf('  Error estacionario: %.3f°C\n', error_ss);
    
    if sobrespico < 0.1
        fprintf('  ✅ SIN OSCILACIONES\n');
    else
        fprintf('  ❌ CON OSCILACIONES\n');
    end
end

%% GRÁFICOS
subplot(2,2,1);
for i = 1:size(configs, 1)
    plot(t, configs{i, 4}, 'Color', configs{i, 3}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s', configs{i, 2}));
    hold on;
end
plot(t, T_ref*ones(size(t)), 'k--', 'LineWidth', 2, 'DisplayName', 'Referencia');
xlabel('Tiempo [s]'); ylabel('Temperatura [°C]');
title('Respuesta de Temperatura - Sin Sobrepico');
legend; grid on; ylim([14 25]);

subplot(2,2,2);
for i = 1:size(configs, 1)
    plot(t, configs{i, 5}, 'Color', configs{i, 3}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s', configs{i, 2}));
    hold on;
end
xlabel('Tiempo [s]'); ylabel('Potencia [W]');
title('Señal de Control');
legend; grid on;

subplot(2,2,3);
for i = 1:size(configs, 1)
    plot(t, configs{i, 6}, 'Color', configs{i, 3}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s', configs{i, 2}));
    hold on;
end
xlabel('Tiempo [s]'); ylabel('Error [°C]');
title('Error de Temperatura');
legend; grid on;

% Zoom en los primeros segundos
subplot(2,2,4);
t_zoom = 15;
idx_zoom = t <= t_zoom;
for i = 1:size(configs, 1)
    plot(t(idx_zoom), configs{i, 4}(idx_zoom), 'Color', configs{i, 3}, ...
         'LineWidth', 3, 'DisplayName', sprintf('%s', configs{i, 2}));
    hold on;
end
plot(t(idx_zoom), T_ref*ones(sum(idx_zoom),1), 'k--', 'LineWidth', 2, ...
     'DisplayName', 'Referencia');
xlabel('Tiempo [s]'); ylabel('Temperatura [°C]');
title('Respuesta Inicial (Zoom)');
legend; grid on; ylim([15 23]);

%% FUNCIÓN AUXILIAR
function t_settle = calcular_tiempo_establecimiento(T, t, T_ref)
    tolerance = 0.02 * T_ref;
    for i = length(T):-1:1
        if abs(T(i) - T_ref) > tolerance
            t_settle = t(min(i+1, length(t)));
            return;
        end
    end
    t_settle = t(1);
end

%% GUÍA PARA ELIMINAR SOBREPICO
fprintf('\n=== GUÍA PARA ELIMINAR SOBREPICO ===\n');
fprintf('Para ELIMINAR OSCILACIONES y tener respuesta GRADUAL:\n');
fprintf('1. REDUCIR Kp (ganancia proporcional)\n');
fprintf('   - Menos Kp → respuesta más lenta pero más estable\n');
fprintf('   - Valores típicos: 1.5 - 4.0\n\n');

fprintf('2. AUMENTAR Kd (ganancia derivativa)\n');
fprintf('   - Más Kd → amortigua oscilaciones\n');
fprintf('   - Valores típicos: 4.0 - 10.0\n\n');

fprintf('3. AJUSTAR Ki (ganancia integral)\n');
fprintf('   - Ki muy alto puede causar oscilaciones\n');
fprintf('   - Valores típicos: 0.5 - 2.0\n\n');

fprintf('CONFIGURACIÓN RECOMENDADA:\n');
fprintf('Kp = 2.0 - 3.5, Ki = 0.8 - 1.5, Kd = 6.0 - 8.0\n');

fprintf('\n=== COMPARACIÓN CON RESPUESTA OSCILATORIA ===\n');
fprintf('Si tuvieras una configuración oscilatoria (ej: Kp=10, Kd=1):\n');
fprintf('- Reducir Kp en 60-80%%\n');
fprintf('- Aumentar Kd en 400-800%%\n');
fprintf('- Reducir Ki en 50-70%%\n');