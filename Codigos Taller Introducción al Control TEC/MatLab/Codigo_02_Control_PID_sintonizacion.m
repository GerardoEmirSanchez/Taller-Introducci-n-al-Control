%% CONTROL DE TEMPERATURA - SISTEMA DE 2DO ORDEN CON CONTROL PID
% Métodos: Simulación Básica + Sintonización Analítica por Ubicación de Polos

% Este código integra el comportamiento del modelo a lazo abierto, lazo 
% cerrado con control P y PID, junto con métodos analíticos para calcular
% las constantes PID mediante ubicación de polos.

% Fecha: 24 de noviembre de 2025
% Autor: MSc. Gerardo Emir Sánchez Valdés


clear; clc; close all;

% Parámetros del sistema
T_ref = 22;
T_amb = 15;
T_inicial = 15;

K = 1;
tau = 1;
zeta_natural = 0.5;

fprintf('=== MÉTODO ANALÍTICO DE SINTONIZACIÓN PID ===\n');
fprintf('Sistema: G(s) = %.1f/(%.1fs² + %.1fs + 1)\n', K, tau^2, 2*zeta_natural*tau);

% Polos del sistema original
polos_originales = roots([tau^2, 2*zeta_natural*tau, 1]);
fprintf('\nPolos del sistema original:\n');
fprintf('Polo 1: %.3f + %.3fi\n', real(polos_originales(1)), imag(polos_originales(1)));
fprintf('Polo 2: %.3f + %.3fi\n', real(polos_originales(2)), imag(polos_originales(2)));

%% CÁLCULO ANALÍTICO DE CONSTANTES PID
fprintf('\n=== CÁLCULO ANALÍTICO POR UBICACIÓN DE POLOS ===\n');

% Configuraciones deseadas
configuraciones = {
    'Subamortiguado'    , 0.3, 0.8;     % ζ, ωn
    'Crít. Amortig.'    , 1.0, 0.6;     % ζ, ωn  
    'Sobreamortig.'     , 1.1, 0.4      % ζ, ωn
};

colores = {'r', 'g', 'b'};
valores_pid = zeros(3, 3);

for i       = 1:size(configuraciones, 1)
    nombre  = configuraciones{i, 1};
    zeta_d  = configuraciones{i, 2};
    wn_d    = configuraciones{i, 3};
    
    fprintf('\n--- %s (ζ=%.1f, ωn=%.1f) ---\n', nombre, zeta_d, wn_d);
    
    % POLOS DESEADOS
    if zeta_d < 1
        % Subamortiguado - polos complejos conjugados
        polo_deseado = -zeta_d * wn_d + 1i * wn_d * sqrt(1 - zeta_d^2);
        polos_deseados = [polo_deseado, conj(polo_deseado)];

    elseif zeta_d == 1
        % Críticamente amortiguado - polos reales e iguales
        polos_deseados = [-wn_d, -wn_d];

    else
        % Sobreamortiguado - polos reales distintos
        polos_deseados = [-zeta_d * wn_d + wn_d * sqrt(zeta_d^2 - 1), ...
                         -zeta_d * wn_d - wn_d * sqrt(zeta_d^2 - 1)];
    end
    
    fprintf('Polos deseados: %.3f + %.3fi, %.3f + %.3fi\n', ...
            real(polos_deseados(1)), imag(polos_deseados(1)), ...
            real(polos_deseados(2)), imag(polos_deseados(2)));
    
    % Cálculo analítico de constantes PID
    % Sistema: G(s) = K/(τ²s² + 2ζτs + 1)
    % Controlador: C(s) = Kp + Ki/s + Kd s
    % Lazo cerrado: T(s) = C(s)G(s)/(1 + C(s)G(s))
    
    % Polinomio característico deseado: s² + 2ζωn s + ωn²
    % Pero con PID, el sistema se convierte en tercer orden:
    % s³ + a₂s² + a₁s + a₀
    
    % Coeficientes del polinomio característico deseado
    if zeta_d < 1
        % Para subamortiguado, agregamos un polo real rápido para el término derivativo
        polo_extra = 5 * max(abs(real(polos_deseados))); % Polo rápido
        a2 = 2*zeta_d*wn_d          + polo_extra;
        a1 = wn_d^2 + 2*zeta_d*wn_d * polo_extra;
        a0 = wn_d^2                 * polo_extra;

    elseif zeta_d == 1
        % Para Críticamente amortiguado
        polo_extra = 1* max(abs(real(polos_deseados))) ; % Polo adicional
        a2 = 2*zeta_d*wn_d           + polo_extra;
        a1 = wn_d^2 + 2*zeta_d*wn_d  * polo_extra;
        a0 = wn_d^2                  * polo_extra;

    else
        % Para Sobreamortiguado
        polo_extra = 2 * min(abs(polos_deseados)); % Polo adicional
        a2 = abs(polos_deseados(1) + polos_deseados(2) + polo_extra);
        a1 = abs(polos_deseados(1)*polos_deseados(2) + ...
                (polos_deseados(1)+polos_deseados(2))*polo_extra);
        a0 = abs(polos_deseados(1)*polos_deseados(2)*polo_extra);
    end
    
    % Sistema de ecuaciones para PID
    % Polinomio característico: τ²s³ + (2ζτ + KKd)s² + (1 + KKp)s + KKi
    % Igualamos coeficientes:
    % τ²s³ + (2ζτ + KKd)s² + (1 + KKp)s + KKi = s³ + a₂s² + a₁s + a₀
    
    % Resolviendo el sistema:
    Kd_calc = (a2 * tau^2 - 2*zeta_natural*tau) / K;
    Kp_calc = (a1 * tau^2 - 1) / K;
    Ki_calc = (a0 * tau^2) / K;
    
    % Aseguramos valores positivos
    Kp_calc = max(Kp_calc, 0.1);
    Ki_calc = max(Ki_calc, 0.01);
    Kd_calc = max(Kd_calc, 0.01);
    
    fprintf('Constantes PID calculadas:\n');
    fprintf('  Kp = %.3f\n', Kp_calc);
    fprintf('  Ki = %.3f\n', Ki_calc);
    fprintf('  Kd = %.3f\n', Kd_calc);
    
    valores_pid(i, :) = [Kp_calc, Ki_calc, Kd_calc];
end

%% VERIFICACIÓN CON SIMULACIÓN
fprintf('\n=== VERIFICACIÓN CON SIMULACIÓN ===\n');

% Parámetros de simulación
t_sim = 50; dt = 0.01;
t = 0:dt:t_sim; N = length(t);
u_max = 50; u_min = 0;

% Discretización
coef_Tk     = 1/dt^2 + (2*zeta_natural)/(tau*dt) + 1/tau^2;
coef_Tk1    = -2/dt^2 - (2*zeta_natural)/(tau*dt);
coef_Tk2    = 1/dt^2;

figure('Position', [100, 100, 1400, 800]);

resultados = cell(3, 5);

for i = 1:3
    Kp = valores_pid(i, 1);
    Ki = valores_pid(i, 2);
    Kd = valores_pid(i, 3);
    
    % Simulación
    T_pid = zeros(1, N); T_pid(1:2) = T_inicial;
    u_pid = zeros(1, N); error_pid = zeros(1, N);
    integral = 0; error_prev = 0;
    derivativo = 0; T_prev = T_inicial;
    
    for k = 3:N
        error_actual = T_ref - T_pid(k-1);
        error_pid(k) = error_actual;
        
        % Término Proporcional
        P_term = Kp * error_actual;
        
        % Término Integral
        integral = integral + error_actual * dt;
        I_term = Ki * integral;
        
        % Término Derivativo (con filtro)
        derivativo = 0.8 * derivativo + 0.2 * ((T_pid(k-1) - T_prev) / dt);
        D_term = -Kd * derivativo; % Derivativo de la salida
        
        u_pid(k) = P_term + I_term + D_term;
        u_pid(k) = max(min(u_pid(k), u_max), u_min);
        
        % Anti-windup
        if u_pid(k) >= u_max || u_pid(k) <= u_min
            integral = integral - error_actual * dt;
        end
        
        T_prev = T_pid(k-1);
        
        % Simulación del sistema
        T_pid(k) = ( -coef_Tk1*T_pid(k-1) - coef_Tk2*T_pid(k-2) + ...
                    (K/tau^2)*u_pid(k) + (1/tau^2)*T_amb ) / coef_Tk;
    end
    
    % Almacenar resultados
    resultados{i, 1} = T_pid;
    resultados{i, 2} = u_pid;
    resultados{i, 3} = error_pid;
    
    % Métricas
    sobrespico = max(0, max(T_pid) - T_ref);
    t_settle = calcular_tiempo_establecimiento(T_pid, t, T_ref);
    error_ss = abs(T_pid(end) - T_ref);
    ISE = sum(error_pid.^2) * dt;
    
    resultados{i, 4} = [sobrespico, t_settle, error_ss, ISE];
    
    % ζ observado
    zeta_obs = calcular_zeta_observado(T_pid, T_ref, t);
    resultados{i, 5} = zeta_obs;
    
    fprintf('%s:\n', configuraciones{i, 1});
    fprintf('  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n', Kp, Ki, Kd);
    fprintf('  ζ deseado=%.1f, ζ observado=%.3f\n', configuraciones{i, 2}, zeta_obs);
    fprintf('  Sobrepico: %.2f%%, t_est=%.2fs\n\n', (sobrespico/T_ref)*100, t_settle);
end

%% GRÁFICOS
subplot(2,2,1);
for i = 1:3
    plot(t, resultados{i, 1}, 'Color', colores{i}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s Kp=%.2f, Ki=%.2f, Kd=%.2f', configuraciones{i, 1}, valores_pid(i,1), valores_pid(i,2), valores_pid(i,3)));
    hold on;
end
plot(t, T_ref*ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
xlabel('Tiempo [s]'); ylabel('Temperatura [°C]');
title('Respuesta de Temperatura - Método Analítico');
legend; grid on;

subplot(2,2,2);
for i = 1:3
    plot(t, resultados{i, 3}, 'Color', colores{i}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s', configuraciones{i, 1}));
    hold on;
end
xlabel('Tiempo [s]'); ylabel('Error [°C]');
plot(t, 0*ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia');
title('Error de Temperatura');
legend; grid on;


subplot(2,2,3);
for i = 1:3
    plot(t, resultados{i, 2}, 'Color', colores{i}, 'LineWidth', 2, ...
         'DisplayName', sprintf('%s', configuraciones{i, 1}));
    hold on;
end
xlabel('Tiempo [s]'); ylabel('Potencia [W]');
title('Señal de Control');
legend; grid on;




%% ANÁLISIS DE POLOS EN LAZO CERRADO
fprintf('\n=== ANÁLISIS DE POLOS EN LAZO CERRADO ===\n');
for i = 1:3
    Kp = valores_pid(i, 1);
    Ki = valores_pid(i, 2);
    Kd = valores_pid(i, 3);
    
    % Polinomio característico en lazo cerrado:
    % τ²s³ + (2ζτ + KKd)s² + (1 + KKp)s + KKi
    polos_lc = roots([tau^2, (2*zeta_natural*tau + K*Kd), (1 + K*Kp), K*Ki]);
    
    fprintf('\n%s:\n', configuraciones{i, 1});
    fprintf('Polos en lazo cerrado:\n');
    for j = 1:length(polos_lc)
        fprintf('  Polo %d: %.3f + %.3fi\n', j, real(polos_lc(j)), imag(polos_lc(j)));
    end
end

%% FUNCIONES AUXILIARES
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

function zeta_obs = calcular_zeta_observado(T, T_ref, ~)
    % Encuentra el primer pico
    [picos, ~] = findpeaks(T);
    
    if isempty(picos) || max(picos) <= T_ref
        % No hay sobrepico - sistema sobreamortiguado o críticamente amortiguado
        zeta_obs = 1.0;
        return;
    end
    
    % Encuentra el primer pico por encima de la referencia
    idx_primer_pico = find(picos > T_ref, 1);
    
    if isempty(idx_primer_pico)
        zeta_obs = 1.0;
        return;
    end
    
    Mp = (picos(idx_primer_pico) - T_ref) / T_ref;
    
    if Mp <= 0.001
        zeta_obs = 1.0;
        return;
    end
    
    % Fórmula: Mp = exp(-ζπ/√(1-ζ²))
    if Mp >= 1
        zeta_obs = 0.01;
    else
        zeta_obs = sqrt(log(Mp)^2 / (pi^2 + log(Mp)^2));
    end
end
