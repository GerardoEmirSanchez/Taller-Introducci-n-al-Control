%% IDENTIFICACIÓN DE SISTEMAS + CONTROL PID - MÉTODO ANALÍTICO

% En este código se podrá encontrar la identificación de un sistema de 2do 
% orden mediante el método de mínimos cuadrados (sin Toolboxes) y posterior
% sintonización de controladores PID utilizando el método analítico de 
% ubicación de polos. Se incluye generación de datos, identificación 
% paramétrica, validación del modelo y diseño de controladores para 
% diferentes especificaciones de desempeño. Se recomienda a los alumnos 
% variar las señales de excitación, órdenes del modelo y ubicación de polos
% deseados para ver las diferencias en la identificación y control.

% Fecha: 24 de noviembre de 2025  
% Autor: MSc. Gerardo Emir Sánchez Valdés


clear; clc; close all;

%% PARÁMETROS DEL SISTEMA REAL (SOLO PARA SIMULACIÓN - DESCONOCIDO EN LA PRÁCTICA)
K_real = 1.2;       % Ganancia real
tau_real = 1.5;     % Constante de tiempo real
zeta_real = 0.6;    % Coeficiente de amortiguamiento real

fprintf('=== SISTEMA REAL (DESCONOCIDO) ===\n');
fprintf('G(s) = %.2f/(%.2fs² + %.2fs + 1)\n', K_real, tau_real^2, 2*zeta_real*tau_real);

%% 1. SIMULACIÓN DEL SISTEMA REAL (GENERACIÓN DE DATOS)
% fprintf('\n=== GENERANDO DATOS DE ENTRADA/SALIDA ===\n');

% Parámetros de simulación
t_sim = 50; dt = 0.01;
t = 0:dt:t_sim;
N = length(t);

% Señal de excitación rica en frecuencias
% fprintf('Generando señal de excitación...\n');
u = 3 * ones(1, N);  % Escalón base

% Agregar componentes de diferentes frecuencias
freqs = [0.05, 0.1, 0.2, 0.5];
amplitudes = [1.0, 0.8, 0.5, 0.3];
for i = 1:length(freqs)
    u = u + amplitudes(i) * sin(2*pi*freqs(i)*t);
end

% Agregar ruido blanco pequeño
u = u + 0.2 * randn(1, N);
u = max(min(u, 6), 0); % Limitar señal

% Simular sistema real (segundo orden)
% fprintf('Simulando sistema real...\n');
y_real = zeros(1, N);
y_real(1:2) = 0; % Condición inicial

% Coeficientes de discretización (sistema de segundo orden)
coef_yk = 1/dt^2 + (2*zeta_real)/(tau_real*dt) + 1/tau_real^2;
coef_yk1 = -2/dt^2 - (2*zeta_real)/(tau_real*dt);
coef_yk2 = 1/dt^2;

for k = 3:N
    y_real(k) = ( -coef_yk1*y_real(k-1) - coef_yk2*y_real(k-2) + ...
                 (K_real/tau_real^2)*u(k) ) / coef_yk;
end

% Agregar ruido de medición
ruido_medicion = 0.05 * std(y_real) * randn(1, N);
y_medida = y_real + ruido_medicion;

% fprintf('Datos generados: %d puntos de muestreo\n', N);

%% 2. ANÁLISIS EXPLORATORIO DE DATOS
% fprintf('\n=== ANÁLISIS EXPLORATORIO DE DATOS ===\n');
% 
% figure('Position', [100, 100, 1200, 800]);
% 
% subplot(1,1,1);
% plot(t, u, 'b', 'LineWidth', 1.5);
% title('Señal de Entrada (Excitación)');
% xlabel('Tiempo [s]'); ylabel('Amplitud');
% grid on;

% subplot(2,3,2);
% plot(t, y_medida, 'r', 'LineWidth', 1.5);
% title('Señal de Salida Medida');
% xlabel('Tiempo [s]'); ylabel('Amplitud');
% grid on;
% 
% subplot(2,3,3);
% plot(t, u, 'b', 'LineWidth', 1); hold on;
% plot(t, y_medida, 'r', 'LineWidth', 1);
% title('Entrada vs Salida');
% xlabel('Tiempo [s]'); ylabel('Amplitud');
% legend('Entrada u(t)', 'Salida y(t)');
% grid on;
% 
% % Análisis estadístico básico
% subplot(2,3,4);
% histogram(y_medida, 30);
% title('Distribución de la Salida');
% xlabel('Amplitud'); ylabel('Frecuencia');
% 
% % Retardo aproximado
% subplot(2,3,5);
% [corr_vals, lags] = xcorr(y_medida - mean(y_medida), u - mean(u), 30, 'coeff');
% plot(lags*dt, corr_vals, 'g', 'LineWidth', 2);
% title('Correlación Cruzada');
% xlabel('Retardo [s]'); ylabel('Correlación');
% grid on;
% 
% % Espectro de frecuencia
% subplot(2,3,6);
% N_fft = 2^nextpow2(N);
% Y_fft = fft(y_medida - mean(y_medida), N_fft);
% f = (0:N_fft-1)/(N_fft*dt);
% plot(f(1:N_fft/2), abs(Y_fft(1:N_fft/2)), 'm', 'LineWidth', 1.5);
% title('Espectro de Frecuencia de la Salida');
% xlabel('Frecuencia [Hz]'); ylabel('Magnitud');
% grid on;

%% 3. IDENTIFICACIÓN POR MÍNIMOS CUADRADOS (ARX)
fprintf('\n=== IDENTIFICACIÓN POR MÍNIMOS CUADRADOS ===\n');

% Definir estructura del modelo (segundo orden)
na = 2;  % Número de polos
nb = 1;  % Número de ceros + 1
nk = 1;  % Retardo

% fprintf('Estructura del modelo: na=%d, nb=%d, nk=%d\n', na, nb, nk);

% Preparar matriz de regresión
N_data = N - max(na, nb+nk-1);
Phi = zeros(N_data, na + nb);
Y = zeros(N_data, 1);

idx = 1;
for k = max(na, nb+nk-1)+1:N
    % Términos AR (salidas pasadas)
    for i = 1:na
        Phi(idx, i) = -y_medida(k-i);
    end
    
    % Términos X (entradas pasadas)
    for j = 1:nb
        Phi(idx, na + j) = u(k - (j + nk - 1));
    end
    
    Y(idx) = y_medida(k);
    idx = idx + 1;
end

% Estimación por mínimos cuadrados
theta = Phi \ Y;

% Extraer parámetros del modelo
a1 = theta(1);
a2 = theta(2);
b1 = theta(3);

% fprintf('Parámetros identificados:\n');
% fprintf('a1 = %.4f, a2 = %.4f, b1 = %.4f\n', a1, a2, b1);

% Función de transferencia discreta manual
num_d = [b1, 0, 0];  % b1 * z^0
den_d = [1, a1, a2]; % z^2 + a1*z + a2

fprintf('\nModelo discreto identificado:\n');
fprintf('G(z) = (%.4f) / (z² + %.4f z + %.4f)\n', b1, a1, a2);

%% 4. CONVERSIÓN A TIEMPO CONTINUO (APROXIMACIÓN)
fprintf('\n=== CONVERSIÓN A TIEMPO CONTINUO ===\n');

% Usar transformación bilineal (Tustin)
syms z s
T = dt;

% Aplicar transformación bilineal: z = (1 + sT/2)/(1 - sT/2)
% G(z) = b1 / (z² + a1*z + a2)
% Sustituir z = (1 + sT/2)/(1 - sT/2)

% Numerador y denominador después de transformación
num_s = b1 * (1 - s*T/2)^2;
den_s = (1 + s*T/2)^2 + a1*(1 + s*T/2)*(1 - s*T/2) + a2*(1 - s*T/2)^2;

% Simplificar y extraer coeficientes
num_s_expanded = expand(num_s);
den_s_expanded = expand(den_s);

% Coeficientes del numerador y denominador
coef_num = double(coeffs(num_s_expanded, s, 'All'));
coef_den = double(coeffs(den_s_expanded, s, 'All'));

% Normalizar para que el término s² en el denominador sea 1
if abs(coef_den(1)) > 1e-10
    coef_den_norm = coef_den / coef_den(1);
    coef_num_norm = coef_num / coef_den(1);
else
    coef_den_norm = coef_den;
    coef_num_norm = coef_num;
end

fprintf('Modelo continuo aproximado:\n');
fprintf('G(s) = (%.4f s² + %.4f s + %.4f) / (s² + %.4f s + %.4f)\n', ...
        coef_num_norm(1), coef_num_norm(2), coef_num_norm(3), ...
        coef_den_norm(2), coef_den_norm(3));

% Extraer parámetros del modelo continuo
K_ident = coef_num_norm(3) / coef_den_norm(3);  % Ganancia DC
tau_ident = 1/sqrt(coef_den_norm(3));           % Constante de tiempo
zeta_ident = coef_den_norm(2)/(2*tau_ident);    % Coeficiente de amortiguamiento

fprintf('\nParámetros del sistema identificado:\n');
fprintf('Ganancia K = %.4f\n', K_ident);
fprintf('Constante de tiempo τ = %.4f\n', tau_ident);
fprintf('Coeficiente de amortiguamiento ζ = %.4f\n', zeta_ident);

%% 5. VALIDACIÓN DEL MODELO IDENTIFICADO
fprintf('\n=== VALIDACIÓN DEL MODELO ===\n');

% Simular modelo identificado (discreto)
y_ident = zeros(1, N);
y_ident(1:2) = y_medida(1:2); % Mismas condiciones iniciales

for k = 3:N
    y_ident(k) = -a1*y_ident(k-1) - a2*y_ident(k-2) + b1*u(k-1);
end

% Métricas de validación
SSE = sum((y_medida - y_ident).^2);
SST = sum((y_medida - mean(y_medida)).^2);
R2 = 1 - SSE/SST;

RMSE = sqrt(mean((y_medida - y_ident).^2));
fit_percent = (1 - norm(y_medida - y_ident)/norm(y_medida - mean(y_medida))) * 100;

fprintf('Métricas de validación:\n');
fprintf('R² = %.4f (1.0 es perfecto)\n', R2);
fprintf('RMSE = %.4f\n', RMSE);
fprintf('Fit = %.2f%%\n', fit_percent);

%% 6. COMPARACIÓN MODELO REAL VS IDENTIFICADO
fprintf('\n=== COMPARACIÓN MODELO REAL VS IDENTIFICADO ===\n');

fprintf('\nComparación de parámetros:\n');
fprintf('┌─────────────────┬──────────┬─────────────┬──────────┐\n');
fprintf('│ Parámetro       │ Real     │ Identificado│ Error %%  │\n');
fprintf('├─────────────────┼──────────┼─────────────┼──────────┤\n');
fprintf('│ Ganancia (K)    │ %8.3f │ %11.3f │ %7.2f%% │\n', ...
        K_real, K_ident, abs(K_real-K_ident)/K_real*100);
fprintf('│ Tau (τ)         │ %8.3f │ %11.3f │ %7.2f%% │\n', ...
        tau_real, tau_ident, abs(tau_real-tau_ident)/tau_real*100);
fprintf('│ Zeta (ζ)        │ %8.3f │ %11.3f │ %7.2f%% │\n', ...
        zeta_real, zeta_ident, abs(zeta_real-zeta_ident)/zeta_real*100);
fprintf('└─────────────────┴──────────┴─────────────┴──────────┘\n');

%% 7. GRÁFICOS DE COMPARACIÓN
figure('Position', [100, 100, 1400, 800]);

% Respuesta temporal
subplot(2,2,1);
plot(t, y_medida, 'r.', 'MarkerSize', 4, 'DisplayName', 'Datos medidos'); hold on;
plot(t, y_ident, 'b-', 'LineWidth', 2, 'DisplayName', 'Modelo identificado');
plot(t, y_real, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Sistema real');
title('Validación Temporal');
xlabel('Tiempo [s]'); ylabel('Amplitud');
legend('Location', 'best'); grid on;

% Error de modelado
subplot(2,2,2);
error = y_medida - y_ident;
plot(t, error, 'm', 'LineWidth', 1.5);
title('Error de Modelado');
xlabel('Tiempo [s]'); ylabel('Error');
grid on;
fprintf('Error máximo: %.4f\n', max(abs(error)));

% Respuesta al escalón (simulada)
subplot(2,2,3);
% Simular respuesta al escalón del sistema real
u_escalon = 3 * ones(1, N);
y_escalon_real = zeros(1, N);
y_escalon_real(1:2) = 0;
for k = 3:N
    y_escalon_real(k) = ( -coef_yk1*y_escalon_real(k-1) - coef_yk2*y_escalon_real(k-2) + ...
                         (K_real/tau_real^2)*u_escalon(k) ) / coef_yk;
end

% Simular respuesta al escalón del modelo identificado
y_escalon_ident = zeros(1, N);
y_escalon_ident(1:2) = 0;
for k = 3:N
    y_escalon_ident(k) = -a1*y_escalon_ident(k-1) - a2*y_escalon_ident(k-2) + b1*u_escalon(k-1);
end

plot(t, y_escalon_real, 'g-', 'LineWidth', 2, 'DisplayName', 'Real'); hold on;
plot(t, y_escalon_ident, 'b--', 'LineWidth', 2, 'DisplayName', 'Identificado');
title('Respuesta al Escalón');
xlabel('Tiempo [s]'); ylabel('Amplitud');
legend; grid on;

% % Análisis de residuos
% subplot(2,3,4);
% histogram(error, 30);
% title('Distribución del Error');
% xlabel('Error'); ylabel('Frecuencia');
% 
% % Autocorrelación de residuos
% subplot(2,3,5);
% [acf_res, lags_acf] = xcorr(error - mean(error), 30, 'coeff');
% plot(lags_acf*dt, acf_res, 'k', 'LineWidth', 2);
% title('Autocorrelación del Error');
% xlabel('Retardo [s]'); ylabel('Autocorrelación');
% grid on;
% 
% % Predicción vs medición
% subplot(2,3,6);
% plot(y_medida, y_ident, 'bo', 'MarkerSize', 4);
% hold on;
% plot([min(y_medida), max(y_medida)], [min(y_medida), max(y_medida)], 'r--', 'LineWidth', 2);
% title('Predicción vs Medición');
% xlabel('Mediciones'); ylabel('Predicciones');
% grid on;
% axis equal;

%% 8. PREDICCIÓN A FUTURO
fprintf('\n=== PREDICCIÓN A FUTURO ===\n');

% Nueva señal de prueba
t_pred = 0:dt:30;
N_pred = length(t_pred);
u_pred = 2 + sin(2*pi*0.15*t_pred) + 0.5*cos(2*pi*0.4*t_pred);

% Simular sistema real
y_pred_real = zeros(1, N_pred);
y_pred_real(1:2) = 0;
for k = 3:N_pred
    y_pred_real(k) = ( -coef_yk1*y_pred_real(k-1) - coef_yk2*y_pred_real(k-2) + ...
                      (K_real/tau_real^2)*u_pred(k) ) / coef_yk;
end

% Simular con modelo identificado
y_pred_ident = zeros(1, N_pred);
y_pred_ident(1:2) = 0;
for k = 3:N_pred
    y_pred_ident(k) = -a1*y_pred_ident(k-1) - a2*y_pred_ident(k-2) + b1*u_pred(k-1);
end

% Agregar ruido a las "mediciones"
y_pred_medida = y_pred_real + 0.05*std(y_pred_real)*randn(1, N_pred);

% figure('Position', [200, 200, 1000, 600]);
% plot(t_pred, y_pred_medida, 'r.', 'MarkerSize', 6, 'DisplayName', 'Mediciones futuras'); hold on;
% plot(t_pred, y_pred_real, 'g-', 'LineWidth', 2, 'DisplayName', 'Sistema real');
% plot(t_pred, y_pred_ident, 'b--', 'LineWidth', 2, 'DisplayName', 'Modelo identificado');
% title('Predicción con Nueva Señal');
% xlabel('Tiempo [s]'); ylabel('Amplitud');
% legend; grid on;

% Calcular error de predicción
error_pred = mean(abs(y_pred_medida - y_pred_ident));
fprintf('Error de predicción promedio: %.4f\n', error_pred);

%% 9. ANÁLISIS DE SENSIBILIDAD
% fprintf('\n=== ANÁLISIS DE SENSIBILIDAD ===\n');

% Probar diferentes órdenes de modelo
ordenes = [1, 2, 3, 4];
errores = zeros(1, length(ordenes));

for i = 1:length(ordenes)
    na_test = ordenes(i);
    nb_test = ordenes(i);
    
    % Identificar modelo
    Phi_test = zeros(N_data, na_test + nb_test);
    Y_test = zeros(N_data, 1);
    
    idx = 1;
    for k = max(na_test, nb_test+nk-1)+1:N
        for ii = 1:na_test
            Phi_test(idx, ii) = -y_medida(k-ii);
        end
        for jj = 1:nb_test
            Phi_test(idx, na_test + jj) = u(k - (jj + nk - 1));
        end
        Y_test(idx) = y_medida(k);
        idx = idx + 1;
    end
    
    theta_test = Phi_test \ Y_test;
    
    % Simular
    y_test = zeros(1, N);
    y_test(1:na_test) = y_medida(1:na_test);
    
    for k = na_test+1:N
        y_test(k) = 0;
        for ii = 1:na_test
            y_test(k) = y_test(k) - theta_test(ii)*y_test(k-ii);
        end
        for jj = 1:nb_test
            y_test(k) = y_test(k) + theta_test(na_test + jj)*u(k - (jj + nk - 1));
        end
    end
    
    errores(i) = sqrt(mean((y_medida - y_test).^2));
end

% fprintf('Análisis de orden del modelo:\n');
for i = 1:length(ordenes)
    % fprintf('Orden %d: RMSE = %.4f\n', ordenes(i), errores(i));
end

%% RESUMEN FINAL
fprintf('\n=== RESUMEN DE IDENTIFICACIÓN ===\n');
fprintf('Proceso completado exitosamente:\n');
fprintf('✓ Sistema real simulado: G(s) = %.2f/(%.2fs² + %.2fs + 1)\n', K_real, tau_real^2, 2*zeta_real*tau_real);
fprintf('✓ Datos generados: %d puntos con ruido\n', N);
fprintf('✓ Modelo identificado: G(z) = %.4f/(z² + %.4f z + %.4f)\n', b1, a1, a2);
fprintf('✓ Validación: R² = %.4f, Fit = %.1f%%\n', R2, fit_percent);
fprintf('✓ Error de predicción: %.4f\n', error_pred);
fprintf('✓ Mejor orden del modelo: %d\n', ordenes(find(errores == min(errores), 1)));