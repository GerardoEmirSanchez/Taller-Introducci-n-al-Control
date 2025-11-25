import java.awt.*;
import java.util.*;
import javax.swing.*;

public class C_03_Control_Identificar_modelo {
    // Parámetros del sistema REAL (desconocido en práctica real)
    private static final double K_REAL = 1.2;
    private static final double TAU_REAL = 1.5;
    private static final double ZETA_REAL = 0.6;
    
    // Parámetros de simulación
    private static final double T_SIM = 50.0;
    private static final double DT = 0.01;
    private static final double T_AMB = 0.0; // Temperatura ambiente
    
    public static void main(String[] args) {
        System.out.println("=== IDENTIFICACIÓN DE SISTEMAS + CONTROL PID - MÉTODO ANALÍTICO ===\n");
        
        System.out.println("=== SISTEMA REAL (DESCONOCIDO) ===");
        System.out.printf("G(s) = %.2f/(%.2fs² + %.2fs + 1)\n", 
                         K_REAL, TAU_REAL*TAU_REAL, 2*ZETA_REAL*TAU_REAL);
        
        // 1. GENERACIÓN DE DATOS
        DatosSimulacion datos = generarDatosSimulacion();
        
        // 2. IDENTIFICACIÓN POR MÍNIMOS CUADRADOS
        ModeloIdentificado modelo = identificarSistema(datos);
        
        // 3. VALIDACIÓN DEL MODELO
        validarModelo(datos, modelo);
        
        // 4. DISEÑO DE CONTROL PID
        diseñarControladoresPID(modelo);
        
        // 5. MOSTRAR RESULTADOS COMPLETOS
        mostrarResultadosCompletos(datos, modelo);
    }
    
    // Clase para almacenar datos de simulación
    static class DatosSimulacion {
        double[] tiempo;
        double[] entrada;
        double[] salidaReal;
        double[] salidaMedida;
        
        public DatosSimulacion(double[] t, double[] u, double[] yReal, double[] yMedida) {
            this.tiempo = t;
            this.entrada = u;
            this.salidaReal = yReal;
            this.salidaMedida = yMedida;
        }
    }
    
    // Clase para modelo identificado
    static class ModeloIdentificado {
        double[] tiempo;
        double[] salidaIdentificada;
        double a1, a2, b1;
        double K, tau, zeta;
        double R2, RMSE, fitPercent;
        
        public ModeloIdentificado(double[] t, double[] yIdent, double a1, double a2, double b1,
                                 double K, double tau, double zeta, double R2, double RMSE, double fit) {
            this.tiempo = t;
            this.salidaIdentificada = yIdent;
            this.a1 = a1;
            this.a2 = a2;
            this.b1 = b1;
            this.K = K;
            this.tau = tau;
            this.zeta = zeta;
            this.R2 = R2;
            this.RMSE = RMSE;
            this.fitPercent = fit;
        }
    }
    
    // 1. GENERACIÓN DE DATOS
    private static DatosSimulacion generarDatosSimulacion() {
        System.out.println("\n=== GENERANDO DATOS DE ENTRADA/SALIDA ===");
        
        int N = (int)(T_SIM / DT) + 1;
        double[] t = new double[N];
        double[] u = new double[N];
        double[] yReal = new double[N];
        double[] yMedida = new double[N];
        
        // Inicializar tiempo
        for (int i = 0; i < N; i++) {
            t[i] = i * DT;
        }
        
        // Generar señal de excitación rica en frecuencias
        Arrays.fill(u, 3.0); // Escalón base
        
        // Agregar componentes de diferentes frecuencias
        double[] freqs = {0.05, 0.1, 0.2, 0.5};
        double[] amplitudes = {1.0, 0.8, 0.5, 0.3};
        
        for (int i = 0; i < freqs.length; i++) {
            for (int j = 0; j < N; j++) {
                u[j] += amplitudes[i] * Math.sin(2 * Math.PI * freqs[i] * t[j]);
            }
        }
        
        // Agregar ruido blanco pequeño
        Random rand = new Random();
        for (int i = 0; i < N; i++) {
            u[i] += 0.2 * rand.nextGaussian();
            u[i] = Math.max(Math.min(u[i], 6), 0); // Limitar señal
        }
        
        // Simular sistema REAL (segundo orden)
        // Coeficientes de discretización
        double coef_yk = 1/(DT*DT) + (2*ZETA_REAL)/(TAU_REAL*DT) + 1/(TAU_REAL*TAU_REAL);
        double coef_yk1 = -2/(DT*DT) - (2*ZETA_REAL)/(TAU_REAL*DT);
        double coef_yk2 = 1/(DT*DT);
        
        // Condiciones iniciales
        yReal[0] = 0;
        yReal[1] = 0;
        
        for (int k = 2; k < N; k++) {
            yReal[k] = (-coef_yk1 * yReal[k-1] - coef_yk2 * yReal[k-2] + 
                       (K_REAL/(TAU_REAL*TAU_REAL)) * u[k]) / coef_yk;
        }
        
        // Agregar ruido de medición
        double stdReal = calcularDesviacionEstandar(yReal);
        for (int i = 0; i < N; i++) {
            yMedida[i] = yReal[i] + 0.05 * stdReal * rand.nextGaussian();
        }
        
        System.out.printf("Datos generados: %d puntos de muestreo\n", N);
        
        return new DatosSimulacion(t, u, yReal, yMedida);
    }
    
    // 2. IDENTIFICACIÓN POR MÍNIMOS CUADRADOS
    private static ModeloIdentificado identificarSistema(DatosSimulacion datos) {
        System.out.println("\n=== IDENTIFICACIÓN POR MÍNIMOS CUADRADOS ===");
        
        int N = datos.tiempo.length;
        double[] u = datos.entrada;
        double[] y = datos.salidaMedida;
        
        // Estructura del modelo (segundo orden)
        int na = 2;  // Número de polos
        int nb = 1;  // Número de ceros + 1
        int nk = 1;  // Retardo
        
        System.out.printf("Estructura del modelo: na=%d, nb=%d, nk=%d\n", na, nb, nk);
        
        // Preparar matriz de regresión
        int N_data = N - Math.max(na, nb + nk - 1);
        double[][] Phi = new double[N_data][na + nb];
        double[] Y = new double[N_data];
        
        int idx = 0;
        for (int k = Math.max(na, nb + nk - 1); k < N; k++) {
            // Términos AR (salidas pasadas)
            for (int i = 0; i < na; i++) {
                Phi[idx][i] = -y[k - (i + 1)];
            }
            
            // Términos X (entradas pasadas)
            for (int j = 0; j < nb; j++) {
                Phi[idx][na + j] = u[k - (j + nk)];
            }
            
            Y[idx] = y[k];
            idx++;
        }
        
        // Estimación por mínimos cuadrados
        double[] theta = resolverMinimosCuadrados(Phi, Y);
        
        // Extraer parámetros
        double a1 = theta[0];
        double a2 = theta[1];
        double b1 = theta[2];
        
        System.out.println("Parámetros identificados:");
        System.out.printf("a1 = %.4f, a2 = %.4f, b1 = %.4f\n", a1, a2, b1);
        System.out.printf("\nModelo discreto identificado:\n");
        System.out.printf("G(z) = (%.4f) / (z² + %.4f z + %.4f)\n", b1, a1, a2);
        
        // Conversión a tiempo continuo (aproximación)
        System.out.println("\n=== CONVERSIÓN A TIEMPO CONTINUO ===");
        
        double T = DT;
        
        // Aproximación por transformación bilineal
        double den0 = 1 + a1 + a2;
        double den1 = 2 * (1 - a2) / T;
        double den2 = 4 * (1 - a1 + a2) / (T * T);
        
        double num0 = b1;
        double num1 = 2 * b1 / T;
        double num2 = 4 * b1 / (T * T);
        
        // Normalizar
        double K_ident = num0 / den0;
        double tau_ident = Math.sqrt(den2 / den0);
        double zeta_ident = den1 / (2 * tau_ident * den0);
        
        System.out.println("Modelo continuo aproximado:");
        System.out.printf("G(s) = (%.4f s² + %.4f s + %.4f) / (s² + %.4f s + %.4f)\n", 
                         num2/den2, num1/den2, num0/den2, den1/den2, den0/den2);
        
        System.out.println("\nParámetros del sistema identificado:");
        System.out.printf("Ganancia K = %.4f\n", K_ident);
        System.out.printf("Constante de tiempo τ = %.4f\n", tau_ident);
        System.out.printf("Coeficiente de amortiguamiento ζ = %.4f\n", zeta_ident);
        
        // Simular modelo identificado para validación
        double[] yIdent = new double[N];
        yIdent[0] = y[0];
        yIdent[1] = y[1];
        
        for (int k = 2; k < N; k++) {
            yIdent[k] = -a1 * yIdent[k-1] - a2 * yIdent[k-2] + b1 * u[k-1];
        }
        
        // Calcular métricas
        double[] metricas = calcularMetricasValidacion(y, yIdent);
        
        return new ModeloIdentificado(datos.tiempo, yIdent, a1, a2, b1, 
                                    K_ident, tau_ident, zeta_ident,
                                    metricas[0], metricas[1], metricas[2]);
    }
    
    // 3. VALIDACIÓN DEL MODELO
    private static void validarModelo(DatosSimulacion datos, ModeloIdentificado modelo) {
        System.out.println("\n=== VALIDACIÓN DEL MODELO ===");
        
        System.out.println("Métricas de validación:");
        System.out.printf("R² = %.4f (1.0 es perfecto)\n", modelo.R2);
        System.out.printf("RMSE = %.4f\n", modelo.RMSE);
        System.out.printf("Fit = %.2f%%\n", modelo.fitPercent);
        
        System.out.println("\n=== COMPARACIÓN MODELO REAL VS IDENTIFICADO ===");
        System.out.println("\nComparación de parámetros:");
        System.out.println("┌─────────────────┬──────────┬─────────────┬──────────┐");
        System.out.println("│ Parámetro       │ Real     │ Identificado│ Error %  │");
        System.out.println("├─────────────────┼──────────┼─────────────┼──────────┤");
        System.out.printf("│ Ganancia (K)    │ %8.3f │ %11.3f │ %7.2f%% │\n",
                K_REAL, modelo.K, Math.abs(K_REAL - modelo.K)/K_REAL*100);
        System.out.printf("│ Tau (τ)         │ %8.3f │ %11.3f │ %7.2f%% │\n",
                TAU_REAL, modelo.tau, Math.abs(TAU_REAL - modelo.tau)/TAU_REAL*100);
        System.out.printf("│ Zeta (ζ)        │ %8.3f │ %11.3f │ %7.2f%% │\n",
                ZETA_REAL, modelo.zeta, Math.abs(ZETA_REAL - modelo.zeta)/ZETA_REAL*100);
        System.out.println("└─────────────────┴──────────┴─────────────┴──────────┘");
    }
    
    // 4. DISEÑO DE CONTROL PID
    private static void diseñarControladoresPID(ModeloIdentificado modelo) {
        System.out.println("\n=== DISEÑO DE CONTROLADORES PID ===");
        
        // Configuraciones deseadas
        String[] configNombres = {"Subamortiguado", "Crit. Amortig.", "Sobreamortig."};
        double[] configZeta = {0.7, 1.0, 1.3};
        double[] configWn = {0.8, 0.6, 0.4};
        
        System.out.println("\nConfiguraciones de diseño:");
        for (int i = 0; i < configNombres.length; i++) {
            System.out.printf("%s: ζ=%.1f, ωn=%.1f\n", configNombres[i], configZeta[i], configWn[i]);
        }
        
        // Diseñar controladores para cada configuración
        System.out.println("\n=== CONSTANTES PID CALCULADAS ===");
        for (int i = 0; i < configNombres.length; i++) {
            double[] pid = calcularConstantesPID(modelo, configZeta[i], configWn[i]);
            System.out.printf("\n%s:\n", configNombres[i]);
            System.out.printf("  Kp = %.3f\n", pid[0]);
            System.out.printf("  Ki = %.3f\n", pid[1]);
            System.out.printf("  Kd = %.3f\n", pid[2]);
        }
    }
    
    // 5. MOSTRAR RESULTADOS COMPLETOS
    private static void mostrarResultadosCompletos(DatosSimulacion datos, ModeloIdentificado modelo) {
        System.out.println("\n=== RESUMEN DE IDENTIFICACIÓN ===");
        System.out.println("Proceso completado exitosamente:");
        System.out.printf("✓ Sistema real simulado: G(s) = %.2f/(%.2fs² + %.2fs + 1)\n",
                         K_REAL, TAU_REAL*TAU_REAL, 2*ZETA_REAL*TAU_REAL);
        System.out.printf("✓ Datos generados: %d puntos con ruido\n", datos.tiempo.length);
        System.out.printf("✓ Modelo identificado: G(z) = %.4f/(z² + %.4f z + %.4f)\n",
                         modelo.b1, modelo.a1, modelo.a2);
        System.out.printf("✓ Validación: R² = %.4f, Fit = %.1f%%\n", modelo.R2, modelo.fitPercent);
        System.out.printf("✓ Parámetros continuos: K=%.3f, τ=%.3f, ζ=%.3f\n",
                         modelo.K, modelo.tau, modelo.zeta);
        
        // Mostrar gráficos
        mostrarGraficosCompletos(datos, modelo);
    }
    
    // MÉTODOS AUXILIARES
    private static double[] resolverMinimosCuadrados(double[][] A, double[] b) {
        int m = A.length;
        int n = A[0].length;
        
        // A^T * A
        double[][] AtA = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < m; k++) {
                    AtA[i][j] += A[k][i] * A[k][j];
                }
            }
        }
        
        // A^T * b
        double[] Atb = new double[n];
        for (int i = 0; i < n; i++) {
            for (int k = 0; k < m; k++) {
                Atb[i] += A[k][i] * b[k];
            }
        }
        
        // Resolver sistema (AtA)x = Atb
        return resolverSistemaLineal(AtA, Atb);
    }
    
    private static double[] resolverSistemaLineal(double[][] A, double[] b) {
        int n = b.length;
        double[] x = new double[n];
        
        // Eliminación gaussiana simple
        for (int i = 0; i < n; i++) {
            // Pivote
            double maxEl = Math.abs(A[i][i]);
            int maxRow = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(A[k][i]) > maxEl) {
                    maxEl = Math.abs(A[k][i]);
                    maxRow = k;
                }
            }
            
            // Intercambiar filas
            if (maxRow != i) {
                double[] tempA = A[i];
                A[i] = A[maxRow];
                A[maxRow] = tempA;
                double tempB = b[i];
                b[i] = b[maxRow];
                b[maxRow] = tempB;
            }
            
            // Hacer triangular superior
            for (int k = i + 1; k < n; k++) {
                double factor = A[k][i] / A[i][i];
                for (int j = i; j < n; j++) {
                    A[k][j] -= factor * A[i][j];
                }
                b[k] -= factor * b[i];
            }
        }
        
        // Sustitución hacia atrás
        for (int i = n - 1; i >= 0; i--) {
            x[i] = b[i];
            for (int j = i + 1; j < n; j++) {
                x[i] -= A[i][j] * x[j];
            }
            x[i] /= A[i][i];
        }
        
        return x;
    }
    
    private static double[] calcularMetricasValidacion(double[] yReal, double[] yIdent) {
        int n = yReal.length;
        
        // Calcular SSE y SST
        double SSE = 0;
        double SST = 0;
        double meanY = calcularMedia(yReal);
        
        for (int i = 0; i < n; i++) {
            SSE += Math.pow(yReal[i] - yIdent[i], 2);
            SST += Math.pow(yReal[i] - meanY, 2);
        }
        
        double R2 = 1 - SSE / SST;
        double RMSE = Math.sqrt(SSE / n);
        double fitPercent = (1 - calcularNorma(yReal, yIdent) / calcularNorma(yReal, meanY)) * 100;
        
        return new double[]{R2, RMSE, fitPercent};
    }
    
    private static double[] calcularConstantesPID(ModeloIdentificado modelo, double zetaD, double wnD) {
        double K = modelo.K;
        double tau = modelo.tau;
        double zeta = modelo.zeta;
        
        // Cálculo simplificado de constantes PID
        double Kp = (2 * zetaD * wnD * tau - 1) / K;
        double Ki = (wnD * wnD * tau * tau) / K;
        double Kd = 0.0; // Para simplificar
        
        // Asegurar valores positivos
        Kp = Math.max(Kp, 0.1);
        Ki = Math.max(Ki, 0.01);
        
        return new double[]{Kp, Ki, Kd};
    }
    
    private static double calcularDesviacionEstandar(double[] data) {
        double mean = calcularMedia(data);
        double sum = 0;
        for (double value : data) {
            sum += Math.pow(value - mean, 2);
        }
        return Math.sqrt(sum / data.length);
    }
    
    private static double calcularMedia(double[] data) {
        double sum = 0;
        for (double value : data) {
            sum += value;
        }
        return sum / data.length;
    }
    
    private static double calcularNorma(double[] a, double[] b) {
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            sum += Math.pow(a[i] - b[i], 2);
        }
        return Math.sqrt(sum);
    }
    
    private static double calcularNorma(double[] a, double b) {
        double sum = 0;
        for (double value : a) {
            sum += Math.pow(value - b, 2);
        }
        return Math.sqrt(sum);
    }
    
    private static void mostrarGraficosCompletos(DatosSimulacion datos, ModeloIdentificado modelo) {
        JFrame frame = new JFrame("Identificación de Sistema + Control PID");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1400, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.addTab("Identificación", new PanelIdentificacion(datos, modelo));
        tabbedPane.addTab("Validación", new PanelValidacion(datos, modelo));
        tabbedPane.addTab("Comparación", new PanelComparacion(datos, modelo));
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    // Paneles para gráficos - CORREGIDOS PARA DIBUJAR LAS CURVAS
    static class PanelIdentificacion extends JPanel {
        private DatosSimulacion datos;
        private ModeloIdentificado modelo;
        
        public PanelIdentificacion(DatosSimulacion datos, ModeloIdentificado modelo) {
            this.datos = datos;
            this.modelo = modelo;
            setBackground(Color.WHITE);
            setPreferredSize(new Dimension(800, 600));
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            dibujarGraficoIdentificacion(g2);
        }
        
        private void dibujarGraficoIdentificacion(Graphics2D g2) {
            int width = getWidth();
            int height = getHeight();
            int padding = 80;
            int graphWidth = width - 2 * padding;
            int graphHeight = height - 2 * padding;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding); // Eje X
            g2.drawLine(padding, height - padding, padding, padding); // Eje Y
            
            // Títulos
            g2.drawString("Identificación del Sistema - Datos y Modelo", width/2 - 100, 30);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Amplitud", 20, height/2);
            
            // Encontrar límites de los datos
            double minY = Double.MAX_VALUE;
            double maxY = -Double.MAX_VALUE;
            
            for (double value : datos.salidaMedida) {
                if (value < minY) minY = value;
                if (value > maxY) maxY = value;
            }
            for (double value : modelo.salidaIdentificada) {
                if (value < minY) minY = value;
                if (value > maxY) maxY = value;
            }
            
            // Dibujar datos medidos (puntos rojos)
            g2.setColor(Color.RED);
            for (int i = 0; i < datos.tiempo.length; i += 10) { // Muestrear cada 10 puntos para mejor rendimiento
                int x = (int)(padding + (datos.tiempo[i] / T_SIM) * graphWidth);
                int y = (int)(height - padding - ((datos.salidaMedida[i] - minY) / (maxY - minY)) * graphHeight);
                g2.fillOval(x-1, y-1, 3, 3);
            }
            
            // Dibujar modelo identificado (línea azul)
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(2));
            for (int i = 1; i < modelo.tiempo.length; i++) {
                int x1 = (int)(padding + (modelo.tiempo[i-1] / T_SIM) * graphWidth);
                int y1 = (int)(height - padding - ((modelo.salidaIdentificada[i-1] - minY) / (maxY - minY)) * graphHeight);
                int x2 = (int)(padding + (modelo.tiempo[i] / T_SIM) * graphWidth);
                int y2 = (int)(height - padding - ((modelo.salidaIdentificada[i] - minY) / (maxY - minY)) * graphHeight);
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Dibujar sistema real (línea verde discontinua)
            g2.setColor(Color.GREEN);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{5}, 0));
            for (int i = 1; i < datos.tiempo.length; i++) {
                int x1 = (int)(padding + (datos.tiempo[i-1] / T_SIM) * graphWidth);
                int y1 = (int)(height - padding - ((datos.salidaReal[i-1] - minY) / (maxY - minY)) * graphHeight);
                int x2 = (int)(padding + (datos.tiempo[i] / T_SIM) * graphWidth);
                int y2 = (int)(height - padding - ((datos.salidaReal[i] - minY) / (maxY - minY)) * graphHeight);
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Leyenda
            int legendX = width - 200;
            int legendY = padding + 20;
            
            g2.setColor(Color.RED);
            g2.drawString("Datos Medidos", legendX, legendY);
            g2.setColor(Color.BLUE);
            g2.drawString("Modelo Identificado", legendX, legendY + 20);
            g2.setColor(Color.GREEN);
            g2.drawString("Sistema Real", legendX, legendY + 40);
        }
    }
    
    static class PanelValidacion extends JPanel {
        private DatosSimulacion datos;
        private ModeloIdentificado modelo;
        
        public PanelValidacion(DatosSimulacion datos, ModeloIdentificado modelo) {
            this.datos = datos;
            this.modelo = modelo;
            setBackground(Color.WHITE);
            setPreferredSize(new Dimension(800, 600));
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            dibujarGraficoValidacion(g2);
        }
        
        private void dibujarGraficoValidacion(Graphics2D g2) {
            int width = getWidth();
            int height = getHeight();
            int padding = 80;
            int graphWidth = width - 2 * padding;
            int graphHeight = height - 2 * padding;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Validación del Modelo Identificado", width/2 - 80, 30);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Error", 20, height/2);
            
            // Calcular error
            double[] error = new double[datos.salidaMedida.length];
            for (int i = 0; i < error.length; i++) {
                error[i] = datos.salidaMedida[i] - modelo.salidaIdentificada[i];
            }
            
            // Encontrar límites del error
            double minError = Arrays.stream(error).min().orElse(0);
            double maxError = Arrays.stream(error).max().orElse(0);
            double maxAbsError = Math.max(Math.abs(minError), Math.abs(maxError));
            
            // Dibujar error
            g2.setColor(Color.MAGENTA);
            g2.setStroke(new BasicStroke(2));
            for (int i = 1; i < datos.tiempo.length; i++) {
                int x1 = (int)(padding + (datos.tiempo[i-1] / T_SIM) * graphWidth);
                int y1 = (int)(height - padding - ((error[i-1] + maxAbsError) / (2 * maxAbsError)) * graphHeight);
                int x2 = (int)(padding + (datos.tiempo[i] / T_SIM) * graphWidth);
                int y2 = (int)(height - padding - ((error[i] + maxAbsError) / (2 * maxAbsError)) * graphHeight);
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Línea de error cero
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{3}, 0));
            int zeroY = (int)(height - padding - graphHeight/2);
            g2.drawLine(padding, zeroY, width - padding, zeroY);
            
            // Información de métricas
            g2.setColor(Color.BLACK);
            g2.drawString(String.format("R² = %.4f", modelo.R2), width - 150, 50);
            g2.drawString(String.format("RMSE = %.4f", modelo.RMSE), width - 150, 70);
            g2.drawString(String.format("Fit = %.1f%%", modelo.fitPercent), width - 150, 90);
            g2.drawString(String.format("Error máximo: %.4f", maxAbsError), width - 150, 110);
        }
    }
    
    static class PanelComparacion extends JPanel {
        private DatosSimulacion datos;
        private ModeloIdentificado modelo;
        
        public PanelComparacion(DatosSimulacion datos, ModeloIdentificado modelo) {
            this.datos = datos;
            this.modelo = modelo;
            setBackground(Color.WHITE);
            setPreferredSize(new Dimension(800, 600));
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            dibujarGraficoComparacion(g2);
        }
        
        private void dibujarGraficoComparacion(Graphics2D g2) {
            int width = getWidth();
            int height = getHeight();
            int padding = 80;
            
            // Títulos
            g2.setColor(Color.BLACK);
            g2.drawString("Comparación de Parámetros del Sistema", width/2 - 100, 30);
            
            // Información de comparación
            int y = 80;
            g2.drawString("PARÁMETRO            REAL       IDENTIFICADO   ERROR", 50, y);
            y += 30;
            g2.drawString(String.format("Ganancia (K)       %6.3f      %6.3f       %5.2f%%", 
                K_REAL, modelo.K, Math.abs(K_REAL - modelo.K)/K_REAL*100), 50, y);
            y += 25;
            g2.drawString(String.format("Tau (τ)            %6.3f      %6.3f       %5.2f%%", 
                TAU_REAL, modelo.tau, Math.abs(TAU_REAL - modelo.tau)/TAU_REAL*100), 50, y);
            y += 25;
            g2.drawString(String.format("Zeta (ζ)           %6.3f      %6.3f       %5.2f%%", 
                ZETA_REAL, modelo.zeta, Math.abs(ZETA_REAL - modelo.zeta)/ZETA_REAL*100), 50, y);
            
            // Información del modelo
            y += 50;
            g2.drawString("MODELO IDENTIFICADO:", 50, y);
            y += 25;
            g2.drawString(String.format("Discreto:  G(z) = %.4f / (z² + %.4f z + %.4f)", 
                modelo.b1, modelo.a1, modelo.a2), 70, y);
            y += 20;
            g2.drawString(String.format("Continuo:  G(s) = %.4f / (%.4f s² + %.4f s + 1)", 
                modelo.K, modelo.tau*modelo.tau, 2*modelo.zeta*modelo.tau), 70, y);
            
            // Métricas de calidad
            y += 50;
            g2.drawString("MÉTRICAS DE VALIDACIÓN:", 50, y);
            y += 25;
            g2.drawString(String.format("R² = %.4f", modelo.R2), 70, y);
            y += 20;
            g2.drawString(String.format("RMSE = %.4f", modelo.RMSE), 70, y);
            y += 20;
            g2.drawString(String.format("Fit = %.1f%%", modelo.fitPercent), 70, y);
        }
    }
}