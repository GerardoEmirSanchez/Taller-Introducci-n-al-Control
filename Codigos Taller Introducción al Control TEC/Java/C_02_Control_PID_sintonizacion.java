import java.awt.*;
import java.util.*;
import javax.swing.*;

public class C_02_Control_PID_sintonizacion {
    // Parámetros del sistema
    private static final double T_REF = 22.0;
    private static final double T_AMB = 15.0;
    private static final double T_INICIAL = 15.0;
    
    // Parámetros del modelo
    private static final double K = 1.0;
    private static final double TAU = 1.0;
    private static final double ZETA_NATURAL = 0.5;
    
    // Límites del actuador
    private static final double U_MAX = 50.0;
    private static final double U_MIN = 0.0;
    
    // Tiempo de simulación
    private static final double T_SIM = 50.0;
    private static final double DT = 0.01;
    
    // Configuraciones deseadas
    private static final String[] CONFIG_NOMBRES = {"Subamortiguado", "Crit. Amortig.", "Sobreamortig."};
    private static final double[] CONFIG_ZETA = {0.3, 1.0, 1.1};
    private static final double[] CONFIG_WN = {0.8, 0.6, 0.4};
    private static final Color[] COLORES = {Color.RED, Color.GREEN, Color.BLUE};
    
    public static void main(String[] args) {
        System.out.println("=== MÉTODO ANALÍTICO DE SINTONIZACIÓN PID ===\n");
        System.out.printf("Sistema: G(s) = %.1f/(%.1fs² + %.1fs + 1)\n", K, TAU*TAU, 2*ZETA_NATURAL*TAU);
        
        // Polos del sistema original
        double[] polosOriginales = calcularPolosOriginales();
        System.out.println("\nPolos del sistema original:");
        System.out.printf("Polo 1: %.3f + %.3fi\n", polosOriginales[0], polosOriginales[1]);
        System.out.printf("Polo 2: %.3f + %.3fi\n", polosOriginales[2], polosOriginales[3]);
        
        // Cálculo analítico de constantes PID
        System.out.println("\n=== CÁLCULO ANALÍTICO POR UBICACIÓN DE POLOS ===");
        
        double[][] valoresPID = new double[3][3]; // [Kp, Ki, Kd] para cada configuración
        
        for (int i = 0; i < 3; i++) {
            String nombre = CONFIG_NOMBRES[i];
            double zetaD = CONFIG_ZETA[i];
            double wnD = CONFIG_WN[i];
            
            System.out.printf("\n--- %s (ζ=%.1f, ωn=%.1f) ---\n", nombre, zetaD, wnD);
            
            // Calcular polos deseados
            double[][] polosDeseados = calcularPolosDeseados(zetaD, wnD);
            System.out.printf("Polos deseados: %.3f + %.3fi, %.3f + %.3fi\n", 
                    polosDeseados[0][0], polosDeseados[0][1],
                    polosDeseados[1][0], polosDeseados[1][1]);
            
            // Calcular constantes PID
            double[] pid = calcularConstantesPID(zetaD, wnD, polosDeseados);
            valoresPID[i] = pid;
            
            System.out.println("Constantes PID calculadas:");
            System.out.printf("  Kp = %.3f\n", pid[0]);
            System.out.printf("  Ki = %.3f\n", pid[1]);
            System.out.printf("  Kd = %.3f\n", pid[2]);
        }
        
        // Verificación con simulación
        System.out.println("\n=== VERIFICACIÓN CON SIMULACIÓN ===");
        ResultadoSimulacion[] resultados = simularSistemas(valoresPID);
        
        // Mostrar resultados
        mostrarResultados(resultados, valoresPID);
        
        // Análisis de polos en lazo cerrado
        System.out.println("\n=== ANÁLISIS DE POLOS EN LAZO CERRADO ===");
        analizarPolosLazoCerrado(valoresPID);
        
        // Mostrar gráficos
        mostrarGraficos(resultados);
    }
    
    private static double[] calcularPolosOriginales() {
        // Polos de: τ²s² + 2ζτs + 1 = 0
        double discriminante = Math.pow(2*ZETA_NATURAL*TAU, 2) - 4*TAU*TAU;
        
        if (discriminante >= 0) {
            // Polos reales
            double polo1 = (-2*ZETA_NATURAL*TAU + Math.sqrt(discriminante)) / (2*TAU*TAU);
            double polo2 = (-2*ZETA_NATURAL*TAU - Math.sqrt(discriminante)) / (2*TAU*TAU);
            return new double[]{polo1, 0, polo2, 0};
        } else {
            // Polos complejos
            double real = -2*ZETA_NATURAL*TAU / (2*TAU*TAU);
            double imag = Math.sqrt(-discriminante) / (2*TAU*TAU);
            return new double[]{real, imag, real, -imag};
        }
    }
    
    private static double[][] calcularPolosDeseados(double zeta, double wn) {
        double[][] polos = new double[2][2]; // [real, imag] para cada polo
        
        if (zeta < 1) {
            // Subamortiguado - polos complejos conjugados
            polos[0][0] = -zeta * wn;
            polos[0][1] = wn * Math.sqrt(1 - zeta*zeta);
            polos[1][0] = polos[0][0];
            polos[1][1] = -polos[0][1];
        } else if (zeta == 1) {
            // Críticamente amortiguado
            polos[0][0] = -wn;
            polos[0][1] = 0;
            polos[1][0] = -wn;
            polos[1][1] = 0;
        } else {
            // Sobreamortiguado
            polos[0][0] = -zeta * wn + wn * Math.sqrt(zeta*zeta - 1);
            polos[0][1] = 0;
            polos[1][0] = -zeta * wn - wn * Math.sqrt(zeta*zeta - 1);
            polos[1][1] = 0;
        }
        
        return polos;
    }
    
    private static double[] calcularConstantesPID(double zetaD, double wnD, double[][] polosDeseados) {
        double a2, a1, a0;
        
        if (zetaD < 1) {
            // Para subamortiguado
            double poloExtra = 5 * Math.max(Math.abs(polosDeseados[0][0]), Math.abs(polosDeseados[1][0]));
            a2 = 2*zetaD*wnD + poloExtra;
            a1 = wnD*wnD + 2*zetaD*wnD * poloExtra;
            a0 = wnD*wnD * poloExtra;
        } else if (zetaD == 1) {
            // Para críticamente amortiguado
            double poloExtra = Math.max(Math.abs(polosDeseados[0][0]), Math.abs(polosDeseados[1][0]));
            a2 = 2*zetaD*wnD + poloExtra;
            a1 = wnD*wnD + 2*zetaD*wnD * poloExtra;
            a0 = wnD*wnD * poloExtra;
        } else {
            // Para sobreamortiguado
            double poloExtra = 2 * Math.min(Math.abs(polosDeseados[0][0]), Math.abs(polosDeseados[1][0]));
            a2 = Math.abs(polosDeseados[0][0] + polosDeseados[1][0] + poloExtra);
            a1 = Math.abs(polosDeseados[0][0]*polosDeseados[1][0] + 
                         (polosDeseados[0][0] + polosDeseados[1][0])*poloExtra);
            a0 = Math.abs(polosDeseados[0][0]*polosDeseados[1][0]*poloExtra);
        }
        
        // Resolver sistema: τ²s³ + (2ζτ + KKd)s² + (1 + KKp)s + KKi = s³ + a₂s² + a₁s + a₀
        double Kd_calc = (a2 * TAU*TAU - 2*ZETA_NATURAL*TAU) / K;
        double Kp_calc = (a1 * TAU*TAU - 1) / K;
        double Ki_calc = (a0 * TAU*TAU) / K;
        
        // Asegurar valores positivos
        Kp_calc = Math.max(Kp_calc, 0.1);
        Ki_calc = Math.max(Ki_calc, 0.01);
        Kd_calc = Math.max(Kd_calc, 0.01);
        
        return new double[]{Kp_calc, Ki_calc, Kd_calc};
    }
    
    private static ResultadoSimulacion[] simularSistemas(double[][] valoresPID) {
        int N = (int)(T_SIM / DT) + 1;
        double[] t = new double[N];
        for (int i = 0; i < N; i++) {
            t[i] = i * DT;
        }
        
        // Coeficientes de discretización
        double coef_Tk = 1/(DT*DT) + (2*ZETA_NATURAL)/(TAU*DT) + 1/(TAU*TAU);
        double coef_Tk1 = -2/(DT*DT) - (2*ZETA_NATURAL)/(TAU*DT);
        double coef_Tk2 = 1/(DT*DT);
        
        ResultadoSimulacion[] resultados = new ResultadoSimulacion[3];
        
        for (int i = 0; i < 3; i++) {
            double Kp = valoresPID[i][0];
            double Ki = valoresPID[i][1];
            double Kd = valoresPID[i][2];
            
            double[] T_pid = new double[N];
            double[] u_pid = new double[N];
            double[] error_pid = new double[N];
            
            Arrays.fill(T_pid, T_INICIAL);
            T_pid[1] = T_INICIAL;
            
            double integral = 0.0;
            double error_prev = 0.0;
            double derivativo = 0.0;
            double T_prev = T_INICIAL;
            
            for (int k = 2; k < N; k++) {
                double error_actual = T_REF - T_pid[k-1];
                error_pid[k] = error_actual;
                
                // Término Proporcional
                double P_term = Kp * error_actual;
                
                // Término Integral
                integral += error_actual * DT;
                double I_term = Ki * integral;
                
                // Término Derivativo (de la salida)
                derivativo = 0.8 * derivativo + 0.2 * ((T_pid[k-1] - T_prev) / DT);
                double D_term = -Kd * derivativo;
                
                u_pid[k] = P_term + I_term + D_term;
                u_pid[k] = Math.max(Math.min(u_pid[k], U_MAX), U_MIN);
                
                // Anti-windup
                if (u_pid[k] >= U_MAX || u_pid[k] <= U_MIN) {
                    integral -= error_actual * DT;
                }
                
                T_prev = T_pid[k-1];
                
                // Simulación del sistema
                T_pid[k] = (-coef_Tk1 * T_pid[k-1] - coef_Tk2 * T_pid[k-2] + 
                           (K/(TAU*TAU)) * u_pid[k] + (1/(TAU*TAU)) * T_AMB) / coef_Tk;
            }
            
            // Calcular métricas
            double sobrespico = Math.max(0, Arrays.stream(T_pid).max().orElse(T_REF) - T_REF);
            double t_settle = calcularTiempoEstablecimiento(T_pid, t);
            double error_ss = Math.abs(T_pid[N-1] - T_REF);
            double ISE = calcularISE(error_pid, DT);
            double zeta_obs = calcularZetaObservado(T_pid);
            
            resultados[i] = new ResultadoSimulacion(T_pid, u_pid, error_pid, 
                                                   sobrespico, t_settle, error_ss, ISE, zeta_obs);
        }
        
        return resultados;
    }
    
    private static double calcularTiempoEstablecimiento(double[] T, double[] t) {
        double tolerance = 0.02 * T_REF;
        for (int i = T.length - 1; i >= 0; i--) {
            if (Math.abs(T[i] - T_REF) > tolerance) {
                return t[Math.min(i + 1, t.length - 1)];
            }
        }
        return t[0];
    }
    
    private static double calcularISE(double[] error, double dt) {
        double ise = 0.0;
        for (double e : error) {
            ise += e * e * dt;
        }
        return ise;
    }
    
    private static double calcularZetaObservado(double[] T) {
        // Encuentra el máximo (sobrepico)
        double max = Arrays.stream(T).max().orElse(T_REF);
        
        if (max <= T_REF) {
            // No hay sobrepico
            return 1.0;
        }
        
        double Mp = (max - T_REF) / T_REF;
        
        if (Mp <= 0.001) {
            return 1.0;
        }
        
        // Fórmula: Mp = exp(-ζπ/√(1-ζ²))
        if (Mp >= 1) {
            return 0.01;
        } else {
            double logMp = Math.log(Mp);
            return Math.sqrt(logMp * logMp / (Math.PI * Math.PI + logMp * logMp));
        }
    }
    
    private static void mostrarResultados(ResultadoSimulacion[] resultados, double[][] valoresPID) {
        for (int i = 0; i < 3; i++) {
            System.out.printf("%s:\n", CONFIG_NOMBRES[i]);
            System.out.printf("  PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", 
                    valoresPID[i][0], valoresPID[i][1], valoresPID[i][2]);
            System.out.printf("  ζ deseado=%.1f, ζ observado=%.3f\n", CONFIG_ZETA[i], resultados[i].zetaObservado);
            System.out.printf("  Sobrepico: %.2f%%, t_est=%.2fs\n\n", 
                    (resultados[i].sobrespico/T_REF)*100, resultados[i].tiempoEstablecimiento);
        }
    }
    
    private static void analizarPolosLazoCerrado(double[][] valoresPID) {
        for (int i = 0; i < 3; i++) {
            double Kp = valoresPID[i][0];
            double Ki = valoresPID[i][1];
            double Kd = valoresPID[i][2];
            
            // Polinomio característico: τ²s³ + (2ζτ + KKd)s² + (1 + KKp)s + KKi
            // Usamos una aproximación para encontrar raíces
            System.out.printf("\n%s:\n", CONFIG_NOMBRES[i]);
            System.out.println("Polos en lazo cerrado (aproximados):");
            
            // Para sistemas de tercer orden, mostramos los coeficientes
            System.out.printf("  Coeficientes: [%.3f, %.3f, %.3f, %.3f]\n", 
                    TAU*TAU, (2*ZETA_NATURAL*TAU + K*Kd), (1 + K*Kp), K*Ki);
        }
    }
    
    private static void mostrarGraficos(ResultadoSimulacion[] resultados) {
        JFrame frame = new JFrame("Control de Temperatura - Método Analítico");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1400, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        
        // Panel de temperaturas
        tabbedPane.addTab("Temperaturas", new GraficoTemperaturas(resultados));
        
        // Panel de errores
        tabbedPane.addTab("Errores", new GraficoErrores(resultados));
        
        // Panel de control
        tabbedPane.addTab("Señal de Control", new GraficoControl(resultados));
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    // Clases para gráficos
    static class GraficoTemperaturas extends JPanel {
        private ResultadoSimulacion[] resultados;
        
        public GraficoTemperaturas(ResultadoSimulacion[] resultados) {
            this.resultados = resultados;
            setBackground(Color.WHITE);
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 60;
            int graphWidth = width - 2 * padding;
            int graphHeight = height - 2 * padding;
            
            // Dibujar curvas
            for (int i = 0; i < resultados.length; i++) {
                dibujarCurva(g2, resultados[i].temperatura, COLORES[i], padding, graphWidth, graphHeight);
            }
            
            // Dibujar referencia
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{5}, 0));
            int yRef = height - padding - (int)((T_REF - 10) / 20.0 * graphHeight);
            g2.drawLine(padding, yRef, width - padding, yRef);
            
            // Leyenda
            dibujarLeyenda(g2, width, height, padding);
            
            // Títulos
            g2.setColor(Color.BLACK);
            g2.drawString("Respuesta de Temperatura - Método Analítico", width / 2 - 100, padding / 2);
            g2.drawString("Tiempo [s]", width / 2 - 30, height - padding / 3);
            g2.drawString("Temperatura [C]", padding / 4, height / 2);
        }
        
        private void dibujarCurva(Graphics2D g2, double[] y, Color color, int padding, int graphWidth, int graphHeight) {
            g2.setColor(color);
            g2.setStroke(new BasicStroke(2));
            
            int N = y.length;
            double[] t = new double[N];
            for (int i = 0; i < N; i++) {
                t[i] = i * DT;
            }
            
            for (int i = 1; i < N; i++) {
                int x1 = (int)(padding + (t[i-1] / T_SIM) * graphWidth);
                int y1 = (int)(getHeight() - padding - ((y[i-1] - 10) / 20.0) * graphHeight);
                int x2 = (int)(padding + (t[i] / T_SIM) * graphWidth);
                int y2 = (int)(getHeight() - padding - ((y[i] - 10) / 20.0) * graphHeight);
                g2.drawLine(x1, y1, x2, y2);
            }
        }
        
        private void dibujarLeyenda(Graphics2D g2, int width, int height, int padding) {
            int legendX = width - 200;
            int legendY = padding + 20;
            
            for (int i = 0; i < resultados.length; i++) {
                g2.setColor(COLORES[i]);
                g2.drawString(CONFIG_NOMBRES[i], legendX, legendY + i * 20);
            }
            g2.setColor(Color.BLACK);
            g2.drawString("Referencia", legendX, legendY + resultados.length * 20);
        }
    }
    
    static class GraficoErrores extends JPanel {
        private ResultadoSimulacion[] resultados;
        
        public GraficoErrores(ResultadoSimulacion[] resultados) {
            this.resultados = resultados;
            setBackground(Color.WHITE);
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            // Implementación similar a GraficoTemperaturas pero para errores
        }
    }
    
    static class GraficoControl extends JPanel {
        private ResultadoSimulacion[] resultados;
        
        public GraficoControl(ResultadoSimulacion[] resultados) {
            this.resultados = resultados;
            setBackground(Color.WHITE);
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            // Implementación similar a GraficoTemperaturas pero para señal de control
        }
    }
    
    // Clase para almacenar resultados de simulación
    static class ResultadoSimulacion {
        double[] temperatura;
        double[] control;
        double[] error;
        double sobrespico;
        double tiempoEstablecimiento;
        double errorEstadoEstacionario;
        double ISE;
        double zetaObservado;
        
        public ResultadoSimulacion(double[] temp, double[] ctrl, double[] err, 
                                  double sobresp, double tEst, double errSS, double ise, double zetaObs) {
            this.temperatura = temp;
            this.control = ctrl;
            this.error = err;
            this.sobrespico = sobresp;
            this.tiempoEstablecimiento = tEst;
            this.errorEstadoEstacionario = errSS;
            this.ISE = ise;
            this.zetaObservado = zetaObs;
        }
    }
}