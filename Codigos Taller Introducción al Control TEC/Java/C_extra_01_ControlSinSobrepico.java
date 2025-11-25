import java.awt.*;
import java.util.*;
import javax.swing.*;

public class C_extra_01_ControlSinSobrepico {
    // Parámetros del sistema
    private static final double T_REF = 22.0;
    private static final double T_AMB = 15.0;
    private static final double T_INICIAL = 15.0;
    private static final double K = 1.0;
    private static final double TAU = 1.0;
    private static final double ZETA_NATURAL = 0.5;
    
    // Límites del actuador
    private static final double U_MAX = 50.0;
    private static final double U_MIN = 0.0;
    
    // Tiempo de simulación
    private static final double T_SIM = 40.0;
    private static final double DT = 0.01;
    
    // Clase para almacenar configuraciones
    static class ConfiguracionPID {
        String nombre;
        Color color;
        double Kp, Ki, Kd;
        
        public ConfiguracionPID(String nombre, Color color, double Kp, double Ki, double Kd) {
            this.nombre = nombre;
            this.color = color;
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
    }
    
    // Configuraciones para respuesta sin sobrepico
    private static final ConfiguracionPID[] CONFIGS = {
        new ConfiguracionPID("Muy Suave", Color.BLUE, 2.0, 0.8, 8.0),
        new ConfiguracionPID("Balanceada", Color.GREEN, 3.5, 1.5, 6.0),
        new ConfiguracionPID("Rápida Sin Sobrepico", Color.RED, 5.0, 2.0, 4.0)
    };
    
    // Variables de simulación
    private static double[] tiempo;
    private static ArrayList<ResultadoSimulacion> resultados;
    
    public static void main(String[] args) {
        System.out.println("=== CONTROL SIN SOBREPICO - RESPUESTA GRADUAL ===\n");
        
        System.out.println("Configuraciones para respuesta sin sobrepico:");
        for (ConfiguracionPID config : CONFIGS) {
            System.out.printf("%s: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
                             config.nombre, config.Kp, config.Ki, config.Kd);
        }
        
        // Ejecutar simulación
        simularSistemas();
        
        // Mostrar resultados
        mostrarResultados();
        
        // Mostrar guía
        mostrarGuia();
        
        // Mostrar gráficos
        SwingUtilities.invokeLater(() -> mostrarGraficos());
    }
    
    private static void simularSistemas() {
        int N = (int)(T_SIM / DT) + 1;
        tiempo = new double[N];
        resultados = new ArrayList<>();
        
        // Inicializar tiempo
        for (int i = 0; i < N; i++) {
            tiempo[i] = i * DT;
        }
        
        // Coeficientes de discretización
        double coef_Tk = 1/(DT*DT) + (2*ZETA_NATURAL)/(TAU*DT) + 1/(TAU*TAU);
        double coef_Tk1 = -2/(DT*DT) - (2*ZETA_NATURAL)/(TAU*DT);
        double coef_Tk2 = 1/(DT*DT);
        
        // Simular cada configuración
        for (ConfiguracionPID config : CONFIGS) {
            double Kp = config.Kp;
            double Ki = config.Ki;
            double Kd = config.Kd;
            String nombre = config.nombre;
            Color color = config.color;
            
            // Arrays de simulación
            double[] T_pid = new double[N];
            double[] u_pid = new double[N];
            double[] error_pid = new double[N];
            
            // Condiciones iniciales
            T_pid[0] = T_INICIAL;
            T_pid[1] = T_INICIAL;
            
            double integral = 0.0;
            double error_prev = 0.0;
            
            // Simulación del control PID
            for (int k = 2; k < N; k++) {
                error_pid[k] = T_REF - T_pid[k-1];
                
                // Término Proporcional
                double P_term = Kp * error_pid[k];
                
                // Término Integral
                integral += (error_pid[k] + error_prev) * DT / 2;
                double I_term = Ki * integral;
                
                // Término Derivativo
                double D_term = Kd * (error_pid[k] - error_prev) / DT;
                
                // Señal de control total
                u_pid[k] = P_term + I_term + D_term;
                u_pid[k] = Math.max(Math.min(u_pid[k], U_MAX), U_MIN);
                
                // Anti-windup
                if (u_pid[k] >= U_MAX || u_pid[k] <= U_MIN) {
                    integral -= (error_pid[k] + error_prev) * DT / 2;
                }
                
                error_prev = error_pid[k];
                
                // Simulación del sistema
                T_pid[k] = (-coef_Tk1 * T_pid[k-1] - coef_Tk2 * T_pid[k-2] + 
                           (K/(TAU*TAU)) * u_pid[k] + (1/(TAU*TAU)) * T_AMB) / coef_Tk;
            }
            
            // Calcular métricas
            double maxTemp = Arrays.stream(T_pid).max().orElse(T_REF);
            double sobrespico = Math.max(0, maxTemp - T_REF);
            double t_settle = calcularTiempoEstablecimiento(T_pid, tiempo);
            double error_ss = Math.abs(T_pid[N-1] - T_REF);
            
            // Almacenar resultados
            resultados.add(new ResultadoSimulacion(nombre, color, T_pid, u_pid, error_pid,
                                                  sobrespico, t_settle, error_ss));
            
            // Mostrar resultados en consola
            System.out.printf("\n%s:\n", nombre);
            System.out.printf("  Sobrepico: %.2f°C\n", sobrespico);
            System.out.printf("  Tiempo establecimiento: %.1f s\n", t_settle);
            System.out.printf("  Error estacionario: %.3f°C\n", error_ss);
            
            if (sobrespico < 0.1) {
                System.out.println("  ✅ SIN OSCILACIONES");
            } else {
                System.out.println("  ❌ CON OSCILACIONES");
            }
        }
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
    
    private static void mostrarResultados() {
        System.out.println("\n=== RESUMEN DE RESULTADOS ===");
        System.out.println("┌──────────────────────┬────────────┬──────────────────┬────────────────┐");
        System.out.println("│ Configuración        │ Sobrepico  │ Tiempo Estab.    │ Error Estac.   │");
        System.out.println("├──────────────────────┼────────────┼──────────────────┼────────────────┤");
        
        for (ResultadoSimulacion resultado : resultados) {
            System.out.printf("│ %-20s │ %8.2f°C  │ %12.1f s   │ %10.3f°C   │\n",
                            resultado.nombre, resultado.sobrespico, 
                            resultado.tiempoEstablecimiento, resultado.errorEstacionario);
        }
        System.out.println("└──────────────────────┴────────────┴──────────────────┴────────────────┘");
    }
    
    private static void mostrarGuia() {
        System.out.println("\n=== GUÍA PARA ELIMINAR SOBREPICO ===");
        System.out.println("Para ELIMINAR OSCILACIONES y tener respuesta GRADUAL:");
        System.out.println("1. REDUCIR Kp (ganancia proporcional)");
        System.out.println("   - Menos Kp → respuesta más lenta pero más estable");
        System.out.println("   - Valores típicos: 1.5 - 4.0\n");
        
        System.out.println("2. AUMENTAR Kd (ganancia derivativa)");
        System.out.println("   - Más Kd → amortigua oscilaciones");
        System.out.println("   - Valores típicos: 4.0 - 10.0\n");
        
        System.out.println("3. AJUSTAR Ki (ganancia integral)");
        System.out.println("   - Ki muy alto puede causar oscilaciones");
        System.out.println("   - Valores típicos: 0.5 - 2.0\n");
        
        System.out.println("CONFIGURACIÓN RECOMENDADA:");
        System.out.println("Kp = 2.0 - 3.5, Ki = 0.8 - 1.5, Kd = 6.0 - 8.0\n");
        
        System.out.println("=== COMPARACIÓN CON RESPUESTA OSCILATORIA ===");
        System.out.println("Si tuvieras una configuración oscilatoria (ej: Kp=10, Kd=1):");
        System.out.println("- Reducir Kp en 60-80%");
        System.out.println("- Aumentar Kd en 400-800%");
        System.out.println("- Reducir Ki en 50-70%");
    }
    
    private static void mostrarGraficos() {
        JFrame frame = new JFrame("Control Sin Sobrepico - Respuesta Gradual");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.addTab("Comparación", new PanelComparacion());
        tabbedPane.addTab("Análisis Detallado", new PanelAnalisisDetallado());
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    // Clase para almacenar resultados de simulación
    static class ResultadoSimulacion {
        String nombre;
        Color color;
        double[] temperatura;
        double[] control;
        double[] error;
        double sobrespico;
        double tiempoEstablecimiento;
        double errorEstacionario;
        
        public ResultadoSimulacion(String nombre, Color color, double[] temp, 
                                  double[] ctrl, double[] err, double sobresp, 
                                  double tEst, double errSS) {
            this.nombre = nombre;
            this.color = color;
            this.temperatura = temp;
            this.control = ctrl;
            this.error = err;
            this.sobrespico = sobresp;
            this.tiempoEstablecimiento = tEst;
            this.errorEstacionario = errSS;
        }
    }
    
    // Panel de comparación principal
    static class PanelComparacion extends JPanel {
        public PanelComparacion() {
            setBackground(Color.WHITE);
            setLayout(new GridLayout(2, 2));
            
            add(new GraficoTemperatura());
            add(new GraficoControl());
            add(new GraficoError());
            add(new GraficoZoom());
        }
    }
    
    static class GraficoTemperatura extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 60;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Respuesta de Temperatura - Sin Sobrepico", width/2 - 120, 25);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Temperatura [°C]", 10, height/2);
            
            // Dibujar referencia
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{5}, 0));
            int yRef = (int)(height - padding - ((T_REF - 14) / 11.0) * (height - 2 * padding));
            g2.drawLine(padding, yRef, width - padding, yRef);
            
            // Dibujar curvas de temperatura
            for (ResultadoSimulacion resultado : resultados) {
                g2.setColor(resultado.color);
                g2.setStroke(new BasicStroke(2));
                
                for (int i = 1; i < tiempo.length; i++) {
                    int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                    int y1 = (int)(height - padding - ((resultado.temperatura[i-1] - 14) / 11.0) * (height - 2 * padding));
                    int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                    int y2 = (int)(height - padding - ((resultado.temperatura[i] - 14) / 11.0) * (height - 2 * padding));
                    g2.drawLine(x1, y1, x2, y2);
                }
            }
            
            // Leyenda
            int legendX = width - 180;
            int legendY = padding + 20;
            for (ResultadoSimulacion resultado : resultados) {
                g2.setColor(resultado.color);
                g2.drawString(resultado.nombre, legendX, legendY);
                legendY += 20;
            }
            g2.setColor(Color.BLACK);
            g2.drawString("Referencia", legendX, legendY);
        }
    }
    
    static class GraficoControl extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 60;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Señal de Control", width/2 - 40, 25);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Potencia [W]", 10, height/2);
            
            // Dibujar curvas de control
            for (ResultadoSimulacion resultado : resultados) {
                g2.setColor(resultado.color);
                g2.setStroke(new BasicStroke(2));
                
                for (int i = 1; i < tiempo.length; i++) {
                    int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                    int y1 = (int)(height - padding - (resultado.control[i-1] / U_MAX) * (height - 2 * padding));
                    int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                    int y2 = (int)(height - padding - (resultado.control[i] / U_MAX) * (height - 2 * padding));
                    g2.drawLine(x1, y1, x2, y2);
                }
            }
        }
    }
    
    static class GraficoError extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 60;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Error de Temperatura", width/2 - 60, 25);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Error [°C]", 10, height/2);
            
            // Encontrar máximo error para escalado
            double maxError = 0;
            for (ResultadoSimulacion resultado : resultados) {
                double configMaxError = Arrays.stream(resultado.error).map(Math::abs).max().orElse(0.0);
                maxError = Math.max(maxError, configMaxError);
            }
            
            // Dibujar curvas de error
            for (ResultadoSimulacion resultado : resultados) {
                g2.setColor(resultado.color);
                g2.setStroke(new BasicStroke(2));
                
                for (int i = 1; i < tiempo.length; i++) {
                    int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                    int y1 = (int)(height - padding - ((resultado.error[i-1] + maxError) / (2 * maxError)) * (height - 2 * padding));
                    int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                    int y2 = (int)(height - padding - ((resultado.error[i] + maxError) / (2 * maxError)) * (height - 2 * padding));
                    g2.drawLine(x1, y1, x2, y2);
                }
            }
            
            // Línea de error cero
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{3}, 0));
            int zeroY = (int)(height - padding - (height - 2 * padding) / 2);
            g2.drawLine(padding, zeroY, width - padding, zeroY);
        }
    }
    
    static class GraficoZoom extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 60;
            
            // Zoom en los primeros 15 segundos
            double t_zoom = 15.0;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Respuesta Inicial (Zoom - Primeros " + (int)t_zoom + "s)", width/2 - 100, 25);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 20);
            g2.drawString("Temperatura [°C]", 10, height/2);
            
            // Dibujar referencia
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{5}, 0));
            int yRef = (int)(height - padding - ((T_REF - 15) / 8.0) * (height - 2 * padding));
            g2.drawLine(padding, yRef, width - padding, yRef);
            
            // Dibujar curvas de temperatura (zoom)
            for (ResultadoSimulacion resultado : resultados) {
                g2.setColor(resultado.color);
                g2.setStroke(new BasicStroke(3));
                
                for (int i = 1; i < tiempo.length; i++) {
                    if (tiempo[i] <= t_zoom) {
                        int x1 = (int)(padding + (tiempo[i-1] / t_zoom) * (width - 2 * padding));
                        int y1 = (int)(height - padding - ((resultado.temperatura[i-1] - 15) / 8.0) * (height - 2 * padding));
                        int x2 = (int)(padding + (tiempo[i] / t_zoom) * (width - 2 * padding));
                        int y2 = (int)(height - padding - ((resultado.temperatura[i] - 15) / 8.0) * (height - 2 * padding));
                        g2.drawLine(x1, y1, x2, y2);
                    }
                }
            }
        }
    }
    
    // Panel de análisis detallado
    static class PanelAnalisisDetallado extends JPanel {
        public PanelAnalisisDetallado() {
            setBackground(Color.WHITE);
            setLayout(new BorderLayout());
            
            JTextArea textArea = new JTextArea();
            textArea.setEditable(false);
            textArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
            
            StringBuilder sb = new StringBuilder();
            sb.append("=== ANÁLISIS DETALLADO - CONTROL SIN SOBREPICO ===\n\n");
            
            sb.append("OBJETIVO: Respuesta gradual sin oscilaciones\n");
            sb.append("BENEFICIOS:\n");
            sb.append("  • Sin sobrepicos que puedan dañar el sistema\n");
            sb.append("  • Menor estrés mecánico en actuadores\n");
            sb.append("  • Comportamiento predecible y estable\n");
            sb.append("  • Ideal para procesos críticos\n\n");
            
            sb.append("RESULTADOS POR CONFIGURACIÓN:\n\n");
            
            for (ResultadoSimulacion resultado : resultados) {
                // Buscar la configuración correspondiente
                ConfiguracionPID configCorrespondiente = null;
                for (ConfiguracionPID config : CONFIGS) {
                    if (config.nombre.equals(resultado.nombre)) {
                        configCorrespondiente = config;
                        break;
                    }
                }
                
                if (configCorrespondiente != null) {
                    sb.append(String.format("%s:\n", resultado.nombre));
                    sb.append(String.format("  PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
                        configCorrespondiente.Kp, configCorrespondiente.Ki, configCorrespondiente.Kd));
                    sb.append(String.format("  Sobrepico: %.2f°C\n", resultado.sobrespico));
                    sb.append(String.format("  Tiempo establecimiento: %.1f s\n", resultado.tiempoEstablecimiento));
                    sb.append(String.format("  Error estacionario: %.3f°C\n", resultado.errorEstacionario));
                    
                    if (resultado.sobrespico < 0.1) {
                        sb.append("  ESTADO: ✅ SIN OSCILACIONES\n");
                    } else {
                        sb.append("  ESTADO: ❌ CON OSCILACIONES\n");
                    }
                    sb.append("\n");
                }
            }
            
            sb.append("RECOMENDACIONES:\n");
            sb.append("1. Configuración 'Balanceada' para la mayoría de aplicaciones\n");
            sb.append("2. Configuración 'Muy Suave' para procesos muy sensibles\n");
            sb.append("3. Configuración 'Rápida Sin Sobrepico' cuando se necesita respuesta más rápida\n");
            sb.append("   pero manteniendo estabilidad\n");
            
            textArea.setText(sb.toString());
            add(new JScrollPane(textArea), BorderLayout.CENTER);
        }
    }
}