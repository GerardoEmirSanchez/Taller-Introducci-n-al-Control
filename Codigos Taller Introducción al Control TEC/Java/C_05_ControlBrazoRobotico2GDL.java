import java.awt.*;
import java.util.*;
import javax.swing.*;
import java.awt.geom.*;
import javax.swing.Timer;

public class C_05_ControlBrazoRobotico2GDL {
    // Parámetros del brazo robótico
    private static final double L1 = 1.0;           // Longitud del primer eslabón [m]
    private static final double L2 = 0.8;           // Longitud del segundo eslabón [m]
    private static final double M1 = 2.0;           // Masa del primer eslabón [kg]
    private static final double M2 = 1.5;           // Masa del segundo eslabón [kg]
    private static final double G = 9.81;           // Gravedad [m/s²]
    
    // Posiciones deseadas (ángulos en radianes)
    private static final double THETA1_REF = Math.PI/2;  // 90° - Articulación 1
    private static final double THETA2_REF = Math.PI/4;  // 45° - Articulación 2
    
    // Posiciones iniciales
    private static final double THETA1_INITIAL = 0;      // 0° - Articulación 1
    private static final double THETA2_INITIAL = 0;      // 0° - Articulación 2
    
    // Parámetros PID
    private static final double KP = 30.0;
    private static final double KI = 0.5;
    private static final double KD = 15.0;
    
    // Límites de torque
    private static final double TAU_MAX = 100.0;    // Torque máximo [Nm]
    private static final double TAU_MIN = -100.0;   // Torque mínimo [Nm]
    
    // Tiempo de simulación
    private static final double T_SIM = 10.0;
    private static final double DT = 0.01;
    
    // Variables de simulación
    private static double[] tiempo;
    private static double[] theta1, theta2;
    private static double[] omega1, omega2;
    private static double[] tau1, tau2;
    private static double[] error1, error2;
    private static double[] x1, y1, x2, y2;
    
    public static void main(String[] args) {
        System.out.println("=== CONTROL DE BRAZO ROBÓTICO - 2 GRADOS DE LIBERTAD CON CONTROL PID ===\n");
        
        System.out.println("=== PARÁMETROS DEL BRAZO ===");
        System.out.printf("Longitud eslabón 1: %.1f m\n", L1);
        System.out.printf("Longitud eslabón 2: %.1f m\n", L2);
        System.out.printf("Posición deseada: θ1=%.1f°, θ2=%.1f°\n", 
                         Math.toDegrees(THETA1_REF), Math.toDegrees(THETA2_REF));
        
        System.out.println("\n=== PARÁMETROS PID ===");
        System.out.printf("ARTICULACIÓN 1: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", KP, KI, KD);
        System.out.printf("ARTICULACIÓN 2: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", KP, KI, KD);
        
        // Ejecutar simulación
        simularSistema();
        
        // Mostrar resultados
        mostrarResultados();
        
        // Iniciar animación
        SwingUtilities.invokeLater(() -> iniciarAnimacion());
    }
    
    private static void simularSistema() {
        int N = (int)(T_SIM / DT) + 1;
        
        // Inicializar arrays
        tiempo = new double[N];
        theta1 = new double[N]; theta2 = new double[N];
        omega1 = new double[N]; omega2 = new double[N];
        tau1 = new double[N]; tau2 = new double[N];
        error1 = new double[N]; error2 = new double[N];
        x1 = new double[N]; y1 = new double[N];
        x2 = new double[N]; y2 = new double[N];
        
        // Inicializar tiempo
        for (int i = 0; i < N; i++) {
            tiempo[i] = i * DT;
        }
        
        // Condiciones iniciales
        theta1[0] = THETA1_INITIAL;
        theta2[0] = THETA2_INITIAL;
        omega1[0] = 0;
        omega2[0] = 0;
        
        // Momentos de inercia
        double I1 = M1 * L1 * L1 / 3.0;
        double I2 = M2 * L2 * L2 / 3.0;
        
        // Variables PID
        double integral1 = 0, integral2 = 0;
        double error1_prev = 0, error2_prev = 0;
        
        // Simulación del sistema
        for (int k = 0; k < N - 1; k++) {
            // Cálculo de errores
            error1[k] = THETA1_REF - theta1[k];
            error2[k] = THETA2_REF - theta2[k];
            
            // CONTROL PID ARTICULACIÓN 1
            double P1 = KP * error1[k];
            integral1 += error1[k] * DT;
            double I1_term = KI * integral1;
            double derivative1 = (error1[k] - error1_prev) / DT;
            double D1 = KD * derivative1;
            tau1[k] = P1 + I1_term + D1;
            
            // CONTROL PID ARTICULACIÓN 2
            double P2 = KP * error2[k];
            integral2 += error2[k] * DT;
            double I2_term = KI * integral2;
            double derivative2 = (error2[k] - error2_prev) / DT;
            double D2 = KD * derivative2;
            tau2[k] = P2 + I2_term + D2;
            
            // Saturación de torques
            tau1[k] = Math.max(Math.min(tau1[k], TAU_MAX), TAU_MIN);
            tau2[k] = Math.max(Math.min(tau2[k], TAU_MAX), TAU_MIN);
            
            // Anti-windup
            if (tau1[k] >= TAU_MAX || tau1[k] <= TAU_MIN) {
                integral1 -= error1[k] * DT;
            }
            if (tau2[k] >= TAU_MAX || tau2[k] <= TAU_MIN) {
                integral2 -= error2[k] * DT;
            }
            
            // DINÁMICA DEL BRAZO (modelo simplificado)
            // Aceleraciones angulares
            double alpha1 = (tau1[k] - M2 * L1 * L2 * Math.sin(theta2[k]) * omega2[k] * omega2[k] -
                          (M1 * G * L1 / 2 + M2 * G * L1) * Math.cos(theta1[k])) / I1;
            
            double alpha2 = (tau2[k] + M2 * L1 * L2 * Math.sin(theta2[k]) * omega1[k] * omega1[k] -
                          M2 * G * L2 / 2 * Math.cos(theta1[k] + theta2[k])) / I2;
            
            // Integración numérica (Euler)
            omega1[k + 1] = omega1[k] + alpha1 * DT;
            omega2[k + 1] = omega2[k] + alpha2 * DT;
            theta1[k + 1] = theta1[k] + omega1[k] * DT;
            theta2[k + 1] = theta2[k] + omega2[k] * DT;
            
            // Actualizar errores anteriores
            error1_prev = error1[k];
            error2_prev = error2[k];
            
            // Calcular posiciones
            x1[k] = L1 * Math.cos(theta1[k]);
            y1[k] = L1 * Math.sin(theta1[k]);
            x2[k] = x1[k] + L2 * Math.cos(theta1[k] + theta2[k]);
            y2[k] = y1[k] + L2 * Math.sin(theta1[k] + theta2[k]);
        }
        
        // Último cálculo
        error1[N-1] = THETA1_REF - theta1[N-1];
        error2[N-1] = THETA2_REF - theta2[N-1];
        x1[N-1] = L1 * Math.cos(theta1[N-1]);
        y1[N-1] = L1 * Math.sin(theta1[N-1]);
        x2[N-1] = x1[N-1] + L2 * Math.cos(theta1[N-1] + theta2[N-1]);
        y2[N-1] = y1[N-1] + L2 * Math.sin(theta1[N-1] + theta2[N-1]);
        
        System.out.println("Simulación completada exitosamente");
    }
    
    private static void mostrarResultados() {
        System.out.println("\n=== RESULTADOS DE LA SIMULACIÓN ===");
        System.out.println("CONFIGURACIÓN INICIAL:");
        System.out.printf("  θ₁: %.1f° → θ₂: %.1f°\n", 
                         Math.toDegrees(THETA1_INITIAL), Math.toDegrees(THETA2_INITIAL));
        System.out.println("CONFIGURACIÓN DESEADA:");
        System.out.printf("  θ₁: %.1f° → θ₂: %.1f°\n", 
                         Math.toDegrees(THETA1_REF), Math.toDegrees(THETA2_REF));
        
        double x_ref = L1 * Math.cos(THETA1_REF) + L2 * Math.cos(THETA1_REF + THETA2_REF);
        double y_ref = L1 * Math.sin(THETA1_REF) + L2 * Math.sin(THETA1_REF + THETA2_REF);
        System.out.printf("  Posición efector: (%.3f, %.3f) m\n", x_ref, y_ref);
        
        System.out.println("\nRESULTADOS FINALES:");
        System.out.printf("  θ₁ final: %.3f° (error: %.3f°)\n", 
                         Math.toDegrees(theta1[theta1.length-1]), Math.toDegrees(error1[error1.length-1]));
        System.out.printf("  θ₂ final: %.3f° (error: %.3f°)\n", 
                         Math.toDegrees(theta2[theta2.length-1]), Math.toDegrees(error2[error2.length-1]));
        System.out.printf("  Posición final: (%.3f, %.3f) m\n", 
                         x2[x2.length-1], y2[y2.length-1]);
        
        double pos_error = Math.sqrt(Math.pow(x_ref - x2[x2.length-1], 2) + 
                                   Math.pow(y_ref - y2[y2.length-1], 2));
        System.out.printf("  Error posición: %.4f m\n", pos_error);
        
        // Calcular torques máximos
        double maxTau1 = Arrays.stream(tau1).map(Math::abs).max().orElse(0.0);
        double maxTau2 = Arrays.stream(tau2).map(Math::abs).max().orElse(0.0);
        System.out.printf("\nTORQUES MÁXIMOS:\n");
        System.out.printf("  τ₁ max: %.1f Nm\n", maxTau1);
        System.out.printf("  τ₂ max: %.1f Nm\n", maxTau2);
    }
    
    private static void iniciarAnimacion() {
        System.out.println("\n=== INICIANDO ANIMACIÓN ===");
        
        JFrame frame = new JFrame("Brazo Robótico 2 GDL - Control PID");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1400, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.addTab("Animación", new PanelAnimacion());
        tabbedPane.addTab("Ángulos y Torques", new PanelAngulosTorques());
        tabbedPane.addTab("Análisis", new PanelAnalisis());
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    // Panel de animación principal
    static class PanelAnimacion extends JPanel {
        private Timer timer;
        private int currentIndex = 0;
        private ArrayList<Point2D.Double> trayectoria;
        
        public PanelAnimacion() {
            setBackground(Color.WHITE);
            trayectoria = new ArrayList<>();
            
            // Configurar timer para animación
            timer = new Timer(50, e -> {
                if (currentIndex < tiempo.length - 1) {
                    currentIndex += 2; // Acelerar animación
                    trayectoria.add(new Point2D.Double(x2[currentIndex], y2[currentIndex]));
                    repaint();
                } else {
                    timer.stop();
                }
            });
            timer.start();
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int centerX = width / 2;
            int centerY = height / 2;
            double scale = 150; // Escala para visualización
            
            // Dibujar área de trabajo
            g2.setColor(new Color(240, 240, 240));
            g2.fillRect(0, 0, width, height);
            
            // Dibujar ejes coordenados
            g2.setColor(Color.LIGHT_GRAY);
            g2.drawLine(0, centerY, width, centerY); // Eje X
            g2.drawLine(centerX, 0, centerX, height); // Eje Y
            
            // Posición deseada del efector final
            double x_ref = L1 * Math.cos(THETA1_REF) + L2 * Math.cos(THETA1_REF + THETA2_REF);
            double y_ref = L1 * Math.sin(THETA1_REF) + L2 * Math.sin(THETA1_REF + THETA2_REF);
            int targetX = centerX + (int)(x_ref * scale);
            int targetY = centerY - (int)(y_ref * scale);
            
            // Dibujar posición deseada
            g2.setColor(Color.RED);
            g2.fillOval(targetX - 5, targetY - 5, 10, 10);
            g2.drawString("Objetivo", targetX + 10, targetY);
            
            // Dibujar trayectoria
            g2.setColor(Color.MAGENTA);
            for (int i = 1; i < trayectoria.size(); i++) {
                Point2D.Double p1 = trayectoria.get(i-1);
                Point2D.Double p2 = trayectoria.get(i);
                int x1 = centerX + (int)(p1.x * scale);
                int y1 = centerY - (int)(p1.y * scale);
                int x2 = centerX + (int)(p2.x * scale);
                int y2 = centerY - (int)(p2.y * scale);
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Dibujar brazo robótico
            int shoulderX = centerX;
            int shoulderY = centerY;
            int elbowX = centerX + (int)(x1[currentIndex] * scale);
            int elbowY = centerY - (int)(y1[currentIndex] * scale);
            int wristX = centerX + (int)(x2[currentIndex] * scale);
            int wristY = centerY - (int)(y2[currentIndex] * scale);
            
            // Eslabón 1
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(8));
            g2.drawLine(shoulderX, shoulderY, elbowX, elbowY);
            
            // Eslabón 2
            g2.setColor(Color.GREEN);
            g2.drawLine(elbowX, elbowY, wristX, wristY);
            
            // Articulaciones
            g2.setColor(Color.BLACK);
            g2.fillOval(shoulderX - 6, shoulderY - 6, 12, 12); // Hombro
            g2.setColor(Color.BLUE);
            g2.fillOval(elbowX - 5, elbowY - 5, 10, 10); // Codo
            g2.setColor(Color.GREEN);
            g2.fillOval(wristX - 5, wristY - 5, 10, 10); // Muñeca
            
            // Información en tiempo real
            g2.setColor(Color.BLACK);
            g2.setFont(new Font("Arial", Font.BOLD, 12));
            String info = String.format(
                "Tiempo: %.1f s\n" +
                "θ₁: %.1f° (ref: %.1f°)\n" +
                "θ₂: %.1f° (ref: %.1f°)\n" +
                "Posición: (%.2f, %.2f) m\n" +
                "Error: %.3f m",
                tiempo[currentIndex],
                Math.toDegrees(theta1[currentIndex]), Math.toDegrees(THETA1_REF),
                Math.toDegrees(theta2[currentIndex]), Math.toDegrees(THETA2_REF),
                x2[currentIndex], y2[currentIndex],
                Math.sqrt(Math.pow(x_ref - x2[currentIndex], 2) + Math.pow(y_ref - y2[currentIndex], 2))
            );
            
            drawMultiLineString(g2, info, 20, 30);
            
            // Título
            g2.setFont(new Font("Arial", Font.BOLD, 16));
            g2.drawString("ANIMACIÓN BRAZO ROBÓTICO 2 GDL - CONTROL PID", width/2 - 180, 25);
            
            // Leyenda
            g2.setFont(new Font("Arial", Font.PLAIN, 12));
            g2.setColor(Color.BLUE);
            g2.drawString("Eslabón 1", width - 100, 30);
            g2.setColor(Color.GREEN);
            g2.drawString("Eslabón 2", width - 100, 50);
            g2.setColor(Color.MAGENTA);
            g2.drawString("Trayectoria", width - 100, 70);
            g2.setColor(Color.RED);
            g2.drawString("Objetivo", width - 100, 90);
        }
        
        private void drawMultiLineString(Graphics2D g2, String text, int x, int y) {
            String[] lines = text.split("\n");
            for (int i = 0; i < lines.length; i++) {
                g2.drawString(lines[i], x, y + i * 20);
            }
        }
    }
    
    // Panel para ángulos y torques
    static class PanelAngulosTorques extends JPanel {
        public PanelAngulosTorques() {
            setBackground(Color.WHITE);
            setLayout(new GridLayout(2, 1));
            
            add(new GraficoAngulos());
            add(new GraficoTorques());
        }
    }
    
    static class GraficoAngulos extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 50;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Ángulos de las Articulaciones", width/2 - 80, 20);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 10);
            g2.drawString("Ángulo [°]", 10, height/2);
            
            // Dibujar curvas de ángulos
            // θ1 - Azul
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(2));
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - (Math.toDegrees(theta1[i-1]) / 180.0) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - (Math.toDegrees(theta1[i]) / 180.0) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // θ2 - Rojo
            g2.setColor(Color.RED);
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - (Math.toDegrees(theta2[i-1]) / 180.0) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - (Math.toDegrees(theta2[i]) / 180.0) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Referencias
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{5}, 0));
            int yRef1 = (int)(height - padding - (Math.toDegrees(THETA1_REF) / 180.0) * (height - 2 * padding));
            g2.drawLine(padding, yRef1, width - padding, yRef1);
            
            g2.setColor(Color.RED);
            int yRef2 = (int)(height - padding - (Math.toDegrees(THETA2_REF) / 180.0) * (height - 2 * padding));
            g2.drawLine(padding, yRef2, width - padding, yRef2);
            
            // Leyenda
            g2.setColor(Color.BLUE);
            g2.drawString("θ₁", width - 60, 30);
            g2.setColor(Color.RED);
            g2.drawString("θ₂", width - 60, 50);
        }
    }
    
    static class GraficoTorques extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            int width = getWidth();
            int height = getHeight();
            int padding = 50;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Torques de Control", width/2 - 50, 20);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 10);
            g2.drawString("Torque [Nm]", 10, height/2);
            
            // Dibujar curvas de torques
            // τ1 - Azul
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(2));
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - ((tau1[i-1] - TAU_MIN) / (TAU_MAX - TAU_MIN)) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - ((tau1[i] - TAU_MIN) / (TAU_MAX - TAU_MIN)) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // τ2 - Rojo
            g2.setColor(Color.RED);
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - ((tau2[i-1] - TAU_MIN) / (TAU_MAX - TAU_MIN)) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - ((tau2[i] - TAU_MIN) / (TAU_MAX - TAU_MIN)) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Leyenda
            g2.setColor(Color.BLUE);
            g2.drawString("τ₁", width - 60, 30);
            g2.setColor(Color.RED);
            g2.drawString("τ₂", width - 60, 50);
        }
    }
    
    // Panel de análisis
    static class PanelAnalisis extends JPanel {
        public PanelAnalisis() {
            setBackground(Color.WHITE);
            setLayout(new BorderLayout());
            
            JTextArea textArea = new JTextArea();
            textArea.setEditable(false);
            textArea.setFont(new Font("Monospaced", Font.PLAIN, 12));
            
            StringBuilder sb = new StringBuilder();
            sb.append("=== ANÁLISIS DEL SISTEMA ===\n\n");
            
            sb.append("PARÁMETROS DEL BRAZO:\n");
            sb.append(String.format("  Longitud eslabón 1: %.1f m\n", L1));
            sb.append(String.format("  Longitud eslabón 2: %.1f m\n", L2));
            sb.append(String.format("  Masa eslabón 1: %.1f kg\n", M1));
            sb.append(String.format("  Masa eslabón 2: %.1f kg\n\n", M2));
            
            sb.append("CONFIGURACIÓN DESEADA:\n");
            sb.append(String.format("  θ₁: %.1f°\n", Math.toDegrees(THETA1_REF)));
            sb.append(String.format("  θ₂: %.1f°\n\n", Math.toDegrees(THETA2_REF)));
            
            sb.append("PARÁMETROS PID:\n");
            sb.append(String.format("  Kp: %.1f\n", KP));
            sb.append(String.format("  Ki: %.1f\n", KI));
            sb.append(String.format("  Kd: %.1f\n\n", KD));
            
            sb.append("RESULTADOS FINALES:\n");
            sb.append(String.format("  θ₁ final: %.2f°\n", Math.toDegrees(theta1[theta1.length-1])));
            sb.append(String.format("  θ₂ final: %.2f°\n", Math.toDegrees(theta2[theta2.length-1])));
            sb.append(String.format("  Error θ₁: %.3f°\n", Math.toDegrees(error1[error1.length-1])));
            sb.append(String.format("  Error θ₂: %.3f°\n", Math.toDegrees(error2[error2.length-1])));
            
            double x_ref = L1 * Math.cos(THETA1_REF) + L2 * Math.cos(THETA1_REF + THETA2_REF);
            double y_ref = L1 * Math.sin(THETA1_REF) + L2 * Math.sin(THETA1_REF + THETA2_REF);
            double pos_error = Math.sqrt(Math.pow(x_ref - x2[x2.length-1], 2) + 
                                       Math.pow(y_ref - y2[y2.length-1], 2));
            sb.append(String.format("  Error posición: %.4f m\n\n", pos_error));
            
            double maxTau1 = Arrays.stream(tau1).map(Math::abs).max().orElse(0.0);
            double maxTau2 = Arrays.stream(tau2).map(Math::abs).max().orElse(0.0);
            sb.append("TORQUES MÁXIMOS:\n");
            sb.append(String.format("  τ₁ max: %.1f Nm\n", maxTau1));
            sb.append(String.format("  τ₂ max: %.1f Nm\n", maxTau2));
            
            textArea.setText(sb.toString());
            add(new JScrollPane(textArea), BorderLayout.CENTER);
        }
    }
}