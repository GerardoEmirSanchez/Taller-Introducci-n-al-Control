import java.awt.*;
import java.util.*;
import javax.swing.*;
import javax.swing.Timer;

public class C_04_ControlElevadorPID {
    // Parámetros del sistema ELEVADOR
    private static final double POS_REF = 10.0;          // Posición deseada [m]
    private static final double POS_INICIAL = 0.0;       // Posición inicial [m]
    private static final double VEL_INICIAL = 0.0;       // Velocidad inicial [m/s]
    
    // Parámetros del modelo
    private static final double K = 1.0;                 // Ganancia [m/N]
    private static final double TAU = 2.0;               // Constante de tiempo [s]
    private static final double ZETA = 0.7;              // Coeficiente de amortiguamiento
    
    // Parámetros PID
    private static final double KP = 3.0;                // Ganancia proporcional [N/m]
    private static final double KI = 0.5;                // Ganancia integral [N/m·s]
    private static final double KD = 4.0;                // Ganancia derivativa [N·s/m]
    
    // Límites del actuador
    private static final double U_MAX = 1000.0;          // Fuerza máxima [N]
    private static final double U_MIN = -1000.0;         // Fuerza mínima [N]
    
    // Tiempo de simulación
    private static final double T_SIM = 20.0;
    private static final double DT = 0.01;
    
    // Variables de simulación
    private static double[] tiempo;
    private static double[] posicion;
    private static double[] fuerza;
    private static double[] error;
    
    public static void main(String[] args) {
        System.out.println("=== CONTROL DE POSICIÓN DE ELEVADOR - SISTEMA DE 2DO ORDEN CON CONTROL PID ===\n");
        
        System.out.println("=== MODELO DEL ELEVADOR ===");
        System.out.printf("G(s) = %.1f m/N / (%.1fs² + %.1fs + 1)\n", K, TAU*TAU, 2*ZETA*TAU);
        
        // Ejecutar simulación
        simularSistema();
        
        // Mostrar resultados
        mostrarResultados();
        
        // Iniciar animación
        SwingUtilities.invokeLater(() -> iniciarAnimacion());
    }
    
    private static void simularSistema() {
        int N = (int)(T_SIM / DT) + 1;
        tiempo = new double[N];
        posicion = new double[N];
        fuerza = new double[N];
        error = new double[N];
        
        // Inicializar arrays
        for (int i = 0; i < N; i++) {
            tiempo[i] = i * DT;
        }
        
        // Coeficientes de discretización
        double coef_posk = 1/(DT*DT) + (2*ZETA)/(TAU*DT) + 1/(TAU*TAU);
        double coef_posk1 = -2/(DT*DT) - (2*ZETA)/(TAU*DT);
        double coef_posk2 = 1/(DT*DT);
        
        // Condiciones iniciales
        posicion[0] = POS_INICIAL;
        posicion[1] = POS_INICIAL;
        
        double integral = 0.0;
        double error_prev = 0.0;
        
        // Simulación del control PID
        for (int k = 2; k < N; k++) {
            // Cálculo del error actual
            error[k] = POS_REF - posicion[k-1];
            
            // Término Proporcional
            double P_term = KP * error[k];
            
            // Término Integral
            integral += (error[k] + error_prev) * DT / 2;
            double I_term = KI * integral;
            
            // Término Derivativo
            double derivative = (error[k] - error_prev) / DT;
            double D_term = KD * derivative;
            
            // Señal de control total
            fuerza[k] = P_term + I_term + D_term;
            fuerza[k] = Math.max(Math.min(fuerza[k], U_MAX), U_MIN);
            
            // Anti-windup
            if (fuerza[k] >= U_MAX || fuerza[k] <= U_MIN) {
                integral -= (error[k] + error_prev) * DT / 2;
            }
            
            // Actualizar error anterior
            error_prev = error[k];
            
            // Simulación del elevador
            posicion[k] = (-coef_posk1 * posicion[k-1] - coef_posk2 * posicion[k-2] + 
                          (K/(TAU*TAU)) * fuerza[k]) / coef_posk;
        }
        
        System.out.println("Simulación completada exitosamente");
    }
    
    private static void mostrarResultados() {
        System.out.println("\n=== RESULTADOS DE LA SIMULACIÓN ===");
        System.out.printf("Posición inicial: %.1f m\n", POS_INICIAL);
        System.out.printf("Posición deseada: %.1f m\n", POS_REF);
        System.out.printf("Posición final: %.3f m\n", posicion[posicion.length-1]);
        System.out.printf("Error estacionario: %.4f m\n", error[error.length-1]);
        System.out.printf("Tiempo de simulación: %.1f s\n", T_SIM);
        
        // Calcular fuerza máxima aplicada
        double maxFuerza = 0;
        for (double f : fuerza) {
            maxFuerza = Math.max(maxFuerza, Math.abs(f));
        }
        System.out.printf("Fuerza máxima aplicada: %.1f N\n", maxFuerza);
        
        // Calcular tiempo de establecimiento
        double tolerance = 0.02 * POS_REF;
        double settlingTime = T_SIM;
        for (int i = 0; i < error.length; i++) {
            if (Math.abs(error[i]) < tolerance) {
                settlingTime = tiempo[i];
                break;
            }
        }
        System.out.printf("Tiempo de establecimiento: %.2f s\n", settlingTime);
        
        // Calcular sobrepico máximo
        double maxPos = Arrays.stream(posicion).max().orElse(POS_REF);
        double overshoot = Math.max(0, maxPos - POS_REF);
        System.out.printf("Sobrepico máximo: %.3f m\n", overshoot);
    }
    
    private static void iniciarAnimacion() {
        System.out.println("\n=== INICIANDO ANIMACIÓN ===");
        
        JFrame frame = new JFrame("Control de Elevador - Sistema PID");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.addTab("Animación", new PanelAnimacion());
        tabbedPane.addTab("Gráficos", new PanelGraficos());
        tabbedPane.addTab("Análisis", new PanelAnalisis());
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    // Panel de animación
    static class PanelAnimacion extends JPanel {
        private static final int ELEVADOR_WIDTH = 60;
        private static final int ELEVADOR_HEIGHT = 80;
        private static final int EDIFICIO_WIDTH = 300;
        private static final int EDIFICIO_HEIGHT = 400;
        private static final int PADDING = 50;
        
        private Timer timer;
        private int currentIndex = 0;
        private double scaleY;
        
        public PanelAnimacion() {
            setBackground(Color.WHITE);
            scaleY = (EDIFICIO_HEIGHT - 2 * PADDING) / (POS_REF + 2);
            
            // Configurar timer para animación
            timer = new Timer(50, e -> {
                if (currentIndex < tiempo.length - 1) {
                    currentIndex += 5; // Acelerar animación
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
            int centerX = width / 2;
            
            // Dibujar edificio
            g2.setColor(new Color(200, 200, 200));
            g2.fillRect(centerX - EDIFICIO_WIDTH/2, PADDING, EDIFICIO_WIDTH, EDIFICIO_HEIGHT);
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(3));
            g2.drawRect(centerX - EDIFICIO_WIDTH/2, PADDING, EDIFICIO_WIDTH, EDIFICIO_HEIGHT);
            
            // Dibujar pisos
            g2.setStroke(new BasicStroke(2));
            for (int piso = 0; piso <= POS_REF; piso++) {
                int y = getYFromPosition(piso);
                g2.drawLine(centerX - EDIFICIO_WIDTH/2, y, centerX + EDIFICIO_WIDTH/2, y);
                
                // Etiqueta del piso
                g2.drawString("Piso " + piso, centerX + EDIFICIO_WIDTH/2 + 10, y + 5);
            }
            
            // Dibujar piso destino
            g2.setColor(Color.RED);
            g2.setStroke(new BasicStroke(2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{5}, 0));
            int yDestino = getYFromPosition(POS_REF);
            g2.drawLine(centerX - EDIFICIO_WIDTH/2, yDestino, centerX + EDIFICIO_WIDTH/2, yDestino);
            g2.drawString("DESTINO", centerX + EDIFICIO_WIDTH/2 + 10, yDestino + 5);
            
            // Dibujar cables
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(2));
            int currentY = getYFromPosition(posicion[currentIndex]);
            g2.drawLine(centerX - ELEVADOR_WIDTH/2, currentY + ELEVADOR_HEIGHT, 
                       centerX - EDIFICIO_WIDTH/2, PADDING);
            g2.drawLine(centerX + ELEVADOR_WIDTH/2, currentY + ELEVADOR_HEIGHT, 
                       centerX + EDIFICIO_WIDTH/2, PADDING);
            
            // Dibujar elevador
            g2.setColor(new Color(50, 150, 255));
            g2.fillRoundRect(centerX - ELEVADOR_WIDTH/2, currentY, 
                           ELEVADOR_WIDTH, ELEVADOR_HEIGHT, 15, 15);
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(3));
            g2.drawRoundRect(centerX - ELEVADOR_WIDTH/2, currentY, 
                           ELEVADOR_WIDTH, ELEVADOR_HEIGHT, 15, 15);
            
            // Puertas del elevador
            g2.setColor(Color.DARK_GRAY);
            g2.fillRect(centerX - ELEVADOR_WIDTH/2 + 5, currentY + 10, 
                       ELEVADOR_WIDTH - 10, ELEVADOR_HEIGHT - 20);
            
            // Información en tiempo real
            g2.setColor(Color.BLACK);
            g2.setFont(new Font("Arial", Font.BOLD, 14));
            String info = String.format("Tiempo: %.1f s\nPosición: %.2f m\nError: %.3f m\nFuerza: %.1f N",
                                      tiempo[currentIndex], posicion[currentIndex], 
                                      error[currentIndex], fuerza[currentIndex]);
            drawMultiLineString(g2, info, 20, 50);
            
            // Título
            g2.drawString("ANIMACIÓN DEL ELEVADOR - CONTROL PID", width/2 - 150, 30);
        }
        
        private int getYFromPosition(double pos) {
            return (int)(PADDING + EDIFICIO_HEIGHT - (pos * scaleY));
        }
        
        private void drawMultiLineString(Graphics2D g2, String text, int x, int y) {
            String[] lines = text.split("\n");
            for (int i = 0; i < lines.length; i++) {
                g2.drawString(lines[i], x, y + i * 20);
            }
        }
    }
    
    // Panel de gráficos
    static class PanelGraficos extends JPanel {
        public PanelGraficos() {
            setBackground(Color.WHITE);
            setLayout(new GridLayout(2, 2));
            
            add(new GraficoPosicion());
            add(new GraficoError());
            add(new GraficoFuerza());
            add(new GraficoComparacion());
        }
    }
    
    static class GraficoPosicion extends JPanel {
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
            g2.drawString("Trayectoria del Elevador", width/2 - 60, 20);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 10);
            g2.drawString("Posición [m]", 10, height/2);
            
            // Dibujar curva de posición
            g2.setColor(Color.BLUE);
            g2.setStroke(new BasicStroke(2));
            
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - (posicion[i-1] / (POS_REF + 2)) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - (posicion[i] / (POS_REF + 2)) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Dibujar referencia
            g2.setColor(Color.RED);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{5}, 0));
            int yRef = (int)(height - padding - (POS_REF / (POS_REF + 2)) * (height - 2 * padding));
            g2.drawLine(padding, yRef, width - padding, yRef);
            
            // Leyenda
            g2.setColor(Color.BLUE);
            g2.drawString("Posición Real", width - 120, 30);
            g2.setColor(Color.RED);
            g2.drawString("Referencia", width - 120, 50);
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
            int padding = 50;
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            g2.drawLine(padding, height - padding, padding, padding);
            
            // Títulos
            g2.drawString("Error de Posición", width/2 - 50, 20);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 10);
            g2.drawString("Error [m]", 10, height/2);
            
            // Encontrar máximo error para escalado
            double maxError = Arrays.stream(error).map(Math::abs).max().orElse(1.0);
            
            // Dibujar curva de error
            g2.setColor(Color.MAGENTA);
            g2.setStroke(new BasicStroke(2));
            
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - ((error[i-1] + maxError) / (2 * maxError)) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - ((error[i] + maxError) / (2 * maxError)) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
            
            // Línea de error cero
            g2.setColor(Color.BLACK);
            g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                0, new float[]{3}, 0));
            int zeroY = (int)(height - padding - (height - 2 * padding) / 2);
            g2.drawLine(padding, zeroY, width - padding, zeroY);
        }
    }
    
    static class GraficoFuerza extends JPanel {
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
            g2.drawString("Fuerza del Motor", width/2 - 50, 20);
            g2.drawString("Tiempo [s]", width/2 - 30, height - 10);
            g2.drawString("Fuerza [N]", 10, height/2);
            
            // Dibujar curva de fuerza
            g2.setColor(Color.GREEN);
            g2.setStroke(new BasicStroke(2));
            
            for (int i = 1; i < tiempo.length; i++) {
                int x1 = (int)(padding + (tiempo[i-1] / T_SIM) * (width - 2 * padding));
                int y1 = (int)(height - padding - ((fuerza[i-1] - U_MIN) / (U_MAX - U_MIN)) * (height - 2 * padding));
                int x2 = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y2 = (int)(height - padding - ((fuerza[i] - U_MIN) / (U_MAX - U_MIN)) * (height - 2 * padding));
                g2.drawLine(x1, y1, x2, y2);
            }
        }
    }
    
    static class GraficoComparacion extends JPanel {
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            // Dibujar las tres curvas juntas para comparación
            int width = getWidth();
            int height = getHeight();
            int padding = 50;
            
            // Título
            g2.drawString("Comparación de Señales", width/2 - 60, 20);
            
            // Dibujar posición (azul)
            g2.setColor(Color.BLUE);
            for (int i = 1; i < tiempo.length; i += 10) {
                int x = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y = (int)(height - padding - (posicion[i] / (POS_REF + 2)) * (height - 2 * padding));
                g2.fillOval(x-1, y-1, 3, 3);
            }
            
            // Dibujar error (magenta)
            g2.setColor(Color.MAGENTA);
            double maxError = Arrays.stream(error).map(Math::abs).max().orElse(1.0);
            for (int i = 1; i < tiempo.length; i += 10) {
                int x = (int)(padding + (tiempo[i] / T_SIM) * (width - 2 * padding));
                int y = (int)(height - padding - ((error[i] + maxError) / (2 * maxError)) * (height - 2 * padding));
                g2.fillOval(x-1, y-1, 3, 3);
            }
            
            // Leyenda
            g2.setColor(Color.BLUE);
            g2.drawString("Posición", width - 80, 40);
            g2.setColor(Color.MAGENTA);
            g2.drawString("Error", width - 80, 60);
            g2.setColor(Color.GREEN);
            g2.drawString("Fuerza", width - 80, 80);
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
            sb.append(String.format("PARÁMETROS DEL ELEVADOR:\n"));
            sb.append(String.format("  Ganancia K: %.1f m/N\n", K));
            sb.append(String.format("  Constante de tiempo τ: %.1f s\n", TAU));
            sb.append(String.format("  Amortiguamiento ζ: %.1f\n\n", ZETA));
            
            sb.append("PARÁMETROS PID:\n");
            sb.append(String.format("  Kp: %.1f N/m\n", KP));
            sb.append(String.format("  Ki: %.1f N/m·s\n", KI));
            sb.append(String.format("  Kd: %.1f N·s/m\n\n", KD));
            
            sb.append("RESULTADOS:\n");
            sb.append(String.format("  Posición final: %.3f m\n", posicion[posicion.length-1]));
            sb.append(String.format("  Error estacionario: %.4f m\n", error[error.length-1]));
            
            double maxFuerza = Arrays.stream(fuerza).map(Math::abs).max().orElse(0.0);
            sb.append(String.format("  Fuerza máxima: %.1f N\n", maxFuerza));
            
            double maxPos = Arrays.stream(posicion).max().orElse(POS_REF);
            double overshoot = Math.max(0, maxPos - POS_REF);
            sb.append(String.format("  Sobrepico: %.3f m\n", overshoot));
            
            textArea.setText(sb.toString());
            add(new JScrollPane(textArea), BorderLayout.CENTER);
        }
    }
}