import java.awt.*;
import java.util.Arrays;
import javax.swing.*;

public class C_01_Control_PID {
    // Parametros del sistema
    private static final double T_REF = 22.0;          // Referencia deseada [C]
    private static final double T_AMB = 15.0;          // Temperatura ambiente [C]
    private static final double T_INICIAL = 15.0;      // Temperatura inicial [C]
    
    // Parametros del modelo: G(s) = K / (tau^2 s^2 + 2*zeta*tau*s + 1)
    private static final double K = 1.0;               // Ganancia [C/W]
    private static final double TAU = 1.0;             // Constante de tiempo [s]
    private static final double ZETA = 0.5;            // Coeficiente de amortiguamiento
    
    // Parametros PID
    private static final double KP = 1.5;              // Ganancia proporcional [W/C]
    private static final double KI = 1.0;              // Ganancia integral [W/C·s]
    private static final double KD = 0.0;              // Ganancia derivativa [W·s/C]
    
    // Limites del actuador
    private static final double U_MAX = 50.0;          // Potencia maxima [W]
    private static final double U_MIN = 0.0;           // Potencia minima [W]
    
    // Tiempo de simulacion
    private static final double T_SIM = 20.0;
    private static final double DT = 0.01;
    
    public static void main(String[] args) {
        System.out.println("=== MODELO DEL SISTEMA ===");
        System.out.printf("G(s) = %.1fC/W / (%.0fs^2 + %.1fs + 1)\n", K, TAU*TAU, 2*ZETA*TAU);
        
        System.out.println("\n=== PARAMETROS PID ===");
        System.out.printf("Kp = %.2f [W/C]\n", KP);
        System.out.printf("Ki = %.2f [W/C·s]\n", KI);
        System.out.printf("Kd = %.2f [W·s/C]\n", KD);
        
        int N = (int)(T_SIM / DT) + 1;
        double[] t = new double[N];
        for (int i = 0; i < N; i++) {
            t[i] = i * DT;
        }
        
        // DISCRETIZACION del sistema de 2do orden
        double coef_Tk = 1/(DT*DT) + (2*ZETA)/(TAU*DT) + 1/(TAU*TAU);
        double coef_Tk1 = -2/(DT*DT) - (2*ZETA)/(TAU*DT);
        double coef_Tk2 = 1/(DT*DT);
        
        // 1. LAZO ABIERTO
        double ganOpen = 2.0;
        double[] uOpen = new double[N];
        double[] TOpen = new double[N];
        double[] errorOpen = new double[N];
        
        Arrays.fill(TOpen, T_INICIAL);
        TOpen[1] = T_INICIAL;
        
        for (int i = 0; i < N; i++) {
            uOpen[i] = (T_REF - T_AMB) * ganOpen;
        }
        
        for (int k = 2; k < N; k++) {
            TOpen[k] = (-coef_Tk1 * TOpen[k-1] - coef_Tk2 * TOpen[k-2] + 
                        (K/(TAU*TAU)) * uOpen[k] + (1/(TAU*TAU)) * T_AMB) / coef_Tk;
            errorOpen[k] = T_REF - TOpen[k];
        }
        
        // 2. LAZO CERRADO con Control PID
        double[] TPid = new double[N];
        double[] uPid = new double[N];
        double[] errorPid = new double[N];
        
        Arrays.fill(TPid, T_INICIAL);
        TPid[1] = T_INICIAL;
        
        double integral = 0.0;
        double errorPrev = 0.0;
        
        for (int k = 2; k < N; k++) {
            // Calculo del error actual
            errorPid[k] = T_REF - TPid[k-1];
            
            // Termino Proporcional
            double PTerm = KP * errorPid[k];
            
            // Termino Integral (integracion trapezoidal)
            integral += (errorPid[k] + errorPrev) * DT / 2;
            double ITerm = KI * integral;
            
            // Termino Derivativo
            double derivative = (errorPid[k] - errorPrev) / DT;
            double DTerm = KD * derivative;
            
            // Senal de control total
            uPid[k] = PTerm + ITerm + DTerm;
            
            // Saturacion del actuador
            uPid[k] = Math.max(Math.min(uPid[k], U_MAX), U_MIN);
            
            // Anti-windup
            if (uPid[k] >= U_MAX || uPid[k] <= U_MIN) {
                integral -= (errorPid[k] + errorPrev) * DT / 2;
            }
            
            // Actualizar error anterior
            errorPrev = errorPid[k];
            
            // Simulacion del sistema
            TPid[k] = (-coef_Tk1 * TPid[k-1] - coef_Tk2 * TPid[k-2] + 
                      (K/(TAU*TAU)) * uPid[k] + (1/(TAU*TAU)) * T_AMB) / coef_Tk;
        }
        
        // 3. CONTROL P SIMPLE
        double KpSimple = KP;
        double[] TSimple = new double[N];
        double[] uSimple = new double[N];
        double[] errorSimple = new double[N];
        
        Arrays.fill(TSimple, T_INICIAL);
        TSimple[1] = T_INICIAL;
        
        for (int k = 2; k < N; k++) {
            errorSimple[k] = T_REF - TSimple[k-1];
            uSimple[k] = KpSimple * errorSimple[k];
            uSimple[k] = Math.max(Math.min(uSimple[k], U_MAX), U_MIN);
            
            TSimple[k] = (-coef_Tk1 * TSimple[k-1] - coef_Tk2 * TSimple[k-2] + 
                         (K/(TAU*TAU)) * uSimple[k] + (1/(TAU*TAU)) * T_AMB) / coef_Tk;
        }
        
        // 4. Calculo de metricas de desempeno
        double ISEOpen = calculateISE(errorOpen, DT);
        double ISESimple = calculateISE(errorSimple, DT);
        double ISEPid = calculateISE(errorPid, DT);
        
        double tSettleOpen = calculateSettlingTime(t, errorOpen, 0.02 * T_REF);
        double tSettleSimple = calculateSettlingTime(t, errorSimple, 0.02 * T_REF);
        double tSettlePid = calculateSettlingTime(t, errorPid, 0.02 * T_REF);
        
        double overshootOpen = calculateOvershoot(TOpen, T_REF);
        double overshootSimple = calculateOvershoot(TSimple, T_REF);
        double overshootPid = calculateOvershoot(TPid, T_REF);
        
        // 5. Resultados Detallados
        printResults(errorOpen, errorSimple, errorPid, 
                    tSettleOpen, tSettleSimple, tSettlePid,
                    overshootOpen, overshootSimple, overshootPid,
                    ISEOpen, ISESimple, ISEPid,
                    uOpen, uSimple, uPid);
        
        // 6. Mostrar graficos
        showGraphs(t, TOpen, TSimple, TPid, errorOpen, errorSimple, errorPid, 
                  uOpen, uSimple, uPid);
    }
    
    private static double calculateISE(double[] error, double dt) {
        double ise = 0.0;
        for (double e : error) {
            ise += e * e * dt;
        }
        return ise;
    }
    
    private static double calculateSettlingTime(double[] t, double[] error, double tolerance) {
        for (int i = 0; i < error.length; i++) {
            if (Math.abs(error[i]) < tolerance) {
                return t[i];
            }
        }
        return t[t.length - 1];
    }
    
    private static double calculateOvershoot(double[] response, double reference) {
        double max = Arrays.stream(response).max().orElse(reference);
        return Math.max(0, max - reference);
    }
    
    private static void printResults(double[] errorOpen, double[] errorSimple, double[] errorPid,
                                   double tSettleOpen, double tSettleSimple, double tSettlePid,
                                   double overshootOpen, double overshootSimple, double overshootPid,
                                   double ISEOpen, double ISESimple, double ISEPid,
                                   double[] uOpen, double[] uSimple, double[] uPid) {
        System.out.println("\n=== RESULTADOS COMPARATIVOS ===");
        System.out.println("METRICA               | LAZO ABIERTO | CONTROL P   | CONTROL PID");
        System.out.println("----------------------------------------------------------------");
        System.out.printf("Error Estacionario    | %6.3f C   | %6.3f C   | %6.3f C\n",
                errorOpen[errorOpen.length-1], errorSimple[errorSimple.length-1], 
                errorPid[errorPid.length-1]);
        System.out.printf("Tiempo Establecimiento| %6.2f s     | %6.2f s     | %6.2f s\n",
                tSettleOpen, tSettleSimple, tSettlePid);
        System.out.printf("Sobrepico Maximo     | %6.3f C   | %6.3f C   | %6.3f C\n",
                overshootOpen, overshootSimple, overshootPid);
        System.out.printf("ISE                  | %6.3f      | %6.3f      | %6.3f\n",
                ISEOpen, ISESimple, ISEPid);
        System.out.printf("Potencia Final       | %6.2f W    | %6.2f W    | %6.2f W\n",
                uOpen[uOpen.length-1], uSimple[uSimple.length-1], uPid[uPid.length-1]);
        
        System.out.println("\n=== RECOMENDACIONES PARA AJUSTAR PID ===");
        System.out.println("• Para reducir sobrepico: DISMINUIR Kp o AUMENTAR Kd");
        System.out.println("• Para acelerar respuesta: AUMENTAR Kp");
        System.out.println("• Para eliminar error estacionario: AUMENTAR Ki");
        System.out.println("• Para reducir oscilaciones: AUMENTAR Kd o DISMINUIR Ki");
    }
    
    private static void showGraphs(double[] t, double[] TOpen, double[] TSimple, double[] TPid,
                                 double[] errorOpen, double[] errorSimple, double[] errorPid,
                                 double[] uOpen, double[] uSimple, double[] uPid) {
        JFrame frame = new JFrame("Control de Temperatura - Sistema de 2do Orden");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1400, 800);
        
        JTabbedPane tabbedPane = new JTabbedPane();
        
        // Panel de temperaturas
        tabbedPane.addTab("Temperaturas", new GraphPanel(t, TOpen, TSimple, TPid, 
                "Tiempo [s]", "Temperatura [C]", "Respuesta de Temperatura",
                new String[]{"Lazo Abierto", "Control P", "Control PID", "Referencia"}));
        
        // Panel de errores
        tabbedPane.addTab("Errores", new GraphPanel(t, errorOpen, errorSimple, errorPid, 
                "Tiempo [s]", "Error [C]", "Error de Temperatura",
                new String[]{"Lazo Abierto", "Control P", "Control PID"}));
        
        // Panel de control
        tabbedPane.addTab("Senal de Control", new GraphPanel(t, uOpen, uSimple, uPid, 
                "Tiempo [s]", "Potencia [W]", "Senal de Control Total",
                new String[]{"Lazo Abierto", "Control P", "Control PID"}));
        
        frame.add(tabbedPane);
        frame.setVisible(true);
    }
    
    static class GraphPanel extends JPanel {
        private double[] t, data1, data2, data3;
        private String xLabel, yLabel, title;
        private String[] legends;
        
        public GraphPanel(double[] t, double[] data1, double[] data2, double[] data3,
                         String xLabel, String yLabel, String title, String[] legends) {
            this.t = t;
            this.data1 = data1;
            this.data2 = data2;
            this.data3 = data3;
            this.xLabel = xLabel;
            this.yLabel = yLabel;
            this.title = title;
            this.legends = legends;
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
            
            // Encontrar limites de los datos
            double minY = Double.MAX_VALUE;
            double maxY = -Double.MAX_VALUE;
            
            for (double[] data : new double[][]{data1, data2, data3}) {
                for (double val : data) {
                    if (val < minY) minY = val;
                    if (val > maxY) maxY = val;
                }
            }
            
            // Dibujar ejes
            g2.setColor(Color.BLACK);
            g2.drawLine(padding, height - padding, padding, padding);
            g2.drawLine(padding, height - padding, width - padding, height - padding);
            
            // Dibujar datos
            drawCurve(g2, t, data1, Color.BLUE, padding, graphWidth, graphHeight, minY, maxY);
            drawCurve(g2, t, data2, Color.GREEN, padding, graphWidth, graphHeight, minY, maxY);
            drawCurve(g2, t, data3, Color.RED, padding, graphWidth, graphHeight, minY, maxY);
            
            // Dibujar referencia si es el grafico de temperaturas
            if (title.contains("Temperatura")) {
                g2.setColor(Color.BLACK);
                g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{5}, 0));
                double yRef = mapY(T_REF, graphHeight, minY, maxY);
                g2.drawLine(padding, (int)(height - padding - yRef), width - padding, (int)(height - padding - yRef));
            }
            
            // Leyenda
            drawLegend(g2, width, height, padding);
            
            // Titulos
            g2.setColor(Color.BLACK);
            g2.drawString(title, width / 2 - 50, padding / 2);
            g2.drawString(xLabel, width / 2 - 20, height - padding / 3);
            g2.drawString(yLabel, padding / 4, height / 2);
        }
        
        private void drawCurve(Graphics2D g2, double[] x, double[] y, Color color, 
                             int padding, int graphWidth, int graphHeight, double minY, double maxY) {
            g2.setColor(color);
            g2.setStroke(new BasicStroke(2));
            
            for (int i = 1; i < x.length; i++) {
                int x1 = (int)(padding + (x[i-1] / x[x.length-1]) * graphWidth);
                int y1 = (int)(getHeight() - padding - mapY(y[i-1], graphHeight, minY, maxY));
                int x2 = (int)(padding + (x[i] / x[x.length-1]) * graphWidth);
                int y2 = (int)(getHeight() - padding - mapY(y[i], graphHeight, minY, maxY));
                g2.drawLine(x1, y1, x2, y2);
            }
        }
        
        private double mapY(double value, int graphHeight, double minY, double maxY) {
            return ((value - minY) / (maxY - minY)) * graphHeight;
        }
        
        private void drawLegend(Graphics2D g2, int width, int height, int padding) {
            int legendX = width - 200;
            int legendY = padding + 20;
            
            g2.setColor(Color.BLUE);
            g2.drawString(legends[0], legendX, legendY);
            g2.setColor(Color.GREEN);
            g2.drawString(legends[1], legendX, legendY + 20);
            g2.setColor(Color.RED);
            g2.drawString(legends[2], legendX, legendY + 40);
            
            if (legends.length > 3) {
                g2.setColor(Color.BLACK);
                g2.setStroke(new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{5}, 0));
                g2.drawString(legends[3], legendX, legendY + 60);
            }
        }
    }
}