package frc.robot;

//bibliotecas da REV  pros motores
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;

//Classe para executar o ciclo do codigo a cada 20 milissegundos e para mapear os botões do controle
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

//variaveis ou objetos que eu vou usar durante o resto do codigo
public class Robot extends TimedRobot {
    private SparkMax frontLeft, backLeft, frontRight, backRight;
    private SparkMax shutterSuperior, shutterInferior;
    private SparkRelativeEncoder encoderEsquerdo, encoderDireito;
    private XboxController controle;
    private int contadorAutonomo;
//constante do PD e a velocidade padrão no autonomo(que estou desenvolvendo).
    private static final double Kp = 0.03;
    private static final double Kd = 0.01;
    private static final double VELOCIDADE_BASE = 0.3;

//erro de rota do ciclo anterior pro KD
private double erroAnterior = 0;
    @Override
    public void robotInit() {
        // Inicialização direta dos IDs
        frontLeft = new SparkMax(6, MotorType.kBrushed);
        backLeft = new SparkMax(7, MotorType.kBrushed);
        frontRight = new SparkMax(4, MotorType.kBrushed);
        backRight = new SparkMax(5, MotorType.kBrushed);
        
        shutterSuperior = new SparkMax(55, MotorType.kBrushed);
        shutterInferior = new SparkMax(44, MotorType.kBrushed);

        controle = new XboxController(0);

        //comando para ler as rotações das rodas
        encoderEsquerdo = (SparkRelativeEncoder) frontLeft.getEncoder();
        encoderDireito = (SparkRelativeEncoder) frontRight.getEncoder();
    }
//zera o cronometro e reseta as posições
    @Override
    public void autonomousInit() {
        contadorAutonomo = 0;
        encoderEsquerdo.setPosition(0);
        encoderDireito.setPosition(0);
    }

    @Override
    public void autonomousPeriodic() {
      
        if (contadorAutonomo < 100) { //cem ciclos para andar 2 segundos
            andarRetoComPD(VELOCIDADE_BASE);
        } else if (contadorAutonomo < 200) {
            // Para e aciona o shutter
            acionarMotores(0, 0);
            shutterSuperior.set(0.6);
            shutterInferior.set(0.6);
        } else {
            stopAll();
        }
        contadorAutonomo++; //adiciona 1 a cada fim de ciclo
    }
    private void andarRetoComPD(double velocidadeBase) {
        // Erro é positivo se o esquerdo andou mais e negativo se o direito andou mais
        double posEsquerda = encoderEsquerdo.getPosition();
        double posDireita  = encoderDireito.getPosition();
        double erro = posEsquerda - posDireita;
 
        //calcula a velocidade com que o robô está se desviando da linha reta e atualiza a variavel pro proximo ciclo que vai acontecer
        double derivativa = erro - erroAnterior;
        erroAnterior = erro;
 
        // Correção PD
        double correcao = (Kp * erro) + (Kd * derivativa);
 
        // Aplica a correção, se o lado esquerdo avançou mais, reduz esquerda e aumenta direita
        double vEsquerda =  velocidadeBase - correcao;
        double vDireita  = -(velocidadeBase + correcao); // negativo por inversão do lado direito
 //envia essas velocidades que foram calculadas para o motor
        acionarMotores(vEsquerda, vDireita);
    }
    @Override
    public void teleopPeriodic() {
        double velocidade = -controle.getLeftY();  //negativo porque a WPI le o valor negativo quando o joystick vai para frente
        double rotacao = controle.getRightX(); //curvas

        // Calculo de movimentação, se quero somente ir para frente, fazer uma curva ou os dois ao mesmo tempo
        double esquerdo = velocidade + rotacao;
        double direito = -(velocidade - rotacao);

        //envia os comandos do controle para as rodas.
        acionarMotores(esquerdo, direito);
        
        // Controle do Shutter
        if (controle.getAButton()) {
            shutterSuperior.set(0.8);
            shutterInferior.set(0.8);
        } else {
            shutterSuperior.set(0);
            shutterInferior.set(0);
        }
    }

    //função para andar
    private void acionarMotores(double vEsquerda, double vDireita) {
        frontLeft.set(vEsquerda);
        backLeft.set(vEsquerda); // Segue 
        
        frontRight.set(vDireita);
        backRight.set(vDireita); // Segue a direita
    }
//função para quando largar os botões parar tudo
    private void stopAll() {
        acionarMotores(0, 0);
        shutterSuperior.set(0);
        shutterInferior.set(0);
    }
}
