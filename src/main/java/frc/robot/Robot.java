package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private SparkMax frontLeft, backLeft, frontRight, backRight;
    private SparkMax shutterSuperior, shutterInferior;
    private SparkRelativeEncoder encoderEsquerdo, encoderDireito;
    private XboxController controle;
    private int contadorAutonomo;

    private static final double Kp = 0.03;
    private static final double Kd = 0.01;
    private static final double VELOCIDADE_BASE = 0.3;

// Variáveis de estado do PD
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

        encoderEsquerdo = (SparkRelativeEncoder) frontLeft.getEncoder();
        encoderDireito = (SparkRelativeEncoder) frontRight.getEncoder();
    }

    @Override
    public void autonomousInit() {
        contadorAutonomo = 0;
        encoderEsquerdo.setPosition(0);
        encoderDireito.setPosition(0);
    }

    @Override
    public void autonomousPeriodic() {
      
        if (contadorAutonomo < 100) {
            andarRetoComPD(VELOCIDADE_BASE);
        } else if (contadorAutonomo < 200) {
            // Para e aciona o shutter
            acionarMotores(0, 0);
            shutterSuperior.set(0.6);
            shutterInferior.set(0.6);
        } else {
            stopAll();
        }
        contadorAutonomo++;
    }
    private void andarRetoComPD(double velocidadeBase) {
        // Erro = diferença entre os encoders (positivo = esquerdo andou mais)
        double posEsquerda = encoderEsquerdo.getPosition();
        double posDireita  = encoderDireito.getPosition();
        double erro = posEsquerda - posDireita;
 
        // Derivativa = variação do erro em relação ao ciclo anterior
        double derivativa = erro - erroAnterior;
        erroAnterior = erro;
 
        // Correção PD
        double correcao = (Kp * erro) + (Kd * derivativa);
 
        // Aplica a correção: se o lado esquerdo avançou mais, reduz esquerda e aumenta direita
        double vEsquerda =  velocidadeBase - correcao;
        double vDireita  = -(velocidadeBase + correcao); // negativo por inversão do lado direito
 
        acionarMotores(vEsquerda, vDireita);
    }
    @Override
    public void teleopPeriodic() {
        double velocidade = -controle.getLeftY(); 
        double rotacao = controle.getRightX();

        // Cálculo de movimentação arcade
        double esquerdo = velocidade + rotacao;
        double direito = -(velocidade - rotacao);

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

    private void acionarMotores(double vEsquerda, double vDireita) {
        frontLeft.set(vEsquerda);
        backLeft.set(vEsquerda); // Segue 
        
        frontRight.set(vDireita);
        backRight.set(vDireita); // Segue a direita
    }

    private void stopAll() {
        acionarMotores(0, 0);
        shutterSuperior.set(0);
        shutterInferior.set(0);
    }
}
