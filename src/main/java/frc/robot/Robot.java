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
            // Move para frente por tempo/iteração
            acionarMotores(0.3, 0.3);
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

    @Override
    public void teleopPeriodic() {
        double velocidade = -controle.getLeftY(); 
        double rotacao = controle.getRightX();

        // Cálculo de movimentação
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
        backLeft.set(vEsquerda); // Segue a esquerda 
        
        frontRight.set(vDireita);
        backRight.set(vDireita); // Segue a direita
    }

    private void stopAll() {
        acionarMotores(0, 0);
        shutterSuperior.set(0);
        shutterInferior.set(0);
    }
}
