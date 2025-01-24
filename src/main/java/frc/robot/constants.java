package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
     
    /** 
    * @author  Fernando Joel Cruz Briones
    * @version 1.0
    */
public class constants {
    
    /**
     * drivetrainConstants
     *
     * <p>Clase que reúne diversas constantes utilizadas en la configuración
     * de un tren motriz (drivetrain) Swerve. Incluye valores de PID, 
     * relaciones de engranajes, offsets de encoders, inercia para simulación,
     * y posiciones físicas de los módulos (ruedas) respecto al centro del robot.</p>
     */
    public static final class drivetrainConstants {

        //SteerMotor

        /**
         * Ganancia Proporcional (KP) para lazo cerrado de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKP = 50;
    
        /**
         * Ganancia Integral (KI) para lazo cerrado de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKI = 0;
    
        /**
         * Ganancia Derivativa (KD) para lazo cerrado de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKD = 0.5;
    
        /**
         * Término de feedforward estático (KS) para lazo cerrado de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKS = 0.1;
    
        /**
         * Término de feedforward proporcional a la velocidad (KV) de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKV = 2.66;
    
        /**
         * Término de feedforward proporcional a la aceleración (KA) de los motores de dirección (steer).
         */
        public static final double Slot0ConfigSteerGainsWithKA = 0;
    
        //DriveMotor
    
        /**
         * Ganancia Proporcional (KP) para lazo cerrado de los motores de propulsión (drive).
         */
        public static final double Slot0ConfigDriveGainsWithKP = 0.1;
    
        /**
         * Ganancia Integral (KI) para lazo cerrado de los motores de propulsión (drive).
         */
        public static final double Slot0ConfigDriveGainsWithKI = 0;
    
        /**
         * Ganancia Derivativa (KD) para lazo cerrado de los motores de propulsión (drive).
         */
        public static final double Slot0ConfigDriveGainsWithKD = 0;
    
        /**
         * Término de feedforward estático (KS) para lazo cerrado de los motores de propulsión (drive).
         */
        public static final double Slot0ConfigDriveGainsWithKS = 0;
    
        /**
         * Término de feedforward proporcional a la velocidad (KV) de los motores de propulsión (drive).
         */
        public static final double Slot0ConfigDriveGainsWithKV = 0.124;
    
        //DrivetrainCofigs
    
        /**
         * Corriente a la que se considera que la rueda comienza a patinar (slip).
         */
        public static final double kSlipCurrentAmp = 120;
    
        /**
         * Límite de corriente de estator para los motores de dirección (stator current limit).
         */
        public static final double StatorCurrentLimitAmps = 60;
    
        /**
         * Bandera para habilitar o deshabilitar el límite de corriente de estator.
         */
        public static final boolean StatorCurrentLimitEnable = true;
    
        /**
         * Nombre del bus CANivore a utilizar para los dispositivos CTRE.
         */
        public static final String CANivoreCanBusName = "6348 Horus CANivore";
    
        /**
         * Velocidad teórica (m/s) alcanzada a 12 V. Se utiliza para ajustar
         * el feedforward en la tracción (drive).
         */
        public static final double kSpeedAt12Volts = 3.92;
    
        /**
         * Relación de acoplamiento entre el giro de la dirección y el giro
         * del motor de tracción (drive). Puede requerir ajuste específico en cada robot.
         */
        public static final double kCoupleRatio = 3.5714285714285716;
    
        /**
         * Relación de engranaje de los motores de propulsión (drive).
         */
        public static final double kDriveGearRatio = 8.142857142857142;
    
        /**
         * Relación de engranaje de los motores de dirección (steer).
         */
        public static final double kSteerGearRatio = 21.428571428571427;
    
        /**
         * Radio de la rueda en pulgadas (convertido a objeto Distance).
         */ 
        
        public static final Distance kWheelRadius = Inches.of(2);
    
        /**
         * Indica si se invierten los motores en el lado izquierdo.
         */
        public static final boolean kInvertLeftSide = true;
    
        /**
         * Indica si se invierten los motores en el lado derecho.
         */
        public static final boolean kInvertRightSide = false;
    
        /**
         * ID del Pigeon2 para el giroscopio, usado en la estabilización del robot.
         */
        public static final int kPigeonId = 13;
    
        /**
         * Momento de inercia simulado para el sistema de dirección.
         */
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    
        /**
         * Momento de inercia simulado para el sistema de propulsión.
         */
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    
        /**
         * Voltaje simulado necesario para superar la fricción en el steer.
         */
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    
        /**
         * Voltaje simulado necesario para superar la fricción en el drive.
         */
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);
    
        //size Config
    
        /**
         * Mitad de la longitud del robot en el eje X (pulgadas).
         */
        private static final double xLenght = 29.5 / 2;
    
        /**
         * Mitad de la longitud del robot en el eje Y (pulgadas).
         */
        private static final double yLenght = 29.5 / 2;
    
        //Enfrente Izquierda
        
        /**
         * ID del motor de propulsión (drive) de la rueda delantera izquierda.
         */
        public static final int kFrontLeftDriveMotorId = 10;
    
        /**
         * ID del motor de dirección (steer) de la rueda delantera izquierda.
         */
        public static final int kFrontLeftSteerMotorId = 6;
    
        /**
         * ID del CANCoder para la rueda delantera izquierda.
         */
        public static final int kFrontLeftEncoderId = 2;
    
        /**
         * Offset (rotaciones) para el CANCoder de la rueda delantera izquierda.
         */
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.20947265625);
    
        /**
         * Indica si el motor de dirección (steer) de la rueda delantera izquierda está invertido.
         */
        public static final boolean kFrontLeftSteerMotorInverted = true;
    
        /**
         * Indica si el CANCoder de la rueda delantera izquierda está invertido.
         */
        public static final boolean kFrontLeftEncoderInverted = false;
    
        /**
         * Posición en X (distancia desde el centro del robot) de la rueda delantera izquierda.
         */
        public static final Distance kFrontLeftXPos = Inches.of(xLenght);
    
        /**
         * Posición en Y (distancia desde el centro del robot) de la rueda delantera izquierda.
         */
        public static final Distance kFrontLeftYPos = Inches.of(yLenght);
    
        //Enfrente Derecha
    
        /**
         * ID del motor de propulsión (drive) de la rueda delantera derecha.
         */
        public static final int kFrontRightDriveMotorId = 9;
    
        /**
         * ID del motor de dirección (steer) de la rueda delantera derecha.
         */
        public static final int kFrontRightSteerMotorId = 5;
    
        /**
         * ID del CANCoder para la rueda delantera derecha.
         */
        public static final int kFrontRightEncoderId = 1;
    
        /**
         * Offset (rotaciones) para el CANCoder de la rueda delantera derecha.
         */
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.1787109375);
    
        /**
         * Indica si el motor de dirección (steer) de la rueda delantera derecha está invertido.
         */
        public static final boolean kFrontRightSteerMotorInverted = true;
    
        /**
         * Indica si el CANCoder de la rueda delantera derecha está invertido.
         */
        public static final boolean kFrontRightEncoderInverted = false;
    
        /**
         * Posición en X (distancia desde el centro del robot) de la rueda delantera derecha.
         */
        public static final Distance kFrontRightXPos = Inches.of(xLenght);
    
        /**
         * Posición en Y (distancia desde el centro del robot) de la rueda delantera derecha.
         */
        public static final Distance kFrontRightYPos = Inches.of(-yLenght);
    
        //Atras Izquierda
        
        /**
         * ID del motor de propulsión (drive) de la rueda trasera izquierda.
         */
        public static final int kBackLeftDriveMotorId = 11;
    
        /**
         * ID del motor de dirección (steer) de la rueda trasera izquierda.
         */
        public static final int kBackLeftSteerMotorId = 7;
    
        /**
         * ID del CANCoder para la rueda trasera izquierda.
         */
        public static final int kBackLeftEncoderId = 3;
    
        /**
         * Offset (rotaciones) para el CANCoder de la rueda trasera izquierda.
         */
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.220947265625);
    
        /**
         * Indica si el motor de dirección (steer) de la rueda trasera izquierda está invertido.
         */
        public static final boolean kBackLeftSteerMotorInverted = true;
    
        /**
         * Indica si el CANCoder de la rueda trasera izquierda está invertido.
         */
        public static final boolean kBackLeftEncoderInverted = false;
    
        /**
         * Posición en X (distancia desde el centro del robot) de la rueda trasera izquierda.
         */
        public static final Distance kBackLeftXPos = Inches.of(-xLenght);
    
        /**
         * Posición en Y (distancia desde el centro del robot) de la rueda trasera izquierda.
         */
        public static final Distance kBackLeftYPos = Inches.of(yLenght);
    
        //Atraz Derecha
        
        /**
         * ID del motor de propulsión (drive) de la rueda trasera derecha.
         */
        public static final int kBackRightDriveMotorId = 12;
    
        /**
         * ID del motor de dirección (steer) de la rueda trasera derecha.
         */
        public static final int kBackRightSteerMotorId = 8;
    
        /**
         * ID del CANCoder para la rueda trasera derecha.
         */
        public static final int kBackRightEncoderId = 4;
    
        /**
         * Offset (rotaciones) para el CANCoder de la rueda trasera derecha.
         */
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.3515625);
    
        /**
         * Indica si el motor de dirección (steer) de la rueda trasera derecha está invertido.
         */
        public static final boolean kBackRightSteerMotorInverted = true;
    
        /**
         * Indica si el CANCoder de la rueda trasera derecha está invertido.
         */
        public static final boolean kBackRightEncoderInverted = false;
    
        /**
         * Posición en X (distancia desde el centro del robot) de la rueda trasera derecha.
         */
        public static final Distance kBackRightXPos = Inches.of(-xLenght);
    
        /**
         * Posición en Y (distancia desde el centro del robot) de la rueda trasera derecha.
         */
        public static final Distance kBackRightYPos = Inches.of(-yLenght);
    
    }    
    
    public class ElevatorConstants {
    
        public static final double SproketDiameterInches = 2 ;
        
        public static final double KP = 0.5;
        
        public static final double KI = 0;
        
        public static final double KD = 0 ;
        
        public static final double OffSetMeters = 0.98;

        public static final double TOLERANCE = 0.05;

        public static final int Motor1ID = 0;

        public static final int Motor2ID = 0;

        public static final int ElevatorVelocity = 0;
        
    }
    
    public class CoralConstants{

        public static final int motor1ID = 0;
        public static final int motor2ID = 0;
        public static final int pivotMotorID = 0;
        public static final boolean twoMotorsIsActive = false;
        public static final double pivotMotorVelocity = 0;
        public static final double MotorsIntakeVelocity = 0;

        

    }
    
    public class AlgaeConstants {
    
    
        
    }    
    
    public class DeepCageConstants {
    
        
    }
    
    //Constantes de la LimeLight
    public static final class LimeConstants {

        public static final double cameraHeight = 20; 
        public static final double cameraAngle = 20.5;

        //La distancia del Speaker que debe que estar el robot
        public static final double kDistanceToSpeaker = 70;

        //La distnacia del robot al Amplificador
        public static final double kDistanceToAmp = 45;

        //La altura de la April Tag del Speaker
        public static final double kTargetSpeakertHeight = 54; 

        //La latura de la April Tag del Amp
        public static final double kTargetAmpHeight = 51.625; // Modiffy 58.375      
    }
}
