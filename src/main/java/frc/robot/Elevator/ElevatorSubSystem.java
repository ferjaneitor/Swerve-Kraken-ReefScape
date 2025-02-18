package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

/**
 * ElevatorSubSystem es un subsistema que gestiona dos motores (SparkMax con NEO)
 * para controlar un elevador de tipo cascada. Emplea encoders integrados y control PID,
 * permitiendo mover el elevador a una altura objetivo o a una velocidad constante.
 *
 * Detalles:
 * - Los encoders se reinician al construir el objeto o cuando se llama a ResetEncoders().
 * - El método Meters2Rotations(double) convierte metros a rotaciones, teniendo en cuenta:
 *   el diámetro del sprocket (convertido de pulgadas a metros), la circunferencia,
 *   el offset mínimo (OffSetMeters), el número de etapas (ElevatorStages) y la relación de engranajes (GearRatio).
 * - Se utilizan dos PIDController independientes (uno por cada motor). El método
 *   targetHeightFromRotations(double) calcula la salida PID y establece el voltaje en cada motor.
 * - El método setVelocity(double) permite mover el elevador con una velocidad deseada,
 *   compensando la dirección de cada motor.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.6
 */
public class ElevatorSubSystem extends SubsystemBase {

    // Objeto SparkMax que controla el primer motor del elevador (RightMotor).
    private SparkMax RightMotor;

    // Objeto SparkMax que controla el segundo motor del elevador (LeftMotor).
    private SparkMax LeftMotor;

    // Encoder relativo integrado en el primer motor (RightMotor).
    private RelativeEncoder RightMotorEncoder;

    private SparkMaxConfig config;

    // Encoder relativo integrado en el segundo motor (LeftMotor).
    private RelativeEncoder LeftMotorEncoder;

    // Controlador PID para el primer motor (RightMotor).
    private PIDController RightMotorPidController;

    // Controlador PID para el segundo motor (LeftMotor).
    private PIDController LeftMotorPidController;

    private ProfiledPIDController trapezoidRightProfiledPIDController;
    private ProfiledPIDController trapezoidLeftProfiledPIDController;

    /**
     * Construye el subsistema del elevador, inicializando los motores, encoders
     * y controladores PID. También reinicia los encoders para que partan de posición 0.
     */
    public ElevatorSubSystem() {
        this.RightMotor = new SparkMax(ElevatorConstants.RightMotorID, MotorType.kBrushless);
        this.LeftMotor = new SparkMax(ElevatorConstants.LeftMotorID, MotorType.kBrushless);

        this.RightMotorEncoder = RightMotor.getEncoder();
        this.LeftMotorEncoder = LeftMotor.getEncoder();

        this.RightMotorPidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        this.LeftMotorPidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        
        this.trapezoidRightProfiledPIDController = new ProfiledPIDController(
            ElevatorConstants.KP, 
            ElevatorConstants.KI, 
            ElevatorConstants.KD, 
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAXVelocity, 
                ElevatorConstants.MAXAcceleration
            )
        );
        this.trapezoidLeftProfiledPIDController = new ProfiledPIDController(
            ElevatorConstants.KP, 
            ElevatorConstants.KI, 
            ElevatorConstants.KD, 
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAXVelocity, 
                ElevatorConstants.MAXAcceleration
            )
        );
        
        // Convierte el diámetro del sprocket de pulgadas a centímetros y lo usa para calcular la circunferencia en metros.
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 2.54;
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;
        
        config = new SparkMaxConfig();
        config.encoder.positionConversionFactor((SproketCircumferenceMeters * ElevatorConstants.ElevatorStages) / ElevatorConstants.GearRatio);

        RightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        LeftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        ResetEncoders();    
    }

    /**
     * Reinicia (pone a cero) la posición de ambos encoders.
     * Se puede llamar cada vez que se desee recalibrar la posición de los motores del elevador.
     */
    public void ResetEncoders() {
        RightMotorEncoder.setPosition(0);
        LeftMotorEncoder.setPosition(0);
    }

    /**
     * Convierte una distancia en metros (DistanceMeters) a rotaciones de los motores.
     * Considera el diámetro del sprocket (convertido de pulgadas a metros), la circunferencia,
     * el offset mínimo (OffSetMeters), el número de etapas (ElevatorStages) y la relación de engranajes (GearRatio).
     *
     * @param DistanceMeters Distancia objetivo en metros.
     * @return Número de rotaciones resultante para los motores.
     */
    public double Meters2Rotations(double DistanceMeters) {
        // Convierte el diámetro del sprocket de pulgadas a metros
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 0.0254;
        // Calcula la circunferencia en metros
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;

        // Aplica el offset y divide por el número de etapas del elevador
        double DistanceMetersWithOffSet = SproketCircumferenceMeters - ElevatorConstants.OffSetMeters;
        double DistanceMetersHalf = DistanceMetersWithOffSet / ElevatorConstants.ElevatorStages;

        // Convierte la distancia final en rotaciones, multiplicando por la relación de engranajes
        return (DistanceMetersHalf / SproketCircumferenceMeters) * ElevatorConstants.GearRatio;
    }

    /**
     * Ajusta la altura del elevador según las rotaciones objetivo.
     * Calcula las salidas PID de cada motor (el segundo motor se invierte)
     * y establece el voltaje en cada motor.
     *
     * @param targetMeters Número de rotaciones deseadas (positivo para subir, negativo para bajar).
     */
    public void targetHeightFromRotations(double targetMeters) {
        double RightMotorOutput = RightMotorPidController.calculate(RightMotorEncoder.getPosition(), -(targetMeters - ElevatorConstants.OffSetMeters));
        double LeftMotorOutput = LeftMotorPidController.calculate(LeftMotorEncoder.getPosition(), +(targetMeters - ElevatorConstants.OffSetMeters));

        RightMotor.setVoltage(RightMotorOutput);
        LeftMotor.setVoltage(LeftMotorOutput);
    }

    /**
     * Ejecuta un movimiento con perfil trapezoidal para alcanzar la altura objetivo.
     *
     * @param targetmeters Altura objetivo en rotaciones.
     */
    public void trapezoidalMotionProfeTargetHeight(double targetmeters) {
        double RightMotorOutput = trapezoidRightProfiledPIDController.calculate(RightMotorEncoder.getPosition(), targetmeters);
        double LeftMotorOutput = trapezoidLeftProfiledPIDController.calculate(LeftMotorEncoder.getPosition(), targetmeters);

        RightMotor.setVoltage(RightMotorOutput);
        LeftMotor.setVoltage(LeftMotorOutput);
    }

    /**
     * Detiene los dos motores del elevador estableciendo su velocidad a 0.
     */
    public void stopMotors() {
        RightMotor.set(0);
        LeftMotor.set(0);
    }

    /**
     * Obtiene la posición (en rotaciones) del motor derecho (RightMotor).
     *
     * @return Posición actual del encoder de RightMotor.
     */
    public double getRightMotorPosition() {
        return RightMotorEncoder.getPosition();
    }

    /**
     * Obtiene la posición (en rotaciones) del motor izquierdo (LeftMotor).
     *
     * @return Posición actual del encoder de LeftMotor.
     */
    public double getLeftMotorPosition() {
        return LeftMotorEncoder.getPosition();
    }

    /**
     * Ajusta directamente la velocidad de ambos motores, aplicando una inversión en el segundo
     * para el correcto movimiento en cascada.
     *
     * @param Velocity Velocidad deseada (por ejemplo, entre -1.0 y 1.0).
     */
    public void setVelocity(double Velocity) {
        RightMotor.set(-Velocity);
        LeftMotor.set(Velocity);
    }
    
    /**
     * Aplica automáticamente un PID que reinicia la posición del elevador.
     */
    public void resetPosition() {
        double RightMotorOutput = RightMotorPidController.calculate(RightMotorEncoder.getPosition(), 0);
        double LeftMotorOutput = LeftMotorPidController.calculate(LeftMotorEncoder.getPosition(), 0);

        RightMotor.set(RightMotorOutput);
        LeftMotor.set(LeftMotorOutput);
    }    
}
