package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

/**
 * @ElevatorSubSystem es un subsistema que gestiona dos motores (SparkMax con NEO)
 * para controlar un elevador de tipo cascada. Emplea encoders integrados y
 * control PID, permitiendo mover el elevador a una altura objetivo o una
 * velocidad constante.
 *
 * <ul>
 *   <li>Los encoders se reinician al construir el objeto o cuando se llama a
 *       {@link #ResetEncoders()}.</li>
 *   <li>El método {@link #Meters2Rotations(double)} realiza la conversión de
 *       metros a rotaciones, teniendo en cuenta el diámetro del sprocket, la
 *       circunferencia, la reducción (GearRatio) y la compensación por etapas.</li>
 *   <li>Se utilizan dos PIDController independientes (uno por cada motor).
 *       El método {@link #targetHeightFromRotations(double)} calcula la salida
 *       PID y establece la potencia en cada motor.</li>
 *   <li>{@link #setVelocity(double)} permite mover el elevador con una
 *       velocidad deseada, compensando la dirección de cada motor.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.6
 */
public class ElevatorSubSystem extends SubsystemBase {

    /**
     * Objeto SparkMax que controla el primer motor del elevador (Motor1).
     */
    private SparkMax Motor1;

    /**
     * Objeto SparkMax que controla el segundo motor del elevador (Motor2).
     */
    private SparkMax Motor2;

    /**
     * Encoder relativo integrado en el primer motor (Motor1).
     */
    private RelativeEncoder motor1Encoder;

    SparkMaxConfig config ;

    /**
     * Encoder relativo integrado en el segundo motor (Motor2).
     */
    private RelativeEncoder motor2Encoder;

    /**
     * Controlador PID para el primer motor (Motor1).
     */
    private PIDController motor1PidController;

    /**
     * Controlador PID para el segundo motor (Motor2).
     */
    private PIDController motor2PidController;

    /**
     * Construye el subsistema del elevador, inicializando los motores,
     * encoders y controladores PID. También reinicia los encoders para
     * que partan de posición 0.
     */
    public ElevatorSubSystem() {
        this.Motor1 = new SparkMax(ElevatorConstants.Motor1ID, MotorType.kBrushless);
        this.Motor2 = new SparkMax(ElevatorConstants.Motor2ID, MotorType.kBrushless);

        this.motor1Encoder = Motor1.getEncoder();
        this.motor2Encoder = Motor2.getEncoder();

        this.motor1PidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        this.motor2PidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        
        // Convierte el diámetro del sprocket de pulgadas a centimetros
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 2.54;
        // Calcula la circunferencia en metros
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;
        
        config = new SparkMaxConfig();

        config.encoder.positionConversionFactor((SproketCircumferenceMeters * ElevatorConstants.ElevatorStages ) / ElevatorConstants.GearRatio) ;

        Motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        Motor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        ResetEncoders();    
    }

    /**
     * Reinicia (pone a cero) la posición de ambos encoders.
     * Se puede llamar cada vez que se desee recalibrar la
     * posición de los motores del elevador.
     */
    public void ResetEncoders() {
        motor1Encoder.setPosition(0);
        motor2Encoder.setPosition(0);
    }

    /**
     * Convierte una distancia en metros ({@code DistanceMeters}) a rotaciones
     * de los motores. Considera el diámetro del sprocket (en pulgadas, convertido
     * a metros), la circunferencia, la compensación ({@code OffSetMeters}),
     * el número de etapas ({@code ElevatorStages}) y la relación de engranajes
     * ({@code GearRatio}).
     *
     * @param DistanceMeters Distancia objetivo en metros.
     * @return Número de rotaciones resultante para los motores.
     */
    public double Meters2Rotations(double DistanceMeters) {

        // Convierte el diámetro del sprocket de pulgadas a metros
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 0.0254;
        // Calcula la circunferencia en metros
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;

        // Aplica el offset de la distancia mínima y el número de etapas del elevador
        double DistanceMetersWithOffSet = SproketCircumferenceMeters - ElevatorConstants.OffSetMeters;
        double DistanceMetersHalf = DistanceMetersWithOffSet / ElevatorConstants.ElevatorStages;

        // Convierte la distancia final en rotaciones, multiplicando por la relación de engranajes
        return (DistanceMetersHalf / SproketCircumferenceMeters) * ElevatorConstants.GearRatio;
    }

    /**
     * Ajusta la altura del elevador según las rotaciones objetivo. Se calculan
     * las salidas PID de cada motor (teniendo en cuenta que el segundo debe
     * invertirse) y se establecen en los motores SparkMax.
     *
     * @param targetMeters Cantidad de rotaciones deseadas (positivo para subir, negativo para bajar).
     */
    public void targetHeightFromRotations(double targetMeters) {
        double motor1Output = motor1PidController.calculate(motor1Encoder.getPosition(), -(targetMeters - ElevatorConstants.OffSetMeters));
        double motor2Output = motor2PidController.calculate(motor2Encoder.getPosition(), +(targetMeters - ElevatorConstants.OffSetMeters));

        Motor1.setVoltage(motor1Output);
        Motor2.setVoltage(motor2Output);
    }

    /**
     * Detiene los dos motores del elevador estableciendo su velocidad a 0.
     */
    public void stopMotors() {
        Motor1.set(0);
        Motor2.set(0);
    }

    /**
     * Obtiene la posición (en rotaciones) del motor 1 (Motor1).
     *
     * @return Posición actual del encoder de Motor1.
     */
    public double getMotor1Position() {
        return motor1Encoder.getPosition();
    }

    /**
     * Obtiene la posición (en rotaciones) del motor 2 (Motor2).
     *
     * @return Posición actual del encoder de Motor2.
     */
    public double getMotor2Position() {
        return motor2Encoder.getPosition();
    }

    /**
     * Ajusta directamente la velocidad de ambos motores, aplicando una inversión
     * en el segundo para el correcto movimiento en cascada.
     *
     * @param Velocity Velocidad deseada (de -1.0 a 1.0, por ejemplo).
     */
    public void setVelocity(double Velocity) {
        Motor1.set(-Velocity);
        Motor2.set(Velocity);
    }
    
     /**
     * Aplica automaticamente un pid que se encarga de reiniciar la posicion del elevador
     */
    public void resetPosition () {
        
        double motor1Output = motor1PidController.calculate(motor1Encoder.getPosition(), 0);
        double motor2Output = motor2PidController.calculate(motor2Encoder.getPosition(), 0);

        Motor1.set(motor1Output);
        Motor2.set(motor2Output);

    }    
}
