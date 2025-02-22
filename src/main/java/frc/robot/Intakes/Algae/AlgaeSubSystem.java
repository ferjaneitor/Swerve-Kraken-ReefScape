package frc.robot.Intakes.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;
import frc.robot.constants.CoralConstants;

/**
 * AlgeaSubSystem es el subsistema que gestiona los motores y el pivote del mecanismo
 * de intake "Algea". Permite activar y desactivar la succión (intake) y controlar la
 * posición y velocidad del pivote.
 *
 * Elementos:
 * - motor1 y motor2: Controlan la succión, cada uno girando en sentidos opuestos.
 * - pivotMotor1: Controla la inclinación o posición angular del intake.
 * - pivotMotor1Encoder: Mide la posición del motor de pivote.
 *
 * Funciones principales:
 * - enableAlgaeIntake: Activa el intake con la velocidad dada.
 * - stopAlgaeIntake: Detiene los motores de succión.
 * - enablePivot: Aplica la velocidad al pivote.
 * - stopPivot: Detiene el motor de pivote.
 * - resetEncoders: Reinicia la posición del encoder del pivote a 0.
 * - stopAll: Detiene todos los motores (succión y pivote).
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class AlgaeSubSystem extends SubsystemBase {

    /** Motor 1 para la succión (intake). */
    private SparkMax motor1;
    /** Motor 2 para la succión (intake), girando en sentido inverso a motor1. */
    private SparkMax motor2;
    /** Motor que controla el ángulo o pivote del sistema de intake. */
    private SparkMax pivotMotor1;
    private SparkMax pivotMotor2;

    /** Encoder relativo del motor de pivote. */
    private RelativeEncoder pivotMotor1Encoder;
    private RelativeEncoder pivotMotor2Encoder;

    private PIDController PivotpidController;

    /**
     * Constructor del subsistema Algea. Inicializa los motores para la succión
     * y el motor/encoder del pivote con los IDs configurados en AlgaeConstants.
     */
    public AlgaeSubSystem() {
        this.motor1 = new SparkMax(AlgaeConstants.motor1ID, MotorType.kBrushless);
        this.motor2 = new SparkMax(AlgaeConstants.motor2ID, MotorType.kBrushless);

        this.pivotMotor1 = new SparkMax(AlgaeConstants.pivotMotor1ID, MotorType.kBrushless);
        this.pivotMotor2 = new SparkMax(AlgaeConstants.pivotMotor2ID, MotorType.kBrushless);
        this.pivotMotor1Encoder = pivotMotor1.getEncoder();
        this.pivotMotor2Encoder = pivotMotor2.getEncoder();
    
        this.PivotpidController = new PIDController(AlgaeConstants.kp, AlgaeConstants.KI, AlgaeConstants.KD);
    }

    /**
     * Activa los motores de succión con la velocidad especificada.
     * Se invierte el segundo motor para que ambos trabajen en sentidos opuestos.
     *
     * @param Velocity Velocidad a aplicar (rango típico: -1.0 a 1.0).
     */
    public void enableAlgaeIntake(double Velocity) {
        motor1.set(Velocity);
        motor2.set(-Velocity);
    }

    /**
     * Detiene los motores de succión, estableciendo sus velocidades en 0.
     */
    public void stopAlgaeIntake() {
        motor1.set(0);
        motor2.set(0);
    }

    /**
     * Detiene los motores de pivote, estableciendo sus velocidades en 0.
     */
    public void stopPivot() {
        pivotMotor1.set(0);
        pivotMotor2.set(0);
    }

    /**
     * Aplica la velocidad dada al motor de pivote, ajustando el ángulo del mecanismo de intake.
     *
     * @param Velocity Velocidad a aplicar (rango: -1.0 a 1.0).
     */
    public void enablePivot(double Velocity) {
        pivotMotor1.set(Velocity);
        pivotMotor2.set(-Velocity);
    }

    /**
     * Devuelve la posición actual del encoder del pivote (en rotaciones).
     *
     * @return Valor del encoder del motor de pivote.
     */
    public double getPosition() {
        return pivotMotor1Encoder.getPosition();
    }

    /**
     * Detiene todos los motores (de succión y de pivote), estableciendo sus velocidades en 0.
     */
    public void stopAll() {
        motor1.set(0);
        motor2.set(0);
        pivotMotor1.set(0);
        pivotMotor2.set(0);
    }

    /**
     * Reinicia (pone a cero) la posición de los encoders del pivote.
     */
    public void resetEncoders() {
        pivotMotor1Encoder.setPosition(0);
        pivotMotor2Encoder.setPosition(0);
    }

    /**
     * Calcula las rotaciones necesarias para alcanzar el ángulo deseado.
     *
     * @param angle Ángulo deseado.
     * @return Rotaciones necesarias para alcanzar el ángulo.
     */
    public double anglesToRotations(Double angle) {
        double rotations = angle / 360;
        rotations *= CoralConstants.gearRatio;
        return rotations;
    }   

    /**
     * Posiciona el mecanismo del intake en el ángulo deseado utilizando PID.
     *
     * @param angle Ángulo deseado.
     */
    public void setPivot2Angle(double angle) {
        double finalOutput1 = PivotpidController.calculate(pivotMotor1Encoder.getPosition(), anglesToRotations(angle));
        double finalOutput2 = PivotpidController.calculate(pivotMotor1Encoder.getPosition(), -anglesToRotations(angle));
        pivotMotor1.set(finalOutput1);
        pivotMotor2.set(finalOutput2);
    }

    /**
     * Reinicia la posición del mecanismo del intake utilizando PID.
     */
    public void resetPosition() {
        double finalOutput = PivotpidController.calculate(pivotMotor1Encoder.getPosition(), 0);
        pivotMotor1.set(finalOutput);
        pivotMotor2.set(finalOutput);
    }
    
    public SparkMax getSparkMax (boolean IsRight) {

        if (IsRight) {
            return motor1;
        }else{
            return motor2;
        }

    }    
    
}