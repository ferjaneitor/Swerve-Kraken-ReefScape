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
 * - IntakeMotor y motor2: Controlan la succión, cada uno girando en sentidos opuestos.
 * - pivotMotor: Controla la inclinación o posición angular del intake.
 * - pivotMotorEncoder: Mide la posición del motor de pivote.
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
    private SparkMax IntakeMotor;
    /** Motor que controla el ángulo o pivote del sistema de intake. */
    private SparkMax pivotMotor;

    /** Encoder relativo del motor de pivote. */
    private RelativeEncoder pivotMotorEncoder;

    private PIDController PivotpidController;

    /**
     * Constructor del subsistema Algea. Inicializa los motores para la succión
     * y el motor/encoder del pivote con los IDs configurados en AlgaeConstants.
     */
    public AlgaeSubSystem() {
        this.IntakeMotor = new SparkMax(AlgaeConstants.IntakeMotorID, MotorType.kBrushless);

        this.pivotMotor = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless);
        this.pivotMotorEncoder = pivotMotor.getEncoder();
    
        this.PivotpidController = new PIDController(AlgaeConstants.kp, AlgaeConstants.KI, AlgaeConstants.KD);
    }

    /**
     * Activa los motores de succión con la velocidad especificada.
     * Se invierte el segundo motor para que ambos trabajen en sentidos opuestos.
     *
     * @param Velocity Velocidad a aplicar (rango típico: -1.0 a 1.0).
     */
    public void enableAlgaeIntake(double Velocity) {
        IntakeMotor.set(Velocity);
    }

    /**
     * Detiene los motores de succión, estableciendo sus velocidades en 0.
     */
    public void stopAlgaeIntake() {
        IntakeMotor.set(0);
    }

    /**
     * Detiene los motores de pivote, estableciendo sus velocidades en 0.
     */
    public void stopPivot() {
        pivotMotor.set(0);
        
    }

    /**
     * Aplica la velocidad dada al motor de pivote, ajustando el ángulo del mecanismo de intake.
     *
     * @param Velocity Velocidad a aplicar (rango: -1.0 a 1.0).
     */
    public void enablePivot(double Velocity) {
        pivotMotor.set(Velocity);
    }

    /**
     * Devuelve la posición actual del encoder del pivote (en rotaciones).
     *
     * @return Valor del encoder del motor de pivote.
     */
    public double getPosition() {
        return pivotMotorEncoder.getPosition();
    }

    /**
     * Detiene todos los motores (de succión y de pivote), estableciendo sus velocidades en 0.
     */
    public void stopAll() {
        IntakeMotor.set(0);
        pivotMotor.set(0);
        
    }

    /**
     * Reinicia (pone a cero) la posición de los encoders del pivote.
     */
    public void resetEncoders() {
        pivotMotorEncoder.setPosition(0);
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
        double finalOutput1 = PivotpidController.calculate(pivotMotorEncoder.getPosition(), anglesToRotations(angle));
        pivotMotor.set(finalOutput1);
        
    }

    /**
     * Reinicia la posición del mecanismo del intake utilizando PID.
     */
    public void resetPosition() {
        double finalOutput = PivotpidController.calculate(pivotMotorEncoder.getPosition(), 0);
        pivotMotor.set(finalOutput);
    }  

    /**
     * se encarga de asignar velocidad al motor derecho del sistema de pivote
     */
    public void EnableRightAlgaePivot( double SetVelocity ){
        
        pivotMotor.set(SetVelocity);

    }
    
}