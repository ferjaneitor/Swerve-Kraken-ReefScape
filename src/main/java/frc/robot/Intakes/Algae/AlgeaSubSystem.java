package frc.robot.Intakes.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;

/**
 * @AlgeaSubSystem es el subsistema que gestiona los motores y el pivote del
 * mecanismo de intake "Algea". Permite activar y desactivar la succión (intake)
 * y controlar la posición/velocidad del pivote.
 *
 * <ul>
 *   <li>motor1 y motor2: Controlan la succión, cada uno girando en sentidos opuestos.</li>
 *   <li>pivotMotor: Controla la inclinación o posición angular del intake.</li>
 *   <li>pivotMotoEncoder: Encargado de medir la posición del motor de pivote.</li>
 * </ul>
 *
 * @Funciones principales:
 * <ul>
 *   <li><strong>enableAlgaeIntake:</strong> Activa el intake con la velocidad dada.</li>
 *   <li><strong>stopAlgaeIntake:</strong> Detiene los motores de succión.</li>
 *   <li><strong>enablePivot:</strong> Aplica la velocidad al pivote.</li>
 *   <li><strong>stopPivot:</strong> Detiene el motor de pivote.</li>
 *   <li><strong>resetEncoders:</strong> Reinicia la posición del encoder del pivote a 0.</li>
 *   <li><strong>stopAll:</strong> Detiene todos los motores (succión y pivote).</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class AlgeaSubSystem extends SubsystemBase {

    /** Motor 1 para la succión (intake). */
    private SparkMax motor1;
    /** Motor 2 para la succión (intake), girando en sentido inverso a motor1. */
    private SparkMax motor2;
    /** Motor que controla el ángulo o pivote del sistema de intake. */
    private SparkMax pivotMotor;

    /** Encoder relativo del motor de pivote. */
    private RelativeEncoder pivotMotoEncoder;

    /**
     * Constructor del subsistema Algea. Inicializa los motores para la succión
     * y el motor/encoder del pivote con sus IDs configurados en
     * {@link AlgaeConstants}.
     */
    public AlgeaSubSystem() {
        this.motor1 = new SparkMax(AlgaeConstants.motor1ID, MotorType.kBrushless);
        this.motor2 = new SparkMax(AlgaeConstants.motor2ID, MotorType.kBrushless);

        this.pivotMotor = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless);
        this.pivotMotoEncoder = pivotMotor.getEncoder();
    }

    /**
     * Activa los motores de succión con la velocidad especificada. 
     * Se invierte el segundo motor para que ambos trabajen en sentidos opuestos.
     *
     * @param Velocity Velocidad a aplicar (rango típico -1.0 a 1.0).
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
     * Detiene el motor de pivote, estableciendo su velocidad en 0.
     */
    public void stopPivot() {
        pivotMotor.set(0);
    }

    /**
     * Aplica la velocidad dada al motor de pivote, ajustando el ángulo 
     * del mecanismo de intake.
     *
     * @param Velocity Velocidad a aplicar (rango -1.0 a 1.0).
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
        return pivotMotoEncoder.getPosition();
    }

    /**
     * Detiene todos los motores (los de succión y el pivote), estableciendo sus
     * velocidades en 0.
     */
    public void stopAll() {
        motor1.set(0);
        motor2.set(0);
        pivotMotor.set(0);
    }

    /**
     * Reinicia (pone a cero) la posición del encoder del pivote.
     */
    public void resetEncoders() {
        pivotMotoEncoder.setPosition(0);
    }
}
