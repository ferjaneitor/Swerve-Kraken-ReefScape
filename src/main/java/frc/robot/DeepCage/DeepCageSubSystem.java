package frc.robot.DeepCage;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeepCageConstants;

/**
 * @code DeepCageSubSystem es el subsistema que agrupa la lógica y el control
 * de dos motores (SparkMax) para el mecanismo "DeepCage". Provee métodos para
 * habilitar ambos motores a una velocidad dada y detenerlos.</p>
 *
 * <ul>
 *   <li><strong>motor1, motor2:</strong> Motores que funcionan al unísono para
 *       el mecanismo DeepCage.</li>
 *   <li><strong>enableMotors(double velocity):</strong> Aplica la misma velocidad
 *       a ambos motores.</li>
 *   <li><strong>stopMotors():</strong> Detiene ambos motores (velocidad = 0).</li>
 * </ul>
 *
 * @author:  Fernando Joel Cruz Briones</p>
 * @Versión: 1.0</p>
 */
public class DeepCageSubSystem extends SubsystemBase {

    /**
     * Motor 1 del mecanismo DeepCage.
     */
    private SparkMax motor1;

    /**
     * Motor 2 del mecanismo DeepCage.
     */
    private SparkMax motor2;

    /**
     * Crea una nueva instancia del subsistema DeepCage, inicializando
     * los dos motores (SparkMax) a partir de los IDs definidos en
     * {@link DeepCageConstants}.
     */
    public DeepCageSubSystem() {
        this.motor1 = new SparkMax(DeepCageConstants.motor1ID, MotorType.kBrushless);
    }

    /**
     * Detiene ambos motores del subsistema, estableciendo su velocidad a 0.
     */
    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
    }

    /**
     * Habilita los motores a la velocidad especificada, aplicando el mismo
     * valor a ambos.
     *
     * @param velocity Velocidad deseada para los motores (rango típico: -1 a 1).
     */
    public void enableMotors(double velocity) {
        motor1.set(velocity);
    }
}
