package frc.robot.DeepCage;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeepCageConstants;

/**
 * DeepCageSubSystem es el subsistema que agrupa la lógica y el control de los motores
 * para el mecanismo "DeepCage". Provee métodos para habilitar los motores a una velocidad
 * dada y para detenerlos.
 *
 * Descripción:
 * - motor1, motor2: Motores que funcionan al unísono para el mecanismo DeepCage.
 * - enableMotors(double velocity): Aplica la misma velocidad a ambos motores.
 * - stopMotors(): Detiene ambos motores (velocidad = 0).
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class DeepCageSubSystem extends SubsystemBase {

    /**
     * Motor 1 del mecanismo DeepCage.
     */
    private SparkMax motor1;

    /**
     * Crea una nueva instancia del subsistema DeepCage, inicializando
     * los motores (SparkMax) a partir de los IDs definidos en DeepCageConstants.
     */
    public DeepCageSubSystem() {
        this.motor1 = new SparkMax(DeepCageConstants.motor1ID, MotorType.kBrushless);
    }

    /**
     * Detiene los motores del subsistema, estableciendo su velocidad a 0.
     */
    public void stopMotors() {
        motor1.set(0);
    }

    /**
     * Habilita los motores a la velocidad especificada, aplicando el mismo
     * valor a cada uno.
     *
     * @param velocity Velocidad deseada para los motores (rango típico: -1 a 1).
     */
    public void enableMotors(double velocity) {
        motor1.set(velocity);
    }
}
