package frc.robot.DeepCage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DeepCageConstants;

/**
 * @code DeepCageCmd es un comando continuo que controla los motores
 * del subsistema DeepCage a una velocidad fija, determinada por 
 * {@link DeepCageConstants#deepCageVelocity} y una bandera de inversión
 * {@code isInverted}.</p>
 *
 * <ul>
 *   <li>En {@code initialize()}, se calcula la velocidad final multiplicando
 *       la velocidad base por -1 si {@code isInverted} es true.</li>
 *   <li>En {@code execute()}, se activan los motores del subsistema con la
 *       velocidad calculada.</li>
 *   <li>En {@code end()}, se detienen los motores.</li>
 *   <li>{@code isFinished()} siempre retorna {@code false}, por lo que este
 *       comando no finaliza de forma autónoma.</li>
 * </ul>
 *
 * @author:  Fernando Joel Cruz Briones</p>
 * @Versión: 1.0</p>
 */
public class DeepCageCmd extends Command {

    /**
     * Indica si la dirección de los motores se invierte.
     */
    private boolean isInverted;

    /**
     * Velocidad final calculada a partir de la constante y la inversión.
     */
    private double finalVelocity;

    /**
     * Subsistema que agrupa la lógica y los motores de DeepCage.
     */
    private DeepCageSubSystem deepCageSubSystem;

    /**
     * Crea un nuevo comando para controlar el subsistema DeepCage.
     *
     * @param isInverted       {@code true} para invertir el sentido de los motores.
     * @param deepCageSubSystem Subsistema a controlar.
     */
    public DeepCageCmd(boolean isInverted, DeepCageSubSystem deepCageSubSystem) {
        this.deepCageSubSystem = deepCageSubSystem;
        this.isInverted = isInverted;
    }

    /**
     * Calcula la velocidad final de los motores según la constante
     * {@link DeepCageConstants#deepCageVelocity} y la bandera {@code isInverted}.
     */
    @Override
    public void initialize() {
        finalVelocity = DeepCageConstants.deepCageVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Llamado repetidamente mientras el comando está activo. Aplica
     * la velocidad a los motores de DeepCage.
     */
    @Override
    public void execute() {
        deepCageSubSystem.enableMotors(finalVelocity);
    }

    /**
     * Se llama cuando el comando finaliza o es interrumpido,
     * deteniendo los motores del subsistema.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        deepCageSubSystem.stopMotors();
    }

    /**
     * Retorna siempre {@code false}, este comando no finaliza automáticamente.
     *
     * @return {@code false}.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
