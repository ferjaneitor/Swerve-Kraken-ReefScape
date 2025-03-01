package frc.robot.DeepCage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DeepCageConstants;

/**
 * DeepCageCmd es un comando continuo que controla los motores del subsistema DeepCage
 * a una velocidad fija. La velocidad se determina mediante la constante 
 * DeepCageConstants.deepCageVelocity y la bandera de inversión isInverted.
 *
 * Descripción del funcionamiento:
 * - En el método initialize(), se calcula la velocidad final multiplicando la velocidad base
 *   por -1 si isInverted es true.
 * - En el método execute(), se activan los motores del subsistema aplicando la velocidad calculada.
 * - En el método end(), se detienen los motores.
 * - El método isFinished() retorna false, por lo que este comando no finaliza automáticamente.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
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
     * @param isInverted        true para invertir el sentido de los motores.
     * @param deepCageSubSystem Subsistema a controlar.
     */
    public DeepCageCmd(boolean isInverted, DeepCageSubSystem deepCageSubSystem) {
        this.deepCageSubSystem = deepCageSubSystem;
        this.isInverted = isInverted;
        addRequirements(deepCageSubSystem);
    }

    /**
     * Calcula la velocidad final de los motores según la constante
     * DeepCageConstants.deepCageVelocity y la bandera isInverted.
     */
    @Override
    public void initialize() {
        finalVelocity = DeepCageConstants.deepCageVelocity * (isInverted ? -0.6 : 1);
    }

    /**
     * Se ejecuta repetidamente mientras el comando está activo y aplica
     * la velocidad calculada a los motores del subsistema.
     */
    @Override
    public void execute() {
        deepCageSubSystem.enableMotors(finalVelocity);
    }

    /**
     * Se llama cuando el comando finaliza o es interrumpido, deteniendo
     * los motores del subsistema.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        deepCageSubSystem.stopMotors();
    }

    /**
     * Este comando nunca finaliza de forma autónoma.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
