package frc.robot.Intakes.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * @AlgaePivotCmd es un comando continuo que controla la inclinación o pivote
 * del mecanismo de intake (Algae). Permite mover el pivote a una velocidad
 * específica y en una dirección determinada mediante la bandera {@code isInverted}.
 *
 * <ul>
 *   <li>En {@code initialize()}, se calcula la velocidad final según 
 *       {@link AlgaeConstants#pivotMotorVelocity} y el estado {@code isInverted}.</li>
 *   <li>En {@code execute()}, se llama al subsistema para aplicar dicha velocidad
 *       de pivote.</li>
 *   <li>En {@code end()}, se detiene el motor del pivote.</li>
 *   <li>{@code isFinished()} siempre retorna {@code false}, por lo que este comando 
 *       no termina automáticamente, sino que normalmente se interrumpiría 
 *       desde otro comando o evento.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class AlgaePivotCmd extends Command {

    /**
     * Indica si la rotación del pivote se invierte.
     */
    private boolean isInverted;

    /**
     * Subsistema que controla el mecanismo Algea (intake) y sus partes móviles,
     * incluido el pivote.
     */
    private AlgeaSubSystem algeaSubSystem;

    /**
     * Velocidad final calculada para el pivote, basada en la constante
     * {@code pivotMotorVelocity} y la bandera {@code isInverted}.
     */
    private double finalVelocity;

    /**
     * Crea un nuevo comando para ajustar la inclinación/pivote de Algea.
     *
     * @param isInverted    Indica si la dirección del pivote debe invertirse.
     * @param algeaSubSystem Instancia del subsistema Algea a controlar.
     */
    public AlgaePivotCmd(boolean isInverted, AlgeaSubSystem algeaSubSystem) {
        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = isInverted;
    }

    /**
     * Calcula la velocidad final que se aplicará al pivote,
     * multiplicando la constante {@code pivotMotorVelocity}
     * por -1 si {@code isInverted} es true.
     */
    @Override
    public void initialize() {
        this.finalVelocity = AlgaeConstants.pivotMotorVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Llamado repetidamente mientras el comando está activo.
     * Establece la velocidad del pivote en {@code finalVelocity}.
     */
    @Override
    public void execute() {
        algeaSubSystem.enablePivot(finalVelocity);
    }

    /**
     * Se llama al terminar o interrumpir el comando, deteniendo
     * el motor del pivote.
     *
     * @param interrupted Indica si el comando terminó de manera
     *                    normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopPivot();
    }

    /**
     * Retorna false para indicar que este comando no termina por sí solo.
     * Generalmente se interrumpe desde otro comando o cuando el robot
     * cambia de estado.
     *
     * @return false siempre
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
