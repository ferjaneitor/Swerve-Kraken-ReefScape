package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;

/**
 * @{@code CoralPivotCmd} es un comando continuo que controla el ángulo o pivote
 * de un mecanismo en el subsistema Coral. Ajusta la velocidad del pivote según
 * {@link CoralConstants#MotorsIntakeVelocity} y la bandera de inversión
 * {@code DirectionInverted}.
 *
 * <ul>
 *   <li>En {@code initialize()}, no se realiza ninguna acción específica en este caso.</li>
 *   <li>En {@code execute()}, se manda la velocidad calculada al subsistema 
 *       mediante {@link CoralSubSystem#enableCoralIntake(double)}.</li>
 *   <li>En {@code end()}, se detiene el pivote llamando a 
 *       {@link CoralSubSystem#stopCoralIntake()}.</li>
 *   <li>{@code isFinished()} devuelve siempre {@code false}, por lo que el comando 
 *       no finaliza de forma autónoma.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class CoralPivotCmd extends Command {

    /**
     * Indica si se invierte la dirección del motor para el pivote.
     */
    private boolean DirectionInverted;

    /**
     * Velocidad final calculada para el pivote, basada en 
     * {@link CoralConstants#MotorsIntakeVelocity} y la inversión.
     */
    private double finalVelocity;

    /**
     * Subsistema responsable de la operación de pivote del mecanismo Coral.
     */
    private final CoralSubSystem coralSubSystem;

    /**
     * Crea un nuevo comando para manejar el pivote en el subsistema Coral.
     *
     * @param invertDirection Indica si la dirección del pivote debe invertirse.
     * @param coralSubSystem  Instancia del subsistema Coral.
     */
    public CoralPivotCmd(boolean invertDirection, CoralSubSystem coralSubSystem) {
        this.DirectionInverted = invertDirection;
        this.finalVelocity = CoralConstants.MotorsIntakeVelocity * (DirectionInverted ? -1 : 1);
        this.coralSubSystem = coralSubSystem;
    }

    /**
     * Inicializa el comando. No se realiza ninguna acción adicional.
     */
    @Override
    public void initialize() {
        // Sin acciones adicionales de inicialización.
    }

    /**
     * Se llama repetidamente mientras el comando está activo.
     * Ajusta el pivote al valor de velocidad {@code finalVelocity}.
     */
    @Override
    public void execute() {
        coralSubSystem.enableCoralIntake(finalVelocity);
    }

    /**
     * Se llama al finalizar o interrumpir el comando, deteniendo el pivote.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        coralSubSystem.stopCoralIntake();
    }

    /**
     * El comando nunca finaliza por sí solo, se debe interrumpir desde otra parte.
     *
     * @return Siempre {@code false}.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
