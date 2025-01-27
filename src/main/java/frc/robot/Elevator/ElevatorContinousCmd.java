package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;

/**
 * @ElevatorContinousCmd es un comando continuo (sin finalización automática)
 * que mueve el elevador a una determinada velocidad. Permite invertir la
 * dirección de movimiento según sea necesario.
 *
 * <ul>
 *   <li>Si {@code isInverted} es true, la velocidad se invierte para
 *       bajar o retraer el elevador.</li>
 *   <li>El comando no termina por sí solo, {@code isFinished()} retorna false.</li>
 *   <li>Al finalizar (por interrupción o cancelación), se detienen los motores
 *       llamando a {@code stopMotors()}.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class ElevatorContinousCmd extends Command {

    /**
     * Indica si se invierte la dirección de la velocidad (true = invertir).
     */
    private boolean isInverted;

    /**
     * Subsistema del elevador, encargado de manejar físicamente los motores.
     */
    private ElevatorSubSystem elevatorSubSystem;

    /**
     * Velocidad final calculada según la constante del elevador y la bandera
     * {@code isInverted}.
     */
    private double finalVelocity;

    /**
     * Crea un nuevo comando continuo para el elevador.
     *
     * @param isInverted Bandera para indicar si se debe invertir la dirección.
     * @param elevatorSubSystem Instancia del subsistema de elevador.
     */
    public ElevatorContinousCmd(boolean isInverted, ElevatorSubSystem elevatorSubSystem) {
        this.isInverted = isInverted;
        this.elevatorSubSystem = elevatorSubSystem;
    }

    /**
     * Inicializa el comando, calculando la velocidad final en función de
     * {@code ElevatorConstants.ElevatorVelocity} y la bandera {@code isInverted}.
     */
    @Override
    public void initialize() {
        finalVelocity = ElevatorConstants.ElevatorVelocity * (isInverted ? -1 : 1);
    }

    /**
     * Ejecutado repetidamente mientras el comando está activo. Ajusta la
     * velocidad de los motores del elevador a {@code finalVelocity}.
     */
    @Override
    public void execute() {
        elevatorSubSystem.setVelocity(finalVelocity);
    }

    /**
     * Se llama cuando el comando finaliza o es interrumpido. Detiene
     * los motores del elevador.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubSystem.stopMotors();
    }

    /**
     * Como es un comando continuo, siempre retorna {@code false} para que
     * no finalice automáticamente.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
