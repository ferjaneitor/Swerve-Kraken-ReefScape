package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;

/**
 * ElevatorResetPosition es un comando que controla el comportamiento del elevador utilizando un ElevatorSubSystem.
 * Su función principal es mover el elevador hasta la altura objetivo y alternar entre los estados de "extender" y "retraer".
 * 
 * Detalles:
 * - Si el sistema está en modo de retracción, la altura objetivo se fuerza a 0 (retrae por completo).
 * - La altura objetivo se convierte a rotaciones mediante el método Meters2Rotations().
 * - Se revisa en isFinished() cuando las posiciones de los motores están cerca de la meta.
 * - Al finalizar, se detienen los motores y se alterna el modo de retracción para la próxima ejecución.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class ElevatorResetPosition extends Command {

    /**
     * Subsistema de elevador que gestiona la lógica y el hardware.
     */
    private ElevatorSubSystem elevatorSubSystem;

    /**
     * Crea una nueva instancia de ElevatorResetPosition.
     *
     * @param elevatorSubSystem Referencia al subsistema del elevador que se controlará.
     */
    public ElevatorResetPosition(ElevatorSubSystem elevatorSubSystem) {
        this.elevatorSubSystem = elevatorSubSystem;
        elevatorSubSystem.ResetEncoders();
        addRequirements(elevatorSubSystem);
    }

    /**
     * Se ejecuta una sola vez al inicio del comando.
     * Si el elevador está en modo retracción, se fuerza la meta a 0 metros y se convierte la meta a rotaciones.
     */
    @Override
    public void initialize() {
        // No se requiere lógica adicional en initialize.
        elevatorSubSystem.changeRunningCmd(true);
    }

    /**
     * Llamado repetidamente mientras el comando está en ejecución.
     * Envía la meta en rotaciones al subsistema para que ajuste la altura del elevador.
     */
    @Override
    public void execute() {
        elevatorSubSystem.resetPosition();
    }

    /**
     * Se llama cuando el comando termina o es interrumpido.
     * Detiene los motores del elevador y alterna el estado entre extender y retraer para la próxima ejecución.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubSystem.stopMotors();
        // Alterna el estado entre extender y retraer
        elevatorSubSystem.changeRunningCmd(false);
    }

    /**
     * Determina si el comando ha finalizado.
     * Comprueba si ambos motores del elevador han alcanzado la posición objetivo dentro del rango de tolerancia.
     *
     * @return true si el elevador llegó a la posición deseada, false en caso contrario.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubSystem.getRightMotorPosition()) < ElevatorConstants.TOLERANCE &&
               Math.abs(elevatorSubSystem.getLeftMotorPosition()) < ElevatorConstants.TOLERANCE;
    }
}
