package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;

/**
 * @ElevatorCmd es un comando (Command) que controla el comportamiento de un elevador
 * utilizando un {@link ElevatorSubSystem}. Su función principal es mover el elevador
 * hasta la altura objetivo establecida en metros ({@code targetMeters}), y alternar
 * entre estado de “extender” y “retraer”.
 *
 * <ul>
 *   <li>Si {@code isRetracting} está activo, la altura objetivo se fuerza a 0 (retrae por completo).</li>
 *   <li>La altura objetivo se convierte a rotaciones mediante {@code Meters2Rotations()}.</li>
 *   <li>Se revisa en {@code isFinished()} cuando las posiciones de los motores
 *       están cerca de la meta.</li>
 *   <li>Al terminar, se detienen los motores y se alterna el modo de retracción
 *       para la próxima ejecución.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.2
 */
public class ElevatorCmd extends Command {

    /**
     * Distancia objetivo en metros a la que se quiere llevar el elevador.
     */
    private double targetMeters;

    /**
     * Indica si el elevador se encuentra en modo "retraer" (TRUE) o en modo
     * "extender" (FALSE).
     */
    private boolean isRetracting;

    /**
     * Distancia objetivo en rotaciones, calculada a partir de {@link #targetMeters}.
     */
    private double targetRotations;

    /**
     * Subsistema de elevador que gestiona la lógica y el hardware.
     */
    private ElevatorSubSystem elevatorSubSystem;

    /**
     * Crea una nueva instancia de {@code ElevatorCmd}.
     *
     * @param DistanceMeters   Distancia objetivo en metros para el elevador.
     * @param elevatorSubSystem Referencia al subsistema del elevador que se controlará.
     */
    public ElevatorCmd(double DistanceMeters, ElevatorSubSystem elevatorSubSystem) {
        this.targetMeters = DistanceMeters;
        this.isRetracting = false; // Inicialmente no está en modo retracción
        this.elevatorSubSystem = elevatorSubSystem;
        elevatorSubSystem.ResetEncoders();
    }

    /**
     * Se ejecuta una sola vez al inicio del comando. Si el elevador está
     * en modo retracción ({@code isRetracting == true}), se fuerza la meta
     * a 0 metros. Después, se convierte la meta en rotaciones.
     */
    @Override
    public void initialize() {
        targetRotations = elevatorSubSystem.Meters2Rotations(targetMeters);
    }

    /**
     * Llamado repetidamente mientras el comando está en ejecución.
     * Envía la meta en rotaciones al subsistema para que éste ajuste
     * la altura del elevador.
     */
    @Override
    public void execute() {
    
        if (isRetracting) {
            // Si está en modo retracción, el objetivo se retrae
            elevatorSubSystem.resetPosition();
        } else {
            elevatorSubSystem.targetHeightFromRotations(targetRotations);
        }
    
    }

    /**
     * Se llama cuando el comando termina o es interrumpido.
     * Detiene los motores y alterna el estado entre extender y retraer
     * para la próxima ejecución.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        elevatorSubSystem.stopMotors();
        // Alterna el estado entre extender y retraer
        isRetracting = !isRetracting;
    }

    /**
     * Determina si el comando ha finalizado. Comprueba si ambos motores
     * del elevador han alcanzado el rango de tolerancia cerca de la
     * posición objetivo.
     *
     * @return TRUE si el elevador llegó a la posición deseada, FALSE en caso contrario.
     */
    @Override
    public boolean isFinished() {
        // Terminar cuando ambos motores alcancen la posición objetivo
        return Math.abs(elevatorSubSystem.getMotor1Position() - elevatorSubSystem.Meters2Rotations(targetMeters))
                   < ElevatorConstants.TOLERANCE
               &&
               Math.abs(elevatorSubSystem.getMotor2Position() + elevatorSubSystem.Meters2Rotations(targetMeters))
                   < ElevatorConstants.TOLERANCE;
    }
}
