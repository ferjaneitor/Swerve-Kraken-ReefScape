package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intakes.Coral.CoralSubSystem;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;

/**
 * ElevatorCmd es un comando que controla el comportamiento de un elevador utilizando un ElevatorSubSystem.
 * Su función principal es mover el elevador hasta la altura objetivo establecida en metros (targetMeters)
 * y alternar entre estado de "extender" y "retraer".
 * 
 * Detalles:
 * - Si el sistema está en modo de retracción, la altura objetivo se fuerza a 0 (retrae por completo).
 * - La altura objetivo se convierte a rotaciones mediante Meters2Rotations().
 * - Se revisa en isFinished() cuando las posiciones de los motores están cerca de la meta.
 * - Al finalizar, se detienen los motores y se alterna el modo de retracción para la próxima ejecución.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.5
 */
public class ElevatorCmd extends Command {

    /**
     * Distancia objetivo en metros a la que se quiere llevar el elevador.
     */
    private double targetMeters;

    /**
     * Ángulo objetivo en grados para el pivote del mecanismo.
     */
    private double targetAngleDeg;

    /**
     * Subsistema de elevador que gestiona la lógica y el hardware.
     */
    private ElevatorSubSystem elevatorSubSystem;

    /**
     * Subsistema Coral que se utiliza para controlar el pivote.
     */
    private CoralSubSystem coralSubSystem;

    /**
     * Crea una nueva instancia de ElevatorCmd.
     *
     * @param DistanceMeters    Distancia objetivo en metros para el elevador.
     * @param angleDeg          Ángulo objetivo en grados para el pivote.
     * @param elevatorSubSystem Referencia al subsistema del elevador a controlar.
     * @param coralSubSystem    Referencia al subsistema Coral para controlar el pivote.
     */
    public ElevatorCmd(double DistanceMeters, double angleDeg, ElevatorSubSystem elevatorSubSystem, CoralSubSystem coralSubSystem) {
        this.targetMeters = DistanceMeters;
        this.elevatorSubSystem = elevatorSubSystem;
        elevatorSubSystem.ResetEncoders();
        this.coralSubSystem = coralSubSystem;
        this.targetAngleDeg = angleDeg;
        addRequirements(elevatorSubSystem);
        addRequirements(coralSubSystem);
    }

    @Override
    public void initialize() {
        // No se realiza acción adicional en initialize.
    }

    /**
     * Llamado repetidamente mientras el comando está en ejecución.
     * Envía la meta en rotaciones al subsistema para que ajuste la altura del elevador,
     * y posiciona el pivote en el ángulo deseado.
     */
    @Override
    public void execute() {
        elevatorSubSystem.targetHeightFromRotations(targetMeters);
        coralSubSystem.setPivot2Angle(targetAngleDeg);
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
    }

    /**
     * Determina si el comando ha finalizado.
     * Comprueba si ambos motores del elevador han alcanzado la posición objetivo dentro de la tolerancia,
     * y si el pivote del subsistema Coral está en el ángulo deseado.
     *
     * @return true si el elevador y el pivote han alcanzado sus posiciones objetivo; false en caso contrario.
     */
    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubSystem.getRightMotorPosition() + (targetMeters - ElevatorConstants.OffSetMeters))
                   < ElevatorConstants.TOLERANCE
               &&
               Math.abs(elevatorSubSystem.getLeftMotorPosition() - (targetMeters - ElevatorConstants.OffSetMeters))
                   < ElevatorConstants.TOLERANCE
               &&
               Math.abs(coralSubSystem.getPivotPosition() - targetAngleDeg)
                   < CoralConstants.TOLERANCE;
    }
}
