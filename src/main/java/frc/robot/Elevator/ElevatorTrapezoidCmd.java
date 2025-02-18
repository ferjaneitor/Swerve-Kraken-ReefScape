package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.Intakes.Coral.CoralSubSystem;

/**
 * ElevatorTrapezoidCmd es un comando que mueve el elevador utilizando un perfil trapezoidal
 * para alcanzar una altura objetivo y, al mismo tiempo, ajustar el ángulo del pivote del subsistema Coral.
 *
 * Funciones:
 * - En execute(), se llama a trapezoidalMotionProfeTargetHeight del subsistema del elevador para moverlo
 *   a la altura deseada, y se ajusta el pivote con setPivot2Angle del subsistema Coral.
 * - El comando finaliza cuando ambos motores del elevador y el pivote alcanzan sus posiciones objetivo
 *   dentro de la tolerancia definida.
 * - En end(), se detienen los motores del elevador.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class ElevatorTrapezoidCmd extends Command {
    
    private double targetMeters;
    private double targetAngleDeg;
    private ElevatorSubSystem elevatorSubSystem;
    private CoralSubSystem coralSubSystem;

    /**
     * Crea un nuevo comando para mover el elevador utilizando un perfil trapezoidal.
     *
     * @param DistanceMeters    Altura objetivo en metros para el elevador.
     * @param angleDeg          Ángulo objetivo en grados para el pivote del subsistema Coral.
     * @param elevatorSubSystem Subsistema del elevador.
     * @param coralSubSystem    Subsistema Coral para el control del pivote.
     */
    public ElevatorTrapezoidCmd(double DistanceMeters, double angleDeg, ElevatorSubSystem elevatorSubSystem, CoralSubSystem coralSubSystem) {
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
        // No se requiere acción en initialize.
    }

    @Override
    public void execute() {
        elevatorSubSystem.trapezoidalMotionProfeTargetHeight(targetMeters);
        coralSubSystem.setPivot2Angle(targetAngleDeg);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubSystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubSystem.getRightMotorPosition() + (targetMeters - ElevatorConstants.OffSetMeters))
                   < ElevatorConstants.TOLERANCE
               && Math.abs(elevatorSubSystem.getLeftMotorPosition() - (targetMeters - ElevatorConstants.OffSetMeters))
                   < ElevatorConstants.TOLERANCE
               && Math.abs(coralSubSystem.getPivotPosition() - targetAngleDeg)
                   < CoralConstants.TOLERANCE;
    }
}
