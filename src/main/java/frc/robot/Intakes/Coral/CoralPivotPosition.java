package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * CoralPivotPosition es un comando que resetea la posición del pivote del mecanismo Coral.
 * 
 * Descripción:
 * - En execute(), se llama al método resetPosition() del subsistema Coral para ajustar el pivote.
 * - En end(), se detiene el pivote mediante stopPivot().
 * - El comando finaliza cuando la posición del pivote es menor que la tolerancia definida en 
 *   AlgaeConstants.TOLERANCE.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class CoralPivotPosition extends Command {

    private CoralSubSystem coralSubSystem;
    
    private double finalAngleDeg;
    
    /**
     * Construye el comando para resetear la posición del pivote del subsistema Coral.
     *
     * @param coralSubSystem Subsistema de Coral que controla el mecanismo.
     */
    public CoralPivotPosition(double angleDeg ,CoralSubSystem coralSubSystem) {
        this.coralSubSystem = coralSubSystem;
        this.finalAngleDeg = angleDeg;
        addRequirements(coralSubSystem);
    }
    
    @Override
    public void initialize() {
        // No se requiere inicialización.
    }

    @Override
    public void execute() {
        coralSubSystem.setPivot2Angle(finalAngleDeg);
    }

    @Override
    public void end(boolean interrupted) {
        coralSubSystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(coralSubSystem.getPivotPosition() - finalAngleDeg) < AlgaeConstants.TOLERANCE;
    }
}
