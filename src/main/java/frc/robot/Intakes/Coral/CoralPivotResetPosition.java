package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * CoralPivotResetPosition es un comando que resetea la posición del pivote del mecanismo Coral.
 * 
 * Descripción:
 * - En execute(), se llama al método resetPosition() del subsistema Coral para ajustar el pivote.
 * - En end(), se detiene el pivote mediante stopPivot().
 * - El comando finaliza cuando la posición del pivote es menor que la tolerancia definida en 
 *   AlgaeConstants.TOLERANCE.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class CoralPivotResetPosition extends Command {

    private CoralSubSystem coralSubSystem;
    
    /**
     * Construye el comando para resetear la posición del pivote del subsistema Coral.
     *
     * @param coralSubSystem Subsistema de Coral que controla el mecanismo.
     */
    public CoralPivotResetPosition(CoralSubSystem coralSubSystem) {
        this.coralSubSystem = coralSubSystem;
        addRequirements(coralSubSystem);
    }
    
    @Override
    public void initialize() {
        // No se requiere inicialización.
    }

    @Override
    public void execute() {
        coralSubSystem.resetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        coralSubSystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return coralSubSystem.getPivotPosition() < AlgaeConstants.TOLERANCE;
    }
}
