package frc.robot.Intakes.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * AlgaePivotResetPosition es un comando que resetea la posición del pivote
 * del subsistema Algea.
 * 
 * Funciones:
 * - En execute(), se llama al método resetPosition() del subsistema para ajustar el pivote.
 * - En end(), se detiene el pivote mediante stopPivot().
 * - El comando finaliza cuando la posición del pivote es menor que la tolerancia definida en AlgaeConstants.TOLERANCE.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.0
 */
public class AlgaePivotResetPosition extends Command {
    
    private AlgaeSubSystem algeaSubSystem;
    
    /**
     * Construye el comando para resetear la posición del pivote del subsistema Algea.
     *
     * @param algeaSubSystem Subsistema Algea encargado del pivote.
     */
    public AlgaePivotResetPosition(AlgaeSubSystem algeaSubSystem) {
        this.algeaSubSystem = algeaSubSystem;
        addRequirements(algeaSubSystem);
    }
    
    @Override
    public void initialize() {
        // No se requiere inicialización adicional.
    }

    @Override
    public void execute() {
        algeaSubSystem.resetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return algeaSubSystem.getPosition() < AlgaeConstants.TOLERANCE;
    }
}
