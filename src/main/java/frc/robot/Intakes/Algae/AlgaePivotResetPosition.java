package frc.robot.Intakes.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */

public class AlgaePivotResetPosition extends Command {
    
    AlgaeSubSystem algeaSubSystem ;
    
    public AlgaePivotResetPosition (AlgaeSubSystem algeaSubSystem) {

        this.algeaSubSystem = algeaSubSystem ;
        addRequirements(algeaSubSystem);

    }
    
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        algeaSubSystem.resetPosition();;
    }

    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopPivot();
    }

    @Override
    public boolean isFinished() {
        return (algeaSubSystem.getPosition()) < AlgaeConstants.TOLERANCE;
    }
    
}
