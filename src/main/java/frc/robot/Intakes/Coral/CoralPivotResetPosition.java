package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */

public class CoralPivotResetPosition extends Command {
    
    CoralSubSystem coralSubSystem;
    
    public CoralPivotResetPosition(CoralSubSystem coralSubSystem){

        this.coralSubSystem = coralSubSystem;
        addRequirements(coralSubSystem);

    }
    
    @Override
    public void initialize() {
        
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
        return (coralSubSystem.getPivotPosition()) < AlgaeConstants.TOLERANCE;
    }
    
}
