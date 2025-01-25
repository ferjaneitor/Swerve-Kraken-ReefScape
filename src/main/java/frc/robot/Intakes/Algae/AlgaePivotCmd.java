package frc.robot.Intakes.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class AlgaePivotCmd extends Command {

    private boolean isInverted;

    private AlgeaSubSystem algeaSubSystem;

    private double finalVelocity;

    public AlgaePivotCmd ( boolean isInverted, AlgeaSubSystem algeaSubSystem) {

        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = isInverted;

    }
    
    @Override
    public void initialize() {
        this.finalVelocity = AlgaeConstants.pivotMotorVelocity * ( isInverted ? -1 : 1 );
    }
    
    @Override
    public void execute(){        
        algeaSubSystem.enablePivot(finalVelocity);
    }
    
    @Override
    public void end(boolean interrupted){
        algeaSubSystem.stopPivot();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
