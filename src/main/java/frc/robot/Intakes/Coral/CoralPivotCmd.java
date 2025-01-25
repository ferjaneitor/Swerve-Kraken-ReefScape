package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class CoralPivotCmd extends Command {
    
    private boolean DirectionInverted;
    private double finalVelocity;
    
    private final CoralSubSystem coralSubSystem;

    public CoralPivotCmd(boolean invertDirection, CoralSubSystem coralSubSystem) {
        this.DirectionInverted = invertDirection;
        this.finalVelocity = CoralConstants.MotorsIntakeVelocity * (DirectionInverted ? -1 : 1);
        this.coralSubSystem = coralSubSystem;
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute(){        
        coralSubSystem.enableCoralIntake(finalVelocity);
    }
    
    @Override
    public void end(boolean interrupted){
        coralSubSystem.disableCoralIntake();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }    
}
