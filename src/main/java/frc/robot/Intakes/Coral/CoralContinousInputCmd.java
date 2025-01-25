package frc.robot.Intakes.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class CoralContinousInputCmd extends Command {
    
    private final boolean directionInverted;
    private final double intakeVelocity;
    
    private final CoralSubSystem coralSubSystem;

    public CoralContinousInputCmd(boolean invertDirection, CoralSubSystem coralSubSystem) {
        this.directionInverted = invertDirection;
        this.intakeVelocity = CoralConstants.MotorsIntakeVelocity * (directionInverted ? -1 : 1);
        this.coralSubSystem = coralSubSystem;
        addRequirements(this.coralSubSystem);
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute(){        
        coralSubSystem.enableCoralIntake(intakeVelocity);
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
