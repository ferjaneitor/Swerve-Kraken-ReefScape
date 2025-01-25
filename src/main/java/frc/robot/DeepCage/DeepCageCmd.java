package frc.robot.DeepCage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DeepCageConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class DeepCageCmd extends Command {
    
    private boolean isInverted;
    
    private double finalVelocity;
    
    private DeepCageSubSystem deepCageSubSystem;
    
    public DeepCageCmd ( boolean isInverted, DeepCageSubSystem deepCageSubSystem ) {

        this.deepCageSubSystem = deepCageSubSystem;

        this.isInverted = isInverted;

    }
    
    @Override
    public void initialize() {

        finalVelocity = DeepCageConstants.deepCageVelocity * ( isInverted ? -1 : 1  );

    }
    
    @Override
    public void execute(){        
        
        deepCageSubSystem.enableMotors(finalVelocity);
        
    }
    
    @Override
    public void end(boolean interrupted){
        
        deepCageSubSystem.stopMotors();  
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
