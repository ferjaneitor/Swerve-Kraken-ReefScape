package frc.robot.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.Intakes.Algae.AlgeaSubSystem;
import frc.robot.Intakes.Coral.CoralSubSystem;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class CoralAlgeaContinousIntake extends Command {
    
    private boolean isInverted ;
    
    private AlgeaSubSystem algeaSubSystem;
    
    private CoralSubSystem coralSubSystem;
    
    private double finalVelocity;
    
    public CoralAlgeaContinousIntake ( boolean isInverted, AlgeaSubSystem algeaSubSystem, CoralSubSystem coralSubSystem) {

        this.isInverted = isInverted;

        this.algeaSubSystem = algeaSubSystem;
        this.coralSubSystem = coralSubSystem;

    }
    
    @Override
    public void initialize() {

        finalVelocity = constants.CoralAlgaeFinalVelocity * ( isInverted ? -1 : 1 );

    }
    
    @Override
    public void execute(){        
        
        algeaSubSystem.enableAlgaeIntake(finalVelocity);
        coralSubSystem.enableCoralIntake(finalVelocity);
        
    }
    
    @Override
    public void end(boolean interrupted){
        
        algeaSubSystem.stopAlgaeIntake();
        coralSubSystem.stopCoralIntake();
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
