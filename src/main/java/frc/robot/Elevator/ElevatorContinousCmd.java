package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class ElevatorContinousCmd extends Command {
    
    private boolean isInverted;
    
    private ElevatorSubSystem elevatorSubSystem;
    
    private double finalVelocity;
    
    public ElevatorContinousCmd(boolean isInverted, ElevatorSubSystem elevatorSubSystem){
        this.isInverted = isInverted;
        this.elevatorSubSystem = elevatorSubSystem;
        
    }
    
    @Override
    public void initialize() {

        finalVelocity = ElevatorConstants.ElevatorVelocity * ( isInverted ? -1 : 1 );

    }
    
    @Override
    public void execute(){        
        elevatorSubSystem.setVelocity(finalVelocity);        
    }
    
    @Override
    public void end(boolean interrupted){
        elevatorSubSystem.stopMotors();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }      
    
}
