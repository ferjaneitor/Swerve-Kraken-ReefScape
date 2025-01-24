package frc.robot.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.1
*/

public class ElevatorCmd extends Command {

    private double targetMeters;
    
    private boolean isRetracting;
    
    private double targetRotations ;
    
    private ElevatorSubSystem elevatorSubSystem;
    
    public ElevatorCmd(double DistanceMeters, ElevatorSubSystem elevatorSubSystem){
        
        this.targetMeters = DistanceMeters;
        
        this.isRetracting = false; // Inicialmente no está en modo retracción
        elevatorSubSystem.ResetEncoders();
        
    }
    
    @Override
    public void initialize() {
        if (isRetracting) {
            // Si está en modo retracción, el objetivo es 0 metros
            targetMeters = 0;
        }
        targetRotations = elevatorSubSystem.Meters2Rotations(targetMeters);
    }
    
    @Override
    public void execute(){        
            
        elevatorSubSystem.targetHeightFromRotations(targetRotations);

    }
    
    @Override
    public void end(boolean interrupted){
        elevatorSubSystem.stopMotors();
        
        // Alterna el estado entre extender y retraer
        isRetracting = !isRetracting;
    }
    
    @Override
    public boolean isFinished() {
        // Terminar cuando ambos motores alcancen la posición objetivo
        return Math.abs(elevatorSubSystem.getMotor1Position() - elevatorSubSystem.Meters2Rotations(targetMeters)) < ElevatorConstants.TOLERANCE &&
               Math.abs(elevatorSubSystem.getMotor2Position() + elevatorSubSystem.Meters2Rotations(targetMeters)) < ElevatorConstants.TOLERANCE;
    }
    
    
    
}
