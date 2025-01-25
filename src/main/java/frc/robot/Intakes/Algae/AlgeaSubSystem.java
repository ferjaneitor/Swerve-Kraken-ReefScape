package frc.robot.Intakes.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class AlgeaSubSystem extends SubsystemBase {
    
    private SparkMax motor1, motor2, pivotMotor;
    
    RelativeEncoder pivotMotoEncoder;
    
    public AlgeaSubSystem () {

        this.motor1 = new SparkMax( AlgaeConstants.motor1ID, MotorType.kBrushless);
        this.motor2 = new SparkMax( AlgaeConstants.motor2ID, MotorType.kBrushless);
        
        this.pivotMotor = new SparkMax( AlgaeConstants.pivotMotorID, MotorType.kBrushless);

        this.pivotMotoEncoder = pivotMotor.getEncoder();

    }
    
    public void enableAlgaeIntake (double Velocity){
        
        motor1.set(Velocity);
        motor2.set(-Velocity);        

    }
    
    public void stopAlgaeIntake () {
        
        motor1.set(0);        
        motor2.set(0);        

    }
    
    public void stopPivot(){
        
        pivotMotor.set(0);        

    }
    
    public void enablePivot (double Velocity) {
        
        pivotMotor.set(Velocity);        

    }
    
    public double getPosition () {
        
        return pivotMotoEncoder.getPosition();
        
    }
    
    public void stopAll () {
        
        motor1.set(0);        
        motor2.set(0);        

        pivotMotor.set(0);

    }
    
    public void resetEncoders(){
        
        pivotMotoEncoder.setPosition(0);        

    }
    
}
