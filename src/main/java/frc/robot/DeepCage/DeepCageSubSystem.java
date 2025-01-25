package frc.robot.DeepCage;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeepCageConstants;;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class DeepCageSubSystem extends SubsystemBase {
    
    private SparkMax motor1,motor2;
    
    public DeepCageSubSystem () {
        
        this.motor1 = new SparkMax(DeepCageConstants.motor1ID, MotorType.kBrushless);
        this.motor2 = new SparkMax(DeepCageConstants.motor2ID, MotorType.kBrushless);

    }   
    
    public void stopMotors () {
        
        motor1.set(0);        
        motor2.set(0);        

    } 
    
    public void enableMotors (double velocity) {
        motor1.set(velocity);
        motor2.set(velocity);
    }
    
}
