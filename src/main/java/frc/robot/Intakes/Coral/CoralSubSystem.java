package frc.robot.Intakes.Coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class CoralSubSystem extends SubsystemBase {
    
    private SparkMax motor1,motor2,pivotMotor;
    
    private boolean twoMotorsActive;    
    
    public CoralSubSystem () {
        
        this.twoMotorsActive = CoralConstants.twoMotorsIsActive;
        
        this.motor1 = new SparkMax(CoralConstants.motor1ID, MotorType.kBrushless);
        
        if (twoMotorsActive){
            this.motor2 = new SparkMax(CoralConstants.motor2ID, MotorType.kBrushless);  
        } else {
            this.motor2 = null;
        }

        this.pivotMotor = new SparkMax(CoralConstants.pivotMotorID, MotorType.kBrushless);

    }
    
    public void enableCoralIntake (double Velocity){
        if (twoMotorsActive) {
            motor2.set(-Velocity);
        }
        motor1.set(Velocity);
    }
    
    public void enablePivot ( double Velocity){
        pivotMotor.set(Velocity);
    }

    public void disablePivot(){
        pivotMotor.set(0);
    }
    
    public void disableCoralIntake(){
        if (twoMotorsActive) {
            motor2.set(0);
        }
        motor1.set(0);
    }
    
    public void disableAll() {
        disableCoralIntake();
        disablePivot();
    }
       
    
}
