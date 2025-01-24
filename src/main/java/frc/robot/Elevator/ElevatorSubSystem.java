package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

/** 
* @author  Fernando Joel Cruz Briones
* @version 1.0
*/

public class ElevatorSubSystem extends SubsystemBase{
    
    private SparkMax Motor1,Motor2;
    
    private RelativeEncoder motor1Encoder, motor2Encoder;    
    
    private PIDController motor1PidController, motor2PidController;
    
    public ElevatorSubSystem(){
        this.Motor1 = new SparkMax(ElevatorConstants.Motor1ID, MotorType.kBrushless);
        this.Motor2 = new SparkMax(ElevatorConstants.Motor2ID, MotorType.kBrushless);
        
        this.motor1Encoder = Motor1.getEncoder();
        this.motor2Encoder = Motor2.getEncoder();
        
        this.motor1PidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        this.motor2PidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    }
    
    public void ResetEncoders (){
        motor1Encoder.setPosition(0);
        motor2Encoder.setPosition(0);
    }   
    
    public double Meters2Rotations(double DistanceMeters){
        
        //Se calcula el diametro de la llanta de pulgadas a metros
        double wheelDiameterMeters = ElevatorConstants.SproketDiameterInches * 0.0254;
        //Se calcula la circunferencia de la llanta ya en metros
        double wheelCircumferenceMeters = wheelDiameterMeters*Math.PI;

        //Se le quita la diferencia de altura que hay entre la posicion mas baja a la mas alta
        double DistanceMetersWithOffSet = wheelCircumferenceMeters - ElevatorConstants.OffSetMeters;
        
        //Se divide entre 2 porque es la cantidad de niveles moviles que tiene el Elevador en cascada
        double DistanceMetersHalf = DistanceMetersWithOffSet/3;

        //Dividimos la altura entre la circunferencia de la llanta para regresar todas las rotaciones
        return DistanceMetersHalf/wheelCircumferenceMeters;
    }  
    
    public void targetHeightFromRotations(double targetRotations){
        double motor1Output = motor1PidController.calculate(motor1Encoder.getPosition(), targetRotations);
        double motor2Output = motor2PidController.calculate(motor2Encoder.getPosition(), -targetRotations);
        
        Motor1.set(motor1Output);        
        Motor2.set(motor2Output);
    }
    
    public void stopMotors () {
        Motor1.set(0);
        Motor2.set(0);
    }
    
    public double getMotor1Position(){
        return motor1Encoder.getPosition();
    }
    
    public double getMotor2Position(){
        return motor2Encoder.getPosition();
    }
    
    public void setVelocity (double Velocity) {
        Motor1.set(Velocity);
        Motor2.set(-Velocity);
    }
    
}
