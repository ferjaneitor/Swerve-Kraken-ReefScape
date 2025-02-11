package frc.robot.Intakes.Coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralConstants;

/**
 * @{@code CoralSubSystem} es el subsistema que controla el mecanismo de “Coral”,
 * encargado de la succión (intake) y del pivote relacionado a este mecanismo.
 * Permite operar con uno o dos motores de succión (según la configuración de 
 * {@link CoralConstants#twoMotorsIsActive}), así como ajustar la velocidad 
 * del pivote.
 *
 * <ul>
 *   <li>{@code motor1, motor2}: Motores de succión; {@code motor2} puede estar
 *       deshabilitado si {@code twoMotorsIsActive} es {@code false}.</li>
 *   <li>{@code pivotMotor}: Motor que controla la inclinación o posición
 *       del mecanismo Coral.</li>
 * </ul>
 *
 * @Métodos destacados:
 * <ul>
 *   <li><strong>enableCoralIntake(double Velocity):</strong> Activa la succión 
 *       con la velocidad especificada, invirtiendo uno de los motores si hay dos disponibles.</li>
 *   <li><strong>stopCoralIntake():</strong> Detiene los motores de succión.</li>
 *   <li><strong>enablePivot(double Velocity):</strong> Ajusta el pivote del Coral
 *       con la velocidad dada.</li>
 *   <li><strong>stopPivot():</strong> Detiene el motor del pivote.</li>
 *   <li><strong>stopAll():</strong> Detiene tanto la succión como el pivote.</li>
 * </ul>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.4
 */
public class CoralSubSystem extends SubsystemBase {

    /** Motor principal de succión. */
    private SparkMax motor1;

    /** Motor que controla el pivote del mecanismo Coral. */
    private SparkMax pivotMotor;

    // Encoder Relativo del motor que pivota el mecanismo del coral
    private RelativeEncoder pivotmotorEncoder ;

    //PID del motor que pivota el mecanismo del coral
    private PIDController PivotpidController ;

    private SparkMaxConfig pivotMaxConfig;

    /**
     * Crea una nueva instancia del {@code CoralSubSystem}, configurando
     * el número de motores de succión y el motor de pivote según las constantes
     * definidas en {@link CoralConstants}.
     */
    public CoralSubSystem() {

        this.motor1 = new SparkMax(CoralConstants.motor1ID, MotorType.kBrushless);

        this.pivotMotor = new SparkMax(CoralConstants.pivotMotorID, MotorType.kBrushless);
        
        this.pivotmotorEncoder = pivotMotor.getEncoder();
        
        this.PivotpidController = new PIDController(CoralConstants.KP, CoralConstants.KI, CoralConstants.KD);
        
        this.pivotMaxConfig = new SparkMaxConfig();
        
        pivotMaxConfig.encoder.positionConversionFactor(360/CoralConstants.gearRatio);
        
        pivotMotor.configure(pivotMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        ResetEncoders();
        
    }

    /**
     * Activa los motores de succión a la velocidad indicada. Si hay dos motores
     * activos, uno de ellos se invierte para girar en sentido opuesto.
     *
     * @param Velocity Velocidad deseada (típicamente de -1.0 a 1.0).
     */
    public void enableCoralIntake(double Velocity) {
        motor1.set(Velocity);
    }

    /**
     * Activa el motor de pivote a la velocidad dada, permitiendo modificar
     * la inclinación del mecanismo.
     *
     * @param Velocity Velocidad deseada (típicamente de -1.0 a 1.0).
     */
    public void enablePivot(double Velocity) {
        pivotMotor.set(Velocity);
    }

    /**
     * Detiene el motor de pivote, estableciendo su velocidad en 0.
     */
    public void stopPivot() {
        pivotMotor.set(0);
    }

    /**
     * Detiene los motores de succión, estableciendo su velocidad en 0.
     */
    public void stopCoralIntake() {
        motor1.set(0);
    }

    /**
     * Detiene todos los motores del subsistema (tanto succión como pivote).
     */
    public void stopAll() {
        stopCoralIntake();
        stopPivot();
    }

    /**
     * Se reinician el encoder relativo del pivotmotor
     */
    public void ResetEncoders () {
        pivotmotorEncoder.setPosition(0);
    }
    
    /**
     * @param angle : Angulo deseado
     * 
     * Calcula las rotaciones necesarias para llegar al angulo deseado
     */
    public double anglesToRotations ( Double angle ) {

        double rotations = angle / 360 ;

        rotations *= CoralConstants.gearRatio ;

        return rotations;

    }    

    /**
     * Con PID se posiciona en el angulo deseado el mecanismo del coral
     */
    public void setPivot2Angle ( double angle) {

        double finalOutput = PivotpidController.calculate(pivotmotorEncoder.getPosition(), angle);

        pivotMotor.set(finalOutput);

    }

    /**
     * Se reinicia la posicion del mecanismo del coral
     */
    public void resetPosition () {
    
        double finalOutput = PivotpidController.calculate(pivotmotorEncoder.getPosition(),0);    

        pivotMotor.set(finalOutput);

    }

    public double getPivotPosition() {

        return pivotmotorEncoder.getPosition();

    }

}
