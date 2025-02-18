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
 * CoralSubSystem es el subsistema que controla el mecanismo de "Coral", encargado de la succión (intake)
 * y del pivote relacionado a este mecanismo. Permite operar con uno o dos motores de succión (según la
 * configuración de CoralConstants.twoMotorsIsActive), así como ajustar la velocidad del pivote.
 *
 * Motores:
 * - motor1, motor2: Motores de succión; motor2 puede estar deshabilitado si twoMotorsIsActive es false.
 * - pivotMotor: Motor que controla la inclinación o posición del mecanismo Coral.
 *
 * Métodos destacados:
 * - enableCoralIntake(double Velocity): Activa la succión con la velocidad especificada, invirtiendo uno de
 *   los motores si hay dos disponibles.
 * - stopCoralIntake(): Detiene los motores de succión.
 * - enablePivot(double Velocity): Ajusta el pivote del Coral con la velocidad dada.
 * - stopPivot(): Detiene el motor del pivote.
 * - stopAll(): Detiene tanto la succión como el pivote.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.4
 */
public class CoralSubSystem extends SubsystemBase {

    /** Motor principal de succión. */
    private SparkMax motor1;

    /** Motor que controla el pivote del mecanismo Coral. */
    private SparkMax pivotMotor;

    // Encoder relativo del motor que pivota el mecanismo del coral.
    private RelativeEncoder pivotmotorEncoder;

    // PID del motor que pivota el mecanismo del coral.
    private PIDController PivotpidController;

    private SparkMaxConfig pivotMaxConfig;

    /**
     * Crea una nueva instancia del CoralSubSystem, configurando el número de motores de succión y el motor de pivote
     * según las constantes definidas en CoralConstants.
     */
    public CoralSubSystem() {
        this.motor1 = new SparkMax(CoralConstants.motor1ID, MotorType.kBrushless);
        this.pivotMotor = new SparkMax(CoralConstants.pivotMotorID, MotorType.kBrushless);
        this.pivotmotorEncoder = pivotMotor.getEncoder();
        this.PivotpidController = new PIDController(CoralConstants.KP, CoralConstants.KI, CoralConstants.KD);
        this.pivotMaxConfig = new SparkMaxConfig();
        pivotMaxConfig.encoder.positionConversionFactor(360 / CoralConstants.gearRatio);
        pivotMotor.configure(pivotMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ResetEncoders();
    }

    /**
     * Activa los motores de succión a la velocidad indicada. Si hay dos motores activos, uno de ellos se invierte para
     * girar en sentido opuesto.
     *
     * @param Velocity Velocidad deseada (típicamente de -1.0 a 1.0).
     */
    public void enableCoralIntake(double Velocity) {
        motor1.set(Velocity);
    }

    /**
     * Activa el motor de pivote a la velocidad dada, permitiendo modificar la inclinación del mecanismo.
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
     * Reinicia el encoder relativo del motor de pivote.
     */
    public void ResetEncoders() {
        pivotmotorEncoder.setPosition(0);
    }

    /**
     * Calcula las rotaciones necesarias para alcanzar el ángulo deseado.
     *
     * @param angle Ángulo deseado.
     * @return Rotaciones necesarias para alcanzar el ángulo.
     */
    public double anglesToRotations(Double angle) {
        double rotations = angle / 360;
        rotations *= CoralConstants.gearRatio;
        return rotations;
    }

    /**
     * Con PID, posiciona el mecanismo del coral en el ángulo deseado.
     *
     * @param angle Ángulo deseado.
     */
    public void setPivot2Angle(double angle) {
        double finalOutput = PivotpidController.calculate(pivotmotorEncoder.getPosition(), angle);
        pivotMotor.set(finalOutput);
    }

    /**
     * Reinicia la posición del mecanismo del coral.
     */
    public void resetPosition() {
        double finalOutput = PivotpidController.calculate(pivotmotorEncoder.getPosition(), 0);
        pivotMotor.set(finalOutput);
    }

    /**
     * Retorna la posición actual del pivote.
     *
     * @return Posición del pivote.
     */
    public double getPivotPosition() {
        return pivotmotorEncoder.getPosition();
    }
}
