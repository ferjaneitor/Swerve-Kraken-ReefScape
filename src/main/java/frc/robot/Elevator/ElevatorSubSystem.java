package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

/**
 * ElevatorSubSystem es un subsistema que gestiona dos motores (SparkMax con NEO)
 * para controlar un elevador de tipo cascada. Emplea encoders integrados y control PID,
 * permitiendo mover el elevador a una altura objetivo o a una velocidad constante.
 *
 * Detalles:
 * - Los encoders se reinician al construir el objeto o cuando se llama a ResetEncoders().
 * - El método Meters2Rotations(double) convierte metros a rotaciones, teniendo en cuenta:
 *   el diámetro del sprocket (convertido de pulgadas a metros), la circunferencia,
 *   el offset mínimo (OffSetMeters), el número de etapas (ElevatorStages) y la relación de engranajes (GearRatio).
 * - Se utilizan dos PIDController independientes (uno por cada motor). El método
 *   targetHeightFromRotations(double) calcula la salida PID y establece el voltaje en cada motor.
 * - El método setVelocity(double) permite mover el elevador con una velocidad deseada,
 *   compensando la dirección de cada motor.
 *
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.6
 */
public class ElevatorSubSystem extends SubsystemBase {

    // Objeto SparkMax que controla el primer motor del elevador (ElevatorMotor).
    private SparkMax ElevatorMotor;

    // Encoder relativo integrado en el primer motor (ElevatorMotor).
    private RelativeEncoder ElevatorMotorEncoder;

    private SparkMaxConfig config;

    // Controlador PID para el primer motor (ElevatorMotor).
    private PIDController ElevatorMotorPidController;

    private PIDController elevatorFeederMotorPidController;

    //Se encarga de revisar si ya existe un comando ejecutandose
    private boolean CmdRunning = false ;

    /**
     * Construye el subsistema del elevador, inicializando los motores, encoders
     * y controladores PID. También reinicia los encoders para que partan de posición 0.
     */
    public ElevatorSubSystem() {
        this.ElevatorMotor = new SparkMax(ElevatorConstants.ElevatorMotorID, MotorType.kBrushless);

        this.ElevatorMotorEncoder = ElevatorMotor.getEncoder();

        this.ElevatorMotorPidController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        
        this.elevatorFeederMotorPidController = new PIDController(ElevatorConstants.KPFeeder, ElevatorConstants.KIFeeder, ElevatorConstants.KDFeeder);        
        
        // Convierte el diámetro del sprocket de pulgadas a centímetros y lo usa para calcular la circunferencia en metros.
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 2.54;
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;
        
        config = new SparkMaxConfig();
        config.encoder.positionConversionFactor((SproketCircumferenceMeters * ElevatorConstants.ElevatorStages) / ElevatorConstants.GearRatio);

        ElevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        ResetEncoders();    
    }

    /**
     * Reinicia (pone a cero) la posición de ambos encoders.
     * Se puede llamar cada vez que se desee recalibrar la posición de los motores del elevador.
     */
    public void ResetEncoders() {
        ElevatorMotorEncoder.setPosition(0);
    }

    /**
     * Convierte una distancia en metros (DistanceMeters) a rotaciones de los motores.
     * Considera el diámetro del sprocket (convertido de pulgadas a metros), la circunferencia,
     * el offset mínimo (OffSetMeters), el número de etapas (ElevatorStages) y la relación de engranajes (GearRatio).
     *
     * @param DistanceMeters Distancia objetivo en metros.
     * @return Número de rotaciones resultante para los motores.
     */
    public double Meters2Rotations(double DistanceMeters) {
        // Convierte el diámetro del sprocket de pulgadas a metros
        double SproketDiameterMeters = ElevatorConstants.SproketDiameterInches * 0.0254;
        // Calcula la circunferencia en metros
        double SproketCircumferenceMeters = SproketDiameterMeters * Math.PI;

        // Aplica el offset y divide por el número de etapas del elevador
        double DistanceMetersWithOffSet = SproketCircumferenceMeters - ElevatorConstants.OffSetMeters;
        double DistanceMetersHalf = DistanceMetersWithOffSet / ElevatorConstants.ElevatorStages;

        // Convierte la distancia final en rotaciones, multiplicando por la relación de engranajes
        return (DistanceMetersHalf / SproketCircumferenceMeters) * ElevatorConstants.GearRatio;
    }

    /**
     * Ajusta la altura del elevador según las rotaciones objetivo.
     * Calcula las salidas PID de cada motor (el segundo motor se invierte)
     * y establece el voltaje en cada motor.
     *
     * @param targetMeters Número de centimetros deseadas (positivo para subir, negativo para bajar).
     */
    public void targetHeightFromCentimeters(double targetMeters) {
        double ElevatorMotorOutput = ElevatorMotorPidController.calculate(ElevatorMotorEncoder.getPosition(), (targetMeters - ElevatorConstants.OffSetMeters));

        ElevatorMotor.setVoltage(ElevatorMotorOutput);
    }

    /**
     * Ajusta la altura del elevador según las rotaciones objetivo.
     * Calcula las salidas PID de cada motor (el segundo motor se invierte)
     * y establece el voltaje en cada motor.
     *
     * @param targetMeters Número de centimetros deseadas (positivo para subir, negativo para bajar).
     */
    public void targetHeightFeeder() {
        double ElevatorMotorOutput = elevatorFeederMotorPidController.calculate(ElevatorMotorEncoder.getPosition(), (ElevatorConstants.FeederHeight - ElevatorConstants.OffSetMeters));

        ElevatorMotor.setVoltage(ElevatorMotorOutput);
    }

    /**
     * Ejecuta un movimiento con perfil trapezoidal para alcanzar la 
    public void trapezoidalMotionProfeTargetHeight(double targetmeters) {
        double ElevatorMotorOutput = trapezoidRightProfiledPIDControllercalculate(-.getPosition(), (targetmeters- ElevatorConstants.OffSetMeters));

        ElevatorMotor.setVoltage(ElevatorMotorOutput);
        LeftMotor.setVoltage(LeftMotorOutput);
    }

    /**
     * Detiene los dos motores del elevador estableciendo su velocidad a 0.
     */
    public void stopMotors() {
        ElevatorMotor.set(0);
    }

    /**
     * Obtiene la posición (en rotaciones) del motor derecho (ElevatorMotor).
     *
     * @return Posición actual del encoder de ElevatorMotor.
     */
    public double getElevatorMotorPosition() {
        return ElevatorMotorEncoder.getPosition();
    }

    /**
     * Ajusta directamente la velocidad de ambos motores, aplicando una inversión en el segundo
     * para el correcto movimiento en cascada.
     *
     * @param Velocity Velocidad deseada (por ejemplo, entre -1.0 y 1.0).
     */
    public void setVelocity(double Velocity) {
        ElevatorMotor.set(-Velocity);
    }
    
    /**
     * Aplica automáticamente un PID que reinicia la posición del elevador.
     */
    public void resetPosition() {
        double ElevatorMotorOutput = ElevatorMotorPidController.calculate(ElevatorMotorEncoder.getPosition(), 0);

        ElevatorMotor.set(ElevatorMotorOutput);
    }    
    
    /**
     * Cambia el estado del comando en ejecución.
     *
     * @param StateOfTheCmd Nuevo estado del comando.
     */
    public void changeRunningCmd (boolean StateOfTheCmd) {

        CmdRunning = StateOfTheCmd;

    }

    /**
     * Verifica si el elevador alcanza la altura deseada.
     * Calcula el objetivo restando ElevatorConstants.OffSetMeters a level y compara
     * las posiciones de los motores con una tolerancia.
     *
     * @param level Altura objetivo.
     * @param rightPos Posición del motor derecho.
     * @param leftPos Posición del motor izquierdo.
     * @return True si ambas posiciones están dentro de la tolerancia; false en caso contrario.
     */
    public boolean isAtHeight(double level, double ElevatorPos) {
        double target = level - ElevatorConstants.OffSetMeters;
        return Math.abs(ElevatorPos - target) < ElevatorConstants.TOLERANCE;
    }

    /**
     * Actualiza el estado del elevador:
     * - Verifica si se encuentra en las alturas definidas (L1, L2, L3, L4, Feeder).
     * - Muestra en SmartDashboard el estado de cada altura y la posición de los motores.
     */
    @Override
    public void periodic() {
        
        boolean l1Height = false;
        boolean l2Height = false;
        boolean l3Height = false;
        boolean l4Height = false;
        boolean feederHeight = false;
        
        if (isAtHeight(ElevatorConstants.L1, getElevatorMotorPosition())) {
            l1Height = true;
            l2Height = false;
            l3Height = false;
            l4Height = false;
            feederHeight = false;
        }
        else if (isAtHeight(ElevatorConstants.L2, getElevatorMotorPosition())) {
            l1Height = false;
            l2Height = true;
            l3Height = false;
            l4Height = false;
            feederHeight = false;
        }
        else if (isAtHeight(ElevatorConstants.L3, getElevatorMotorPosition())) {
            l1Height = false;
            l2Height = false;
            l3Height = true;
            l4Height = false;
            feederHeight = false;
        }
        else if (isAtHeight(ElevatorConstants.L4, getElevatorMotorPosition())) {
            l1Height = false;
            l2Height = false;
            l3Height = false;
            l4Height = true;
            feederHeight = false;
        }
        else if (isAtHeight(ElevatorConstants.FeederHeight, getElevatorMotorPosition())) {
            l1Height = false;
            l2Height = false;
            l3Height = false;
            l4Height = false;
            feederHeight = true;
        }
        else {
            l1Height = false;
            l2Height = false;
            l3Height = false;
            l4Height = false;
            feederHeight = false;
        }
        
        SmartDashboard.putBoolean("The elevator Has a running CMD ?", CmdRunning);
        
        SmartDashboard.putBoolean("Your are at L1 Height", l1Height);
        SmartDashboard.putBoolean("Your are at L2 Height", l2Height);
        SmartDashboard.putBoolean("Your are at L3 Height", l3Height);
        SmartDashboard.putBoolean("Your are at L4 Height", l4Height);
        SmartDashboard.putBoolean("Your are at Feeder Height", feederHeight);
    
        SmartDashboard.putNumber("Right Elevator Motor Position", -getElevatorMotorPosition());
         
        super.periodic();
    }
}
