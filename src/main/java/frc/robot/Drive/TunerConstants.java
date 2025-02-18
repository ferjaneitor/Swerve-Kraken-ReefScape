package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.constants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.constants.drivetrainConstants;

/**
 * TunerConstants
 * 
 * Esta clase gestiona la configuración y las constantes necesarias para implementar un sistema de Swerve Drive
 * utilizando la API Phoenix 6 de CTR Electronics (TalonFX, CANCoder, Pigeon2, etc.).
 * 
 * Provee las ganancias (PID/Feedforward), relaciones de engranaje, configuraciones de limitación de corriente,
 * tipos de bucle cerrado y coordenadas de cada módulo (rueda) del robot. Además, incluye la creación de un objeto
 * CommandSwerveDrivetrain que integra todos estos componentes para la conducción.
 * 
 * Autor:  Fernando Joel Cruz Briones
 * Versión: 1.0
 * 
 * Generado por Tuner X Swerve Project Generator
 * https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 */
@SuppressWarnings("unused")
public class TunerConstants {
    
    // Configuraciones de la ranura 0 para los motores de dirección (steer).
    // Se definen los valores de control PID:
    // - KP: Ganancia Proporcional
    // - KI: Ganancia Integral
    // - KD: Ganancia Derivativa
    // - KS, KV, KA: Parámetros de feedforward
    // - StaticFeedforwardSign: Determina el signo del feedforward
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(drivetrainConstants.Slot0ConfigSteerGainsWithKP)
        .withKI(drivetrainConstants.Slot0ConfigSteerGainsWithKI)
        .withKD(drivetrainConstants.Slot0ConfigSteerGainsWithKD)
        .withKS(drivetrainConstants.Slot0ConfigSteerGainsWithKS)
        .withKV(drivetrainConstants.Slot0ConfigSteerGainsWithKV)
        .withKA(drivetrainConstants.Slot0ConfigSteerGainsWithKA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    // Configuraciones de la ranura 0 para los motores de propulsión (drive).
    // Similar a la configuración de dirección, pero con valores de PID y feedforward adecuados para tracción.
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(drivetrainConstants.Slot0ConfigDriveGainsWithKP)
        .withKI(drivetrainConstants.Slot0ConfigDriveGainsWithKI)
        .withKD(drivetrainConstants.Slot0ConfigDriveGainsWithKD)
        .withKS(drivetrainConstants.Slot0ConfigDriveGainsWithKS)
        .withKV(drivetrainConstants.Slot0ConfigDriveGainsWithKV);

    // Tipo de control en bucle cerrado para los motores de dirección (steer). Se usa control basado en voltaje.
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    
    // Tipo de control en bucle cerrado para los motores de propulsión (drive). También se establece en control por voltaje.
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Tipo de motor usado para la propulsión (drive). Se utiliza un TalonFX (Falcon 500) con codificador integrado.
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    
    // Tipo de motor usado para la dirección (steer). También se utiliza un TalonFX con codificador integrado.
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // Tipo de retroalimentación para los motores de dirección.
    // FusedCANcoder combina datos del encoder integrado y el CANCoder. Si no se tiene licencia Pro, se hace fallback a RemoteCANcoder.
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Corriente de deslizamiento (slip) para detectar cuando la rueda está patinando.
    private static final Current kSlipCurrent = Amps.of(drivetrainConstants.kSlipCurrentAmp);

    // Configuración inicial para el motor de propulsión (drive). Se dejan valores por defecto.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    
    // Configuración inicial para el motor de dirección (steer). Se establece un límite de corriente de estator bajo, ya que la dirección no requiere tanto torque.
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(drivetrainConstants.StatorCurrentLimitAmps))
                .withStatorCurrentLimitEnable(drivetrainConstants.StatorCurrentLimitEnable)
        );
    
    // Configuración inicial para el CANCoder (encoder de azimut o dirección).
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    
    // Configuración para el giroscopio Pigeon2. Se deja en null, sin configuraciones adicionales.
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CANBus a utilizar. Normalmente, todos los dispositivos de swerve deben compartir el mismo bus CAN.
    // Se define además una ruta para logs (opcional).
    public static final CANBus kCANBus = new CANBus(drivetrainConstants.CANivoreCanBusName, "./logs/example.hoot");

    // Velocidad teórica (en m/s) con 12 V aplicados directamente a los motores.
    // Este valor se usa en los cálculos de feedforward y depende de cada robot.
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(drivetrainConstants.kSpeedAt12Volts);

    // Constantes generales del tren motriz swerve.
    // Se establece el ID del Pigeon2 y el nombre del CANBus.
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(drivetrainConstants.kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    // Fábrica para crear las configuraciones de los módulos de Swerve (drive, steer y encoder) utilizando los parámetros definidos.
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(drivetrainConstants.kDriveGearRatio)
            .withSteerMotorGearRatio(drivetrainConstants.kSteerGearRatio)
            .withCouplingGearRatio(drivetrainConstants.kCoupleRatio)
            .withWheelRadius(drivetrainConstants.kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(drivetrainConstants.kSteerInertia)
            .withDriveInertia(drivetrainConstants.kDriveInertia)
            .withSteerFrictionVoltage(drivetrainConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(drivetrainConstants.kDriveFrictionVoltage);
    
    // Configuración del módulo swerve de la rueda Front-Left (delantera izquierda).
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            drivetrainConstants.kFrontLeftSteerMotorId,
            drivetrainConstants.kFrontLeftDriveMotorId, 
            drivetrainConstants.kFrontLeftEncoderId, 
            drivetrainConstants.kFrontLeftEncoderOffset,
            drivetrainConstants.kFrontLeftXPos, 
            drivetrainConstants.kFrontLeftYPos, 
            drivetrainConstants.kInvertLeftSide, 
            drivetrainConstants.kFrontLeftSteerMotorInverted, 
            drivetrainConstants.kFrontLeftEncoderInverted
        );
    
    // Configuración del módulo swerve de la rueda Front-Right (delantera derecha).
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            drivetrainConstants.kFrontRightSteerMotorId, 
            drivetrainConstants.kFrontRightDriveMotorId, 
            drivetrainConstants.kFrontRightEncoderId, 
            drivetrainConstants.kFrontRightEncoderOffset,
            drivetrainConstants.kFrontRightXPos, 
            drivetrainConstants.kFrontRightYPos, 
            drivetrainConstants.kInvertRightSide, 
            drivetrainConstants.kFrontRightSteerMotorInverted, 
            drivetrainConstants.kFrontRightEncoderInverted
        );
    
    // Configuración del módulo swerve de la rueda Back-Left (trasera izquierda).
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            drivetrainConstants.kBackLeftSteerMotorId, 
            drivetrainConstants.kBackLeftDriveMotorId, 
            drivetrainConstants.kBackLeftEncoderId, 
            drivetrainConstants.kBackLeftEncoderOffset,
            drivetrainConstants.kBackLeftXPos, 
            drivetrainConstants.kBackLeftYPos, 
            drivetrainConstants.kInvertLeftSide, 
            drivetrainConstants.kBackLeftSteerMotorInverted, 
            drivetrainConstants.kBackLeftEncoderInverted
        );
    
    // Configuración del módulo swerve de la rueda Back-Right (trasera derecha).
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            drivetrainConstants.kBackRightSteerMotorId, 
            drivetrainConstants.kBackRightDriveMotorId, 
            drivetrainConstants.kBackRightEncoderId, 
            drivetrainConstants.kBackRightEncoderOffset,
            drivetrainConstants.kBackRightXPos, 
            drivetrainConstants.kBackRightYPos, 
            drivetrainConstants.kInvertRightSide, 
            drivetrainConstants.kBackRightSteerMotorInverted, 
            drivetrainConstants.kBackRightEncoderInverted
        );

    /**
     * Crea una instancia de CommandSwerveDrivetrain.
     * 
     * Se recomienda llamar a este método una sola vez en el programa, típicamente en RobotContainer,
     * para inicializar el subsistema de conducción.
     * 
     * @return Una nueva instancia de CommandSwerveDrivetrain.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    /**
     * Clase interna que extiende SwerveDrivetrain para construir un chasis swerve utilizando TalonFX y CANcoder.
     * Provee varios constructores para configurar la frecuencia de actualización de la odometría y desviaciones estándar
     * para la fusión de sensores.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        
        /**
         * Constructor principal.
         * Permite inicializar el swerve a partir de las constantes del tren motriz y los módulos (ruedas) específicos.
         *
         * @param drivetrainConstants Constantes y configuraciones generales para el tren motriz.
         * @param modules             Configuración de cada módulo swerve a construir.
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new,     // Función para construir TalonFX de Drive
                TalonFX::new,     // Función para construir TalonFX de Steer
                CANcoder::new,    // Función para construir CANcoder
                drivetrainConstants, modules
            );
        }

        /**
         * Constructor que permite especificar la frecuencia de actualización de la odometría.
         *
         * @param drivetrainConstants     Constantes y configuraciones generales para el tren motriz.
         * @param odometryUpdateFrequency Frecuencia en Hz para actualizar la odometría.
         * @param modules                 Configuración de cada módulo swerve.
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructor que permite definir desviaciones estándar para la fusión de la odometría y la visión.
         *
         * @param drivetrainConstants       Constantes generales del tren motriz.
         * @param odometryUpdateFrequency   Frecuencia en Hz para la odometría.
         * @param odometryStandardDeviation Desviación estándar para la odometría (matriz de 3x1) en [x, y, theta].
         * @param visionStandardDeviation   Desviación estándar para la visión (matriz de 3x1) en [x, y, theta].
         * @param modules                   Configuración de cada módulo swerve.
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
