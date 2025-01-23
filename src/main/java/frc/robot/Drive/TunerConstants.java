package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.constants;
import frc.robot.Drive.CommandSwerveDrivetrain;
import frc.robot.constants.drivetrainConstants;

/**
 * TunerConstants
 * 
 * <p>Clase que gestiona la configuración y constantes necesarias para
 * implementar un sistema de Swerve Drive con la API Phoenix 6 de
 * CTR Electronics (TalonFX, CANCoder, Pigeon2, etc.).</p>
 *
 * <p>Provee las ganancias (PID/Feedforward), relaciones de engranaje,
 * configuraciones de limitación de corriente, tipos de bucle cerrado,
 * y coordenadas de cada módulo (rueda) del robot. También incluye
 * la creación de un objeto {@link CommandSwerveDrivetrain} que
 * integra todos estos componentes para la conducción.</p>
 *
 * @author  Fernando Joel Cruz Briones
 * @version 1.0
 */

// Generado por Tuner X Swerve Project Generator
// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
@SuppressWarnings("unused")
public class TunerConstants {
    
    /**
     * Configuraciones de la ranura 0 (Slot0Configs) para los motores de dirección (steer).
     * En esta configuración se definen los valores de control PID:
     * <ul>
     *   <li>KP: Ganancia Proporcional</li>
     *   <li>KI: Ganancia Integral</li>
     *   <li>KD: Ganancia Derivativa</li>
     *   <li>KS, KV, KA: Parámetros de feedforward</li>
     *   <li>StaticFeedforwardSign: Determina el signo del feedforward</li>
     * </ul>
     */
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(drivetrainConstants.Slot0ConfigSteerGainsWithKP)
        .withKI(drivetrainConstants.Slot0ConfigSteerGainsWithKI)
        .withKD(drivetrainConstants.Slot0ConfigSteerGainsWithKD)
        .withKS(drivetrainConstants.Slot0ConfigSteerGainsWithKS)
        .withKV(drivetrainConstants.Slot0ConfigSteerGainsWithKV)
        .withKA(drivetrainConstants.Slot0ConfigSteerGainsWithKA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    /**
     * Configuraciones de la ranura 0 (Slot0Configs) para los motores de propulsión (drive).
     * Similar a la configuración de dirección, pero con sus propios valores
     * de PID y feedforward adecuados para la parte de tracción.
     */
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(drivetrainConstants.Slot0ConfigDriveGainsWithKP)
        .withKI(drivetrainConstants.Slot0ConfigDriveGainsWithKI)
        .withKD(drivetrainConstants.Slot0ConfigDriveGainsWithKD)
        .withKS(drivetrainConstants.Slot0ConfigDriveGainsWithKS)
        .withKV(drivetrainConstants.Slot0ConfigDriveGainsWithKV);

    /**
     * Tipo de control en bucle cerrado para los motores de dirección (steer).
     * En este caso, se utiliza {@code ClosedLoopOutputType.Voltage},
     * es decir, control basado en voltaje.
     */
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    
    /**
     * Tipo de control en bucle cerrado para los motores de propulsión (drive).
     * También se establece en control por voltaje.
     */
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    /**
     * Tipo de motor usado para la propulsión (drive). Aquí se define que
     * se utiliza un TalonFX (Falcon 500) con codificador integrado.
     */
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    
    /**
     * Tipo de motor usado para la dirección (steer). Igualmente un TalonFX con
     * codificador integrado.
     */
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

     /**
     * Tipo de retroalimentación (feedback) para los motores de dirección.
     * {@code FusedCANcoder} combina datos del encoder integrado y el CANCoder.
     * En caso de no tener licencia Pro, se hace fallback a {@code RemoteCANcoder}.
     */
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    /**
     * Corriente de deslizamiento (slip) que se utiliza para detectar cuando
     * la rueda está patinando. Este valor se debe ajustar según el robot.
     */
    private static final Current kSlipCurrent = Amps.of(drivetrainConstants.kSlipCurrentAmp);

     /**
     * Configuración inicial para el motor de propulsión (drive).
     * Aquí se dejan valores por defecto.
     */
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    
    /**
     * Configuración inicial para el motor de dirección (steer).
     * En este ejemplo, se establece un límite de corriente de estator
     * relativamente bajo, dado que la dirección no requiere tanto torque.
     */
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(drivetrainConstants.StatorCurrentLimitAmps))
                .withStatorCurrentLimitEnable(drivetrainConstants.StatorCurrentLimitEnable)
        );
    
    /**
     * Configuración inicial para el CANCoder (encoder de azimut o dirección).
     */
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    
    /**
     * Configuración para el giroscopio Pigeon2. En este caso se deja en null,
     * lo que significa que no se aplican configuraciones adicionales.
     */
    private static final Pigeon2Configuration pigeonConfigs = null;

    /**
     * CANBus a utilizar. Normalmente, todos los dispositivos de swerve
     * (motores, encoders) deben compartir el mismo bus CAN.
     * Se define además una ruta para logs (opcional).
     */
    public static final CANBus kCANBus = new CANBus(drivetrainConstants.CANivoreCanBusName, "./logs/example.hoot");

    /**
     * Velocidad teórica (en m/s) con 12 V aplicados directamente a los motores.
     * Este valor se usa en los cálculos de feedforward y depende de cada robot.
     */
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(drivetrainConstants.kSpeedAt12Volts);

    /**
     * Constantes generales del tren motriz swerve.
     * Se establece el ID del Pigeon2 y el nombre del CANBus.
     */
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(drivetrainConstants.kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    /**
     * Fábrica para crear las configuraciones de los módulos de Swerve
     * (drive + steer + encoder) utilizando los parámetros definidos.
     */
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
    
    /**
     * Configuraciones del módulo swerve de la rueda Front-Left (delantera izquierda).
     * Incluye IDs de motor de dirección, propulsión, CANCoder,
     * offset del encoder, posición física (X, Y) y banderas de inversión.
     */
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
    
    /**
     * Configuraciones del módulo swerve de la rueda Front-Right (delantera derecha).
     */
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
    
    /**
     * Configuraciones del módulo swerve de la rueda Back-Left (trasera izquierda).
     */
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
    
    /**
     * Configuraciones del módulo swerve de la rueda Back-Right (trasera derecha).
     */
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
     * Crea una instancia de {@link CommandSwerveDrivetrain}.
     * 
     * <p>Se recomienda llamar a este método una sola vez en el programa,
     * típicamente en la clase RobotContainer o similar, para inicializar
     * el subsistema de conducción.</p>
     *
     * @return Una nueva instancia de {@code CommandSwerveDrivetrain}.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    /**
     * Clase interna que extiende {@link SwerveDrivetrain} para construir
     * un chasis swerve utilizando {@link TalonFX} y {@link CANcoder}.
     * Provee varios constructores para configurar frecuencia de actualización
     * de odometría y desviaciones estándar para la fusión de sensores.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        
        /**
         * Constructor principal. Permite inicializar el swerve a partir de
         * constantes del tren motriz y los módulos (ruedas) específicos.
         *
         * @param drivetrainConstants  Constantes y configuraciones generales
         *                             para el tren motriz.
         * @param modules              Configuración de cada módulo swerve
         *                             (rueda) a construir.
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new,     // Función para construir TalonFX de Drive
                TalonFX::new,     // Función para construir TalonFX de Steer
                CANcoder::new,    // Función para construir CANCoder
                drivetrainConstants, modules
            );
        }

        /**
         * Constructor que además permite especificar la frecuencia de
         * actualización para la odometría.
         *
         * @param drivetrainConstants     Constantes y configuraciones generales
         *                                para el tren motriz.
         * @param odometryUpdateFrequency Frecuencia de actualización de la odometría
         *                                (en Hz). Si se deja en 0, se aplican valores
         *                                predeterminados (250 Hz en CAN FD, 100 Hz
         *                                en CAN 2.0).
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
         * Constructor que permite además definir desviaciones estándar para la
         * fusión de la odometría y la visión.
         *
         * @param drivetrainConstants       Constantes generales del tren motriz.
         * @param odometryUpdateFrequency   Frecuencia de actualización de odometría.
         * @param odometryStandardDeviation Desviación estándar para la odometría
         *                                  (matriz de 3x1) en [x, y, theta].
         * @param visionStandardDeviation   Desviación estándar para la visión
         *                                  (matriz de 3x1) en [x, y, theta].
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
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
