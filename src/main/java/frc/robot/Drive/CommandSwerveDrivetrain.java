package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Drive.TunerConstants.TunerSwerveDrivetrain;

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
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /**
     * Período del hilo de simulación, en segundos (5 ms).
     */
    private static final double kSimLoopPeriod = 0.005; // 5 ms

    /**
     * Notificador para la simulación (se ejecuta en un hilo separado).
     */
    private Notifier m_simNotifier = null;

    /**
     * Último instante de actualización de la simulación, en segundos.
     */
    private double m_lastSimTime;

    /**
     * Rotación que define la perspectiva de la Alianza Azul en el campo (0° mirando hacia la pared de la Alianza Roja).
     */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

    /**
     * Rotación que define la perspectiva de la Alianza Roja en el campo (180° mirando hacia la pared de la Alianza Azul).
     */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    /**
     * Bandera para saber si ya se ha aplicado la perspectiva del operador (para cambiar la referencia de "adelante"
     * según la alianza).
     */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Petición de Swerve para la caracterización de la traslación (motores de drive). Se utiliza en el modo SysIdTranslation.
     */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();

    /**
     * Petición de Swerve para la caracterización de los motores de dirección (steer). Se utiliza en el modo SysIdSteer.
     */
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Petición de Swerve para la caracterización rotacional. Se utiliza en el modo SysIdRotation, por ejemplo para PID de rotación.
     */
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /**
     * SwerveRequest que se utiliza para aplicar velocidades en el marco de referencia del robot (robot-centric).
     * Se emplea al seguir trayectorias donde los comandos de traslación y rotación se expresan desde la perspectiva
     * del propio robot.
     */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

     /**
     * Rutina SysId para la caracterización de la traslación (motores de drive).
     * Permite determinar las ganancias PID para los motores de tracción.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,                   // Ramp rate por defecto (1 V/s)
            Volts.of(4),            // Voltaje dinámico reducido a 4V para evitar brownout
            null,                   // Timeout por defecto (10 s)
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /**
     * Rutina SysId para caracterizar los motores de dirección (steer).
     * Ayuda a encontrar las ganancias PID apropiadas para el steering.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,                  // Ramp rate por defecto (1 V/s)
            Volts.of(7),           // Voltaje dinámico de 7 V
            null,                  // Timeout por defecto (10 s)
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /**
     * Rutina SysId para caracterizar la rotación.
     * Se utiliza para determinar las ganancias PID de rotación para FieldCentricFacingAngle u otros controladores similares.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            // Valor en rad/s² (SysId solo soporta "volts per second")
            Volts.of(Math.PI / 6).per(Second),
            // Valor en rad/s (SysId solo soporta "volts")
            Volts.of(Math.PI),
            null, // Timeout por defecto (10 s)
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                // 'output' está en rad/s, se convierte a volts para la petición de rotación
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /**
     * Rutina SysId activa, por defecto la de traslación.
     */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

     /**
     * Constructor principal para un CTRE SwerveDrivetrain. Inicializa los dispositivos de hardware subyacentes
     * y, si está en simulación, inicia un hilo de simulación a mayor frecuencia.
     *
     * @param drivetrainConstants Constantes generales del tren motriz swerve.
     * @param modules             Configuraciones de cada módulo swerve (rueda).
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

     /**
     * Constructor que permite especificar la frecuencia de actualización de la odometría.
     * Si se deja en 0, se utilizarán valores por defecto (250 Hz en CAN FD y 100 Hz en CAN 2.0).
     *
     * @param drivetrainConstants     Constantes generales del tren motriz swerve.
     * @param odometryUpdateFrequency Frecuencia en Hz para actualizar la odometría.
     * @param modules                 Configuraciones de cada módulo swerve.
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructor que permite definir desviaciones estándar para la fusión de odometría y visión,
     * junto con la frecuencia de actualización de la odometría.
     *
     * @param drivetrainConstants       Constantes generales del tren motriz swerve.
     * @param odometryUpdateFrequency   Frecuencia en Hz para la odometría.
     * @param odometryStandardDeviation Desviación estándar de la odometría ([x, y, theta]) en metros y radianes.
     * @param visionStandardDeviation   Desviación estándar de la visión ([x, y, theta]) en metros y radianes.
     * @param modules                   Configuraciones de cada módulo swerve.
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Configura el AutoBuilder para la ejecución de trayectorias (PathPlanner).
     * 
     * - Obtiene la pose actual del robot y permite restablecerla al inicio de la ruta.
     * - Aplica las velocidades y feedforwards al tren motriz en modo robot-centric, utilizando SwerveRequest.ApplyRobotSpeeds.
     * - Configura el controlador holonómico (PPHolonomicDriveController) con ganancias PID para traslación y rotación.
     * - Determina si se debe invertir la ruta según la alianza (Roja/Azul) reportada por DriverStation.
     * 
     * En caso de error al cargar la configuración de PathPlanner, se reporta un mensaje a la consola de DriverStation.
     */
    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Proveedor de la pose actual del robot
                this::resetPose,         // Consumer para restablecer la pose al inicio
                () -> getState().Speeds, // Proveedor de velocidades actuales del robot
                
                // Consumer de ChassisSpeeds y feedforwards para manejar el robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // Constantes PID para traslación
                    new PIDConstants(10, 0, 0),
                    // Constantes PID para rotación
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Supone que la ruta debe invertirse para la Alianza Roja
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem para requerimientos
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

     /**
     * Crea y devuelve un Command que, al ejecutarse, aplica la petición (request) dada por requestSupplier
     * a este tren motriz swerve.
     *
     * @param requestSupplier Función que retorna el SwerveRequest a aplicar.
     * @return Un Command que llama a setControl() con el request obtenido.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

     /**
     * Ejecuta la prueba SysId Quasistatic en la dirección especificada (avance o retroceso)
     * para la rutina seleccionada en m_sysIdRoutineToApply.
     *
     * @param direction Dirección de la prueba (FORWARD o BACKWARD).
     * @return Un Command que realiza la caracterización Quasistatic.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Ejecuta la prueba SysId Dynamic en la dirección especificada (avance o retroceso)
     * para la rutina seleccionada en m_sysIdRoutineToApply.
     *
     * @param direction Dirección de la prueba (FORWARD o BACKWARD).
     * @return Un Command que realiza la caracterización Dynamic.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

     /**
     * Método periódico de WPILib que se llama regularmente.
     * 
     * Se utiliza para aplicar la perspectiva del operador (rotación hacia la Alianza Roja o Azul)
     * si el robot está deshabilitado o si aún no se ha establecido.
     */
    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Inicia un hilo separado para la simulación, que se ejecuta a un período menor (5 ms)
     * para mejorar el comportamiento de los bucles PID.
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

             // Se actualiza el estado de la simulación con el delta de tiempo medido,
            // usando la tensión de la batería que provee WPILib.
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Añade una medición de visión al filtro de Kalman, ajustando la estimación de la pose del robot
     * conforme a la información de la cámara de visión y tomando en cuenta el ruido de medición.
     *
     * @param visionRobotPoseMeters La pose estimada por la cámara de visión (en metros).
     * @param timestampSeconds      Momento en que se registró la medición, en segundos.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Añade una medición de visión al filtro de Kalman, ajustando la estimación de la pose del robot
     * conforme a la información de la cámara y teniendo en cuenta desviaciones estándar específicas.
     *
     * @param visionRobotPoseMeters   La pose estimada por la cámara (en metros).
     * @param timestampSeconds        Momento en que se registró la medición, en segundos.
     * @param visionMeasurementStdDevs Desviaciones estándar de la medición ([x, y, theta]) en metros y radianes.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

}
