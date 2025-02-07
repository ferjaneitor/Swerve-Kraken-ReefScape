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
 * CommandSwerveDrivetrain
 *
 * <p>Clase que extiende la implementación de Phoenix 6 para SwerveDrivetrain
 * e implementa la interfaz Subsystem de WPILib, facilitando su uso en un
 * proyecto basado en comandos (command-based).</p>
 *
 * <p>Permite aplicar peticiones (Requests) de swerve específicas, manejar
 * rutinas SysId para caracterizar los motores (tanto de tracción como de
 * dirección), y, en caso de simulación, ejecutar el ciclo de simulación
 * con mayor frecuencia para tener una respuesta de PID razonable.</p>
 *
 * <p>Autor:  Fernando Joel Cruz Briones</p>
 * <p>Versión: 1.0</p>
 * 
 * <p>Clase que extiende {@link TunerSwerveDrivetrain} e implementa la interfaz
 * {@link Subsystem}, lo cual facilita el uso de este tren motriz en un entorno
 * basado en comandos (command-based). Incluye manejo de rutinas SysId para
 * caracterización de motores (drive y steer), así como lógica especial en
 * simulación.</p>
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
     * Último momento de actualización de la simulación, en segundos.
     */
    private double m_lastSimTime;

    /**
     * Rotación que define la "perspectiva" de la Alianza Azul en el campo
     * (0 grados mirando hacia la pared de la Alianza Roja).
     */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

    /**
     * Rotación que define la "perspectiva" de la Alianza Roja en el campo
     * (180 grados mirando hacia la pared de la Alianza Azul).
     */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    /**
     * Bandera para saber si ya se ha aplicado la perspectiva del operador
     * (para cambiar la referencia de "adelante" según la alianza).
     */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Petición de Swerve específica para la caracterización de la traslación
     * (drive motors). Se utiliza en el modo SysIdTranslation.
     */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();

    /**
     * Petición de Swerve específica para la caracterización de los motores de
     * dirección (steer). Se utiliza en el modo SysIdSteer.
     */
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    /**
     * Petición de Swerve específica para la caracterización rotacional.
     * Se utiliza en el modo SysIdRotation (ej. para PID de rotación).
     */
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /**
     * SwerveRequest que se utiliza para aplicar velocidades en el marco de referencia
     * del robot (robot-centric). Específicamente se emplea al seguir trayectorias 
     * donde los comandos de traslación y rotación se expresan desde la perspectiva
     * del propio robot.
     */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

     /**
     * Rutina SysId para la caracterización de la traslación (drive motors).
     * Permite determinar las ganancias PID para los motores de tracción.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,           // Ramp rate por defecto (1 V/s)
            Volts.of(4),    // Voltage dinámico reducido a 4V para evitar brownout
            null,           // Timeout por defecto (10 s)
            // Log de estado con SignalLogger
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            // Función que aplica el voltaje de caracterización a los motores
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /**
     * Rutina SysId para caracterizar los motores de dirección (steer).
     * Ayuda a encontrar las ganancias PID apropiadas para steering.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,          // Ramp rate por defecto (1 V/s)
            Volts.of(7),   // Voltage dinámico de 7 V
            null,          // Timeout por defecto (10 s)
            // Log de estado con SignalLogger
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
     * Se utiliza para determinar las ganancias PID de rotación para
     * {@code FieldCentricFacingAngle} u otros controladores similares.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* Valor en rad/s², pero SysId solo soporta 'volts per second'. */
            Volts.of(Math.PI / 6).per(Second),
            /* Valor en rad/s, pero SysId solo soporta 'volts'. */
            Volts.of(Math.PI),
            null, // Timeout por defecto (10 s)
            // Log de estado con SignalLogger
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /*
                 * 'output' está en rad/s, pero SysId solo maneja 'volts' como unidad.
                 * Se aplica la conversión para mandar la petición de rotación.
                 */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));

                // También se registra el valor pedido para SysId
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
     * Constructor principal para un CTRE SwerveDrivetrain, el cual inicializa
     * los dispositivos de hardware subyacentes y, si está en simulación,
     * inicia un hilo de simulación a mayor frecuencia.
     *
     * @param drivetrainConstants   Constantes generales del tren motriz swerve.
     * @param modules               Configuraciones de cada módulo swerve (rueda).
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
     * Constructor que además permite especificar la frecuencia de actualización
     * de la odometría. Si se deja en 0, se utilizarán valores por defecto
     * (250 Hz en CAN FD y 100 Hz en CAN 2.0).
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
     * Constructor que permite además definir desviaciones estándar para la
     * fusión de la odometría y la visión, con la frecuencia de actualización
     * de odometría especificada.
     *
     * @param drivetrainConstants       Constantes generales del tren motriz swerve.
     * @param odometryUpdateFrequency   Frecuencia en Hz para la odometría.
     * @param odometryStandardDeviation Desviación estándar de la odometría
     *                                  ([x, y, theta]) con unidades de
     *                                  metros y radianes.
     * @param visionStandardDeviation   Desviación estándar de la visión
     *                                  ([x, y, theta]) con unidades de
     *                                  metros y radianes.
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
     * Configura el {@code AutoBuilder} para la ejecución de trayectorias (PathPlanner).
     * <p>
     * - Obtiene la pose actual del robot y permite restablecerla al inicio de la ruta.
     * - Aplica las velocidades y feedforwards al tren motriz en modo robot-centric,
     *   utilizando {@link SwerveRequest.ApplyRobotSpeeds}.
     * - Configura el controlador holonómico ({@code PPHolonomicDriveController})
     *   con ganancias PID para traslación y rotación.
     * - Determina si se debe invertir la ruta según la Alianza (Roja/Azul) reportada
     *   por {@link DriverStation}.
     * </p>
     * <p>
     * En caso de ocurrir un error al cargar la configuración de PathPlanner,
     * se reporta un mensaje a la consola de DriverStation.
     * </p>
     */
    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier de la pose actual del robot
                this::resetPose,         // Consumer para restablecer la pose al inicio
                () -> getState().Speeds, // Supplier de velocidades actuales del robot
                
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
     * Crea y devuelve un {@link Command} que al ejecutarse aplica la petición
     * (request) dada por {@code requestSupplier} a este tren motriz swerve.
     *
     * @param requestSupplier Función que retorna el {@link SwerveRequest} a aplicar.
     * @return Un comando que, al ejecutarse, llama a {@code setControl()} con
     *         el request obtenido.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Ejecuta la prueba SysId Quasistatic en la dirección especificada (avance
     * o retroceso) para la rutina seleccionada en {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Dirección de la prueba (FORWARD o BACKWARD).
     * @return Un comando que, al ejecutarse, realiza la caracterización Quasistatic.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Ejecuta la prueba SysId Dynamic en la dirección especificada (avance
     * o retroceso) para la rutina seleccionada en {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Dirección de la prueba (FORWARD o BACKWARD).
     * @return Un comando que, al ejecutarse, realiza la caracterización Dynamic.
     */ 
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Método periódico de WPILib que se llama regularmente.
     * <p>
     * Se utiliza para aplicar la perspectiva de operador (rotación hacia la
     * Alianza Roja o Azul) si la estación de conducción está deshabilitada o
     * si aún no se ha establecido.
     */
    @Override
    public void periodic() {
        /*
         * Intentar aplicar la perspectiva del operador de manera periódica.
         * Si nunca se ha aplicado, se hace directamente.
         * De lo contrario, solo se aplica cuando el robot está deshabilitado,
         * para no alterar la conducción en medio de un periodo habilitado.
         */
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
     * Inicia un hilo separado para la simulación, ejecutándose a un
     * período menor (5 ms) para mejorar el comportamiento de los bucles PID.
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
     * Añade una medición de visión al filtro de Kalman, ajustando la estimación de la
     * pose del robot conforme a la información de la cámara de visión, tomando en cuenta
     * el ruido de medición.
     * <p>
     * El método recibe la pose del robot medida por visión ({@code visionRobotPoseMeters})
     * y el instante de tiempo ({@code timestampSeconds}) en que se tomó la medición.
     * Se encarga de convertir ese tiempo a la referencia interna con
     * {@code Utils.fpgaToCurrentTime}.
     * </p>
     * @param visionRobotPoseMeters La pose estimada por la cámara de visión (en metros).
     * @param timestampSeconds Momento en que se registró la medición, en segundos.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Añade una medición de visión al filtro de Kalman, ajustando la estimación de la
     * pose del robot conforme a la información de la cámara de visión y teniendo en
     * cuenta desviaciones estándar específicas para esta medición.
     * <p>
     * La matriz {@code visionMeasurementStdDevs} representa la desviación estándar
     * para x, y y theta ([x, y, theta]) expresadas en metros y radianes. Estos valores
     * se aplicarán a mediciones futuras hasta que se vuelva a llamar a
     * {@link #setVisionMeasurementStdDevs(Matrix)} o se llame nuevamente a este método
     * con parámetros distintos.
     * </p>
     * @param visionRobotPoseMeters La pose estimada por la cámara de visión (en metros).
     * @param timestampSeconds Momento en que se registró la medición, en segundos.
     * @param visionMeasurementStdDevs Desviaciones estándar de la medición de visión
     *                                 en el formato [x, y, theta], con x e y en metros
     *                                 y theta en radianes.
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
