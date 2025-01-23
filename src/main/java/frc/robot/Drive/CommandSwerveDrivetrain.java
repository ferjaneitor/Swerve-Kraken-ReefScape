package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
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
}
