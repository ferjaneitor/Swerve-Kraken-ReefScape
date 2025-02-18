package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Telemetry se encarga de publicar información de telemetría relacionada con el sistema de conducción swerve.
 * Envía datos a NetworkTables (para visualizarlos en SmartDashboard o Shuffleboard), a un archivo de registro
 * mediante SignalLogger, y a visualizaciones gráficas mediante Mechanism2d.
 *
 * Funciones principales:
 * - Publicar el estado del drive swerve (pose, velocidades, estados y posiciones de los módulos) en la tabla "DriveState".
 * - Enviar datos en formato de arreglos de double a SignalLogger para análisis offline.
 * - Publicar la pose del robot para una visualización virtual en Field2d.
 * - Actualizar visualizaciones de cada módulo swerve en Mechanism2d, mostrando ángulo y velocidad relativa.
 *
 * Autor: Fernando Joel Cruz Briones
 * Versión: 1.0
 */
public class Telemetry {

    /**
     * Velocidad máxima del robot (m/s). Se utiliza para escalar la longitud de los ligamentos de velocidad en Mechanism2d.
     */
    private final double MaxSpeed;

    /**
     * Construye un objeto de telemetría, especificando la velocidad máxima del robot para la visualización adecuada.
     *
     * @param maxSpeed Velocidad máxima en m/s de la unidad swerve.
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();
    }

    // Instancia de NetworkTables para publicar datos de telemetría.
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Publicadores de DriveState.
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    // Publicadores de pose para Field2d.
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    // Visualizaciones de Mechanism2d para cada uno de los 4 módulos swerve.
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };

    // Ligamentos para representar la velocidad de cada módulo en Mechanism2d. La longitud se escala según la velocidad relativa al máximo.
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };

    // Ligamentos para representar la dirección (ángulo) de cada módulo en Mechanism2d. La longitud es fija y el ángulo varía.
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    // Buffers temporales para registrar arrays de datos en SignalLogger.
    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /**
     * Publica datos de telemetría del estado swerve.
     * Este método debe llamarse periódicamente (por ejemplo, en periodic() o en un callback de odometría).
     *
     * Se realizan las siguientes acciones:
     * 1. Publica a NetworkTables la pose, velocidades, estados, targets y posiciones de los módulos.
     * 2. Envía datos en crudo a SignalLogger para registro offline.
     * 3. Publica la pose en formato adecuado para Field2d virtual.
     * 4. Actualiza las visualizaciones en Mechanism2d para cada módulo (ángulo y velocidad).
     *
     * @param state Estado actual de la unidad swerve, que incluye pose, velocidades, estados y posiciones de los módulos.
     */
    public void telemeterize(SwerveDriveState state) {
        // Publicar a NetworkTables (DriveState)
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        // Publicar a SignalLogger para registro offline
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }
        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        // Publicar la pose para visualización en un Field2d virtual
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        // Actualizar Mechanism2d para cada módulo (dirección y velocidad)
        for (int i = 0; i < 4; ++i) {
            // Actualiza el ángulo del módulo
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            // Escala la longitud del ligamento según la velocidad relativa al máximo
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
            // Publica la visualización de Mechanism2d en SmartDashboard
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}
