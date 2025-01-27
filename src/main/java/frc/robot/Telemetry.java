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
 * Telemetry
 *
 * @Esta clase se encarga de publicar información de telemetría relacionada con
 * el sistema de conducción swerve. Envía datos tanto a NetworkTables (para ver en
 * SmartDashboard/Shuffleboard) como a un archivo de registro (utilizando
 * {@link SignalLogger}).
 *
 * <ul>
 *   <li><strong>SwerveDriveState:</strong> Objeto que contiene la pose actual,
 *       velocidades, estados y posiciones de los módulos swerve, entre otros
 *       datos.</li>
 *   <li><strong>NetworkTables:</strong> Publica la pose, velocidades y estados
 *       de los módulos en la tabla "DriveState" y también la pose para un
 *       "Field2d" virtual.</li>
 *   <li><strong>SignalLogger:</strong> Registra datos en un archivo de log para
 *       análisis posterior (por ejemplo, SysId, tuning, etc.).</li>
 *   <li><strong>Mechanism2d:</strong> Visualiza la orientación (ángulo) y
 *       velocidad de los módulos en SmartDashboard, usando ligamentos para
 *       representar la dirección y la velocidad relativa (con una longitud
 *       proporcional a la velocidad).</li>
 * </ul>
 *
 * <p>{@code Telemetry} encapsula la publicación de datos de la conducción swerve
 * en varios formatos y destinos. Principalmente envía:</p>
 *
 * <ol>
 *   <li>Estados de la unidad swerve a NetworkTables.</li>
 *   <li>Datos en crudo (arreglos de double) a {@link SignalLogger} para registro.</li>
 *   <li>Visualizaciones de ángulo y velocidad en {@code Mechanism2d} para
 *       cada uno de los cuatro módulos swerve.</li>
 * </ol>
 *
 * @Autor:  Fernando Joel Cruz Briones
 * @Versión: 1.0
 */

public class Telemetry {

    /**
     * Velocidad máxima del robot (m/s), usada para escalar la longitud
     * de los ligamentos de velocidad en Mechanism2d.
     */
    private final double MaxSpeed;

    /**
     * Construye un objeto de telemetría, especificando la velocidad máxima
     * del robot para la visualización adecuada de los módulos.
     *
     * @param maxSpeed Velocidad máxima (en m/s) de la unidad swerve.
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();
    }

    /* Instancia de NetworkTable para publicar datos de telemetría */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* --- Publicadores de DriveState --- */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* --- Publicadores de pose para Field2d --- */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /**
     * Visualizaciones de Mechanism2d, uno por cada módulo (4 módulos en total).
     */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    
    /**
     * Ligamentos para representar la velocidad de cada módulo en Mechanism2d.
     * La longitud se escala con base en la velocidad del módulo respecto a la
     * velocidad máxima.
     */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    
    /**
     * Ligamentos para representar la dirección de cada módulo en Mechanism2d
     * (ángulo). La longitud se mantiene constante, pero el ángulo varía.
     */
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

     /* Buffers temporales para registrar arrays de datos en SignalLogger */
    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /**
     * <p>Método principal para publicar datos del estado swerve {@link SwerveDriveState}
     * a NetworkTables, SignalLogger y Mechanism2d. Debería llamarse periódicamente 
     * (por ejemplo, en {@code periodic()} o en un callback de odometría).</p>
     *
     * @param state El estado actual de la unidad swerve, que contiene la pose,
     *              velocidades, estados y posiciones de los módulos, etc.
     */
    public void telemeterize(SwerveDriveState state) {
        
        /* 1. Publicar a NetworkTables (DriveState) */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* 2. Publicar a SignalLogger para registro offline */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* 3. Publicar la pose para visualización en un Field2d virtual */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* 4. Actualizar Mechanism2d para cada módulo (dirección y velocidad) */
        for (int i = 0; i < 4; ++i) {
            
            // Ángulo del módulo
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            
            // Escalar la longitud con relación a la velocidad máxima
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            // Publicar el Mechanism2d resultante en SmartDashboard
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }
}
