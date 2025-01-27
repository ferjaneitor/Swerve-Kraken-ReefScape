// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Drive.TunerConstants;
import frc.robot.Drive.CommandSwerveDrivetrain;

/**
 * RobotContainer
 *
 * @Clase que configura la integración del robot, especialmente en proyectos FRC
 * basados en WPILib. Incluye:</p>
 *
 * <ul>
 *   <li>Creación y asignación de comandos por defecto al subsistema de swerve.</li>
 *   <li>Mapeo de botones del control de Xbox para distintas funciones:
 *       freno, orientación de ruedas, SysId y reinicio de heading.</li>
 *   <li>Selección de rutinas de autonomía (autoChooser) integradas con PathPlanner.</li>
 *   <li>Registro de telemetría en {@link Telemetry} para log de datos de swerve.</li>
 * </ul>
 *
 *<p>Esta clase {@code RobotContainer} se encarga de:</p>
 * <ol>
 *   <li>Definir y configurar el subsistema principal de tracción {@code drivetrain} (Swerve).
 *   <li>Asignar comandos por defecto y atajos de botones del control de Xbox.
 *   <li>Configurar la selección de rutinas de autonomía usando {@link AutoBuilder}.</li>
 *   <li>Registrar telemetría personalizada.</li>
 * </ol>
 *
 * @Autor:  Fernando Joel Cruz Briones</p>
 * @Versión: 1.0</p>
*/

public class RobotContainer {

    /**
     * Velocidad máxima (en m/s) correspondiente a {@link TunerConstants#kSpeedAt12Volts}.
     */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    
    /**
     * Velocidad angular máxima (en rad/s). En este caso se fija a 0.75 rotaciones por segundo.
     */
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /**
     * Request de Swerve para conducción Field-Centric (se provee la velocidad de traslación en X, Y
     * y la velocidad angular). También se configura el deadband y el tipo de control de los motores
     * (OpenLoopVoltage).
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    /**
     * Request de freno de Swerve. Hace que las ruedas se pongan a 90 grados y se aplique
     * una fuerza de retención en su lugar.
     */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
    /**
     * Request para apuntar las ruedas en cierta dirección en el plano (PointWheelsAt).
     */
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    /**
     * Objeto de telemetría que guarda y/o publica datos de la conducción swerve.
     */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * Control de Xbox, asignado en el puerto 0.
     */
    private final CommandXboxController joystick = new CommandXboxController(0);

    /**
     * Subsistema principal de tracción swerve, creado desde los TunerConstants.
     */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     /**
     * Selector de rutinas de autonomía, utilizando {@link AutoBuilder}.
     */
    private final SendableChooser<Command> autoChooser;

    /**
     * Constructor principal de RobotContainer. Se encarga de configurar el mapeo de
     * botones y cargar las rutinas de autonomía disponibles.
     */
    public RobotContainer() {
        configureBindings();
        
        // Carga del chooser para rutinas de PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser("Test");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
    }

    /**
     * Configura la asignación de botones y los comandos por defecto
     * para el drivetrain, incluyendo SysId, freno, orientación de ruedas
     * y reinicio de heading field-centric.
     */
    private void configureBindings() {
        
        // Comando por defecto: control field-centric con ejes de joystick
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Botón A: mantenerlo presionado activa el freno (SwerveDriveBrake)
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // Botón B: mantenerlo presionado apunta las ruedas hacia la dirección del joystick izquierdo
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Configuración de SysId con combinación de botones (back/start con X/Y)
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reiniciar el heading field-centric al presionar Left Bumper
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Registrar la función de telemetría para actualización periódica
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Retorna el comando que se ejecutará en modo autónomo, seleccionado
     * desde el {@link SendableChooser} en el SmartDashboard.
     *
     * @return Comando de autonomía actualmente seleccionado.
     */
    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
        
    }
}
