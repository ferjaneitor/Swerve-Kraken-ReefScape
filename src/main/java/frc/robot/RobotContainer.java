// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Drive.TunerConstants;
import frc.robot.Elevator.ElevatorCmd;
import frc.robot.Elevator.ElevatorCmdAuto;
import frc.robot.Elevator.ElevatorContinousCmd;
import frc.robot.Elevator.ElevatorResetPosition;
import frc.robot.Elevator.ElevatorSubSystem;
import frc.robot.Intakes.Algae.AlgaeEnableIntakeCmd;
import frc.robot.Intakes.Algae.AlgaePivotCmd;
import frc.robot.Intakes.Algae.AlgaeSubSystem;
import frc.robot.Intakes.Coral.CoralContinousInputCmd;
import frc.robot.Intakes.Coral.CoralPivotCmd;
import frc.robot.Intakes.Coral.CoralPivotResetPosition;
import frc.robot.Intakes.Coral.CoralSubSystem;
import frc.robot.LimeLight.VisionSubsystem;
import frc.robot.constants.CoralConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.DeepCage.DeepCageCmd;
import frc.robot.DeepCage.DeepCageSubSystem;
import frc.robot.Drive.CommandSwerveDrivetrain;

/**
 * RobotContainer
 * 
 * Esta clase configura la integración del robot, especialmente en proyectos FRC basados en WPILib.
 * Incluye:
 * - Creación y asignación de comandos por defecto al subsistema de swerve.
 * - Mapeo de botones del control de Xbox para distintas funciones: freno, orientación de ruedas, SysId y reinicio de heading.
 * - Selección de rutinas de autonomía (autoChooser) integradas con PathPlanner.
 * - Registro de telemetría en Telemetry para log de datos de swerve.
 * 
 * Esta clase se encarga de:
 *  1. Definir y configurar el subsistema principal de tracción (drivetrain) (Swerve).
 *  2. Asignar comandos por defecto y atajos de botones del control de Xbox.
 *  3. Configurar la selección de rutinas de autonomía usando AutoBuilder.
 *  4. Registrar telemetría personalizada.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.3
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
     * Control de Xbox, asignado en el puerto 0. Con este se conduce y maneja el robot
     */
    private final CommandXboxController driverController = new CommandXboxController(0);

    /**
     * Control de Xbox, asignado en el puerto 1. Con este se controlan los aditamentos
     */
    private final CommandXboxController AddOnsController = new CommandXboxController(1);

    /**
     * Subsistema principal de tracción swerve, creado desde los TunerConstants.
     */
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

     /**
     * Selector de rutinas de autonomía, utilizando {@link AutoBuilder}.
     */
    private final SendableChooser<Command> autoChooser;

    /**
     * Subsistema principal de la DeepCage
     */
    private final DeepCageSubSystem deepCageSubSystem = new DeepCageSubSystem();

    /**
     * Subsistema principal del Elevador
     */
    private final ElevatorSubSystem elevatorSubSystem = new ElevatorSubSystem();

    /**
     * Subsistema principal del mecanismo del alga
     */
    private final AlgaeSubSystem algeaSubSystem = new AlgaeSubSystem();    

    /**
     * Subsistema principal del mecanismo del Coral
     */
    private final CoralSubSystem coralSubSystem = new CoralSubSystem();

    @SuppressWarnings("unused")
    private final VisionSubsystem visionSubsystem;

    /**
     * Constructor principal de RobotContainer. Se encarga de configurar el mapeo de
     * botones y cargar las rutinas de autonomía disponibles.
     */
    public RobotContainer() {
        configureBindings();
        
        visionSubsystem = new VisionSubsystem();
        
        NamedCommands.registerCommand("Elevator to L4", new ElevatorCmdAuto(ElevatorConstants.L4,CoralConstants.angleL4, elevatorSubSystem, coralSubSystem));   
        NamedCommands.registerCommand("Elevator to L3", new ElevatorCmdAuto(ElevatorConstants.L3,CoralConstants.angleL3, elevatorSubSystem, coralSubSystem));   
        NamedCommands.registerCommand("Elevator to L2", new ElevatorCmdAuto(ElevatorConstants.L2,CoralConstants.angleL2, elevatorSubSystem, coralSubSystem));   
        NamedCommands.registerCommand("Elevator to L1", new ElevatorCmdAuto(ElevatorConstants.L1,CoralConstants.angleL1, elevatorSubSystem, coralSubSystem));   
        NamedCommands.registerCommand("Feeder Position", new ElevatorCmdAuto(ElevatorConstants.FeederHeight,CoralConstants.FeederAngle, elevatorSubSystem, coralSubSystem));   
        
        NamedCommands.registerCommand("Algae In", new AlgaeEnableIntakeCmd(false, algeaSubSystem));
        NamedCommands.registerCommand("Algae Out", new AlgaeEnableIntakeCmd(true, algeaSubSystem));
        
        NamedCommands.registerCommand("Coral In", new CoralContinousInputCmd(true, coralSubSystem));
        NamedCommands.registerCommand("Coral Out", new CoralContinousInputCmd(false, coralSubSystem));
        
        NamedCommands.registerCommand("Reset Elevator", new ElevatorResetPosition(elevatorSubSystem));
        
        NamedCommands.registerCommand("Reset Coral", new CoralPivotResetPosition(coralSubSystem));
        
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
        
        // Chasis
        
        // Comando por defecto: control field-centric con ejes de driverController
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * (driverController.leftBumper().getAsBoolean() == true? 0.2 : 0.8)) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * (driverController.leftBumper().getAsBoolean() == true? 0.2 : 0.8)) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * (driverController.leftBumper().getAsBoolean() == true? 0.2 : 0.8)) // Drive counterclockwise with negative X (left)
            )
        );

        // Bumper derecho: mantenerlo presionado activa el freno (SwerveDriveBrake)
        driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        
        // Trigger Izquierdo: mantenerlo presionado apunta las ruedas hacia la dirección del driverController izquierdo
        driverController.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Configuración de SysId con combinación de botones (back/start con X/Y)
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Boton y : Reiniciar el heading field-centric
        driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Registrar la función de telemetría para actualización periódica
        drivetrain.registerTelemetry(logger::telemeterize);
        
        //Bumper Derecho : A las manecillas del reloj se mueve el climber
        driverController.a().whileTrue(new DeepCageCmd(false, deepCageSubSystem));
        
        // Bumper Izquierdo : En contra de las manecillas del reloj se mueve el climber
        driverController.x().whileTrue(new DeepCageCmd(true, deepCageSubSystem));
        
        //Aditamentos
        
        //Boton y : Sacamos el Alga
        AddOnsController.y().whileTrue(new AlgaeEnableIntakeCmd(true, algeaSubSystem));
        
        //Boton x : chupamos el Alga
        AddOnsController.x().whileTrue(new AlgaeEnableIntakeCmd(false, algeaSubSystem));
        
        //Boton a : Chupamos el coral
        AddOnsController.a().whileTrue(new CoralContinousInputCmd(true, coralSubSystem));
        
        //Boton b : sacamos el coral
        AddOnsController.b().whileTrue(new CoralContinousInputCmd(false, coralSubSystem));
        
        //pov Arriba : Se extiende el Elevador hasta L4
        AddOnsController.povUp().toggleOnTrue(new ElevatorCmd(ElevatorConstants.L4, CoralConstants.angleL4, elevatorSubSystem, coralSubSystem));
        
        //pov Izquierda : Se extiende el Elevador hasta L1
        AddOnsController.povLeft().toggleOnTrue(new ElevatorCmd(ElevatorConstants.FeederHeight, CoralConstants.FeederAngle, elevatorSubSystem, coralSubSystem));
        
        //pov Derecho : Se extiende el Elevador hasta L3
        AddOnsController.povRight().toggleOnTrue(new ElevatorCmd(ElevatorConstants.L3, CoralConstants.angleL3, elevatorSubSystem, coralSubSystem));
        
        //pov Abajo : Se extiende el Elevador hasta L2
        AddOnsController.povDown().toggleOnTrue(new ElevatorCmd(ElevatorConstants.L2, CoralConstants.angleL2, elevatorSubSystem, coralSubSystem));
        
        //bumper derecho : Se extiende de manera continua el Elevador
        AddOnsController.rightBumper().whileTrue(new ElevatorContinousCmd(false, elevatorSubSystem));
        
        //bumper izquierdo : Se retrae de manera continua el elevador
        AddOnsController.leftBumper().whileTrue(new ElevatorContinousCmd(true, elevatorSubSystem));
        
        //Joystick Derecho Eje Y : Se controla que tanto va a pivotar el mecanismo de la alga
        algeaSubSystem.setDefaultCommand( new AlgaePivotCmd(true, algeaSubSystem, ()-> AddOnsController.getRightY()));
        
        // Joystick Izquierdo Eje Y : Se controla que tanto va a pivotar el mecanismo del Coral
        coralSubSystem.setDefaultCommand(new CoralPivotCmd(false, coralSubSystem, ()-> AddOnsController.getLeftY()));;

        // Trigger Derecho : Se posiciona el elevador y el coral para nada mas recivir del feeder
        //AddOnsController.rightTrigger().onTrue(new ElevatorTrapezoidCmd(ElevatorConstants.FeederHeight, CoralConstants.FeederAngle, elevatorSubSystem, coralSubSystem));
        
        //AddOnsController.povDown().onTrue(new CoralPivotPosition(CoralConstants.FeederAngle, coralSubSystem));
        
        //AddOnsController.povRight().onTrue(new CoralPivotPosition(CoralConstants.angleL1, coralSubSystem));
        
        //AddOnsController.povUp().onTrue(new CoralPivotPosition(CoralConstants.angleL4, coralSubSystem));
        
        //AddOnsController.povLeft().onTrue(new CoralPivotResetPosition(coralSubSystem));
        
        //algeaSubSystem.setDefaultCommand(new AlgaeEnableIndividualIntake(false, false, algeaSubSystem, () -> AddOnsController.getLeftTriggerAxis()));
        //algeaSubSystem.setDefaultCommand(new AlgaeEnableIndividualIntake(false, true, algeaSubSystem, () -> AddOnsController.getRightTriggerAxis()));
        
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
