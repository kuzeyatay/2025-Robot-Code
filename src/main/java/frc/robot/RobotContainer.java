package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulator;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIO;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIOKrakenFOC;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIOSim;
import frc.robot.subsystems.superstructure.algeManipulatorFlywheels.AlgeManipulatorFlywheels;
import frc.robot.subsystems.superstructure.algeManipulatorFlywheels.AlgeManipulatorFlywheelsIO;
import frc.robot.subsystems.superstructure.algeManipulatorFlywheels.AlgeManipulatorFlywheelsSim;
import frc.robot.subsystems.superstructure.algeManipulatorFlywheels.AlgeManipulatorFlywheelsSparkMax;
import frc.robot.subsystems.superstructure.coralFlywheels.CoralFlywheels;
import frc.robot.subsystems.superstructure.coralFlywheels.CoralFlywheelsIO;
import frc.robot.subsystems.superstructure.coralFlywheels.CoralFlywheelsSim;
import frc.robot.subsystems.superstructure.coralFlywheels.CoralFlywheelsSparkMax;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.elevatorFlywheels.ElevatorFlywheels;
import frc.robot.subsystems.superstructure.elevatorFlywheels.ElevatorFlywheelsIO;
import frc.robot.subsystems.superstructure.elevatorFlywheels.ElevatorFlywheelsIOSim;
import frc.robot.subsystems.superstructure.elevatorFlywheels.ElevatorFlywheelsIOSparkMax;
import frc.robot.subsystems.superstructure.elevatorFlywheels.TopElevatorFlywheels;
import frc.robot.subsystems.superstructure.elevatorFlywheels.TopElevatorFlywheelsIO;
import frc.robot.subsystems.superstructure.elevatorFlywheels.TopElevatorFlywheelsIOSim;
import frc.robot.subsystems.superstructure.elevatorFlywheels.TopElevatorFlywheelsIOSparkMax;
import frc.robot.subsystems.vision.*;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Vision vision;
  private final Elevator elevator;
  private final AlgeManipulator algeManipulator;
  private final AlgeManipulatorFlywheels algeManipulatorFlywheels;
  private final ElevatorFlywheels elevatorFlywheel;
  private final TopElevatorFlywheels topElevatorFlywheel;
  private final CoralFlywheels coralFlywheels;

  // ivate final CoralFlywheel coralFlywheel;
  // rivate final CoralWrist coralWrist;
  // ivate final Flywheels flywheels;

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (ModeSetter.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        elevator = new Elevator(new ElevatorIOTalonFX());
        algeManipulator = new AlgeManipulator(new AlgeManipulatorIOKrakenFOC());
        algeManipulatorFlywheels =
            new AlgeManipulatorFlywheels(new AlgeManipulatorFlywheelsSparkMax());
        elevatorFlywheel = new ElevatorFlywheels(new ElevatorFlywheelsIOSparkMax());
        topElevatorFlywheel = new TopElevatorFlywheels(new TopElevatorFlywheelsIOSparkMax());
        coralFlywheels = new CoralFlywheels(new CoralFlywheelsSparkMax());
        // oralWrist = new CoralWrist(new CoralWristIOKrakenFOC());
        // ralFlywheel = new CoralFlywheel(new CoralFlywheelIOSparkMax());
        // ywheels = new Flywheels(new FlywheelsIOSparkMax());
        break;
      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        elevator = new Elevator(new ElevatorIOSim());
        algeManipulator = new AlgeManipulator(new AlgeManipulatorIOSim());
        algeManipulatorFlywheels = new AlgeManipulatorFlywheels(new AlgeManipulatorFlywheelsSim());
        elevatorFlywheel = new ElevatorFlywheels(new ElevatorFlywheelsIOSim());
        topElevatorFlywheel = new TopElevatorFlywheels(new TopElevatorFlywheelsIOSim());
        coralFlywheels = new CoralFlywheels(new CoralFlywheelsSim());

        // ralWrist = new CoralWrist(new CoralWristIOSim());
        // ralFlywheel = new CoralFlywheel(new CoralFlywheelIOSim());
        // ywheels = new Flywheels(new FlywheelsIOSim());
        // AIRobotInSimulation.startOpponentRobotSimulations();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        algeManipulator = new AlgeManipulator(new AlgeManipulatorIO() {});
        algeManipulatorFlywheels =
            new AlgeManipulatorFlywheels(new AlgeManipulatorFlywheelsIO() {});
        elevatorFlywheel = new ElevatorFlywheels(new ElevatorFlywheelsIO() {});
        topElevatorFlywheel = new TopElevatorFlywheels(new TopElevatorFlywheelsIO() {});
        coralFlywheels = new CoralFlywheels(new CoralFlywheelsIO() {});
        // ralWrist = new CoralWrist(new CoralWristIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    /* // registering commands for pathplanner
    NamedCommands.registerCommand("shootAtStart", shooter.launchAutoCommand());
    // NamedCommands.registerCommand("inAndOut", new feedAndStopAuto(intake, shooter, arm));
    // NamedCommands.registerCommand("shooterFeed", new feedAndStop(intake, shooter));
    NamedCommands.registerCommand("armOpen", arm.autoSubwoofer());
    NamedCommands.registerCommand("armIn", arm.Neutral());
    NamedCommands.registerCommand("shooterAtPos", arm.autoSubwoofer());
    NamedCommands.registerCommand("distanceShoot", arm.autoShootFromDistance());
    NamedCommands.registerCommand(
        "Speaker Pos", arm.subwooferShoot().alongWith(Commands.run(() -> shooter.subwooferMode()))); */

    // Configure the button bindings
    configureButtonBindings();

    // initiate Pathplanner
    Pathplanner();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

    // Set up SysId routines
    autoChooser.addOption("Elevatorerization", elevator.staticCharacterization(2.0));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRawAxis(2)));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d(0)));

    autoChooser.addOption("Elevator static", elevator.staticCharacterization(2.0));

    // Elevator Test
    controller.b().onTrue(elevator.homingSequence());
    controller.y().onTrue(new InstantCommand(() -> elevator.setGoal(Elevator.Goal.L2)));
    // controller
    //  .a()
    // .whileTrue(
    //   Commands.startEnd(
    //   () -> coralFlywheel.runVelocity(4000), coralFlywheel::stop, coralFlywheel));

    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> algeManipulatorFlywheels.runVelocity(-1000),
                algeManipulatorFlywheels::stop,
                algeManipulatorFlywheels));

    controller
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> coralFlywheels.runVolts(9), coralFlywheels::stop, coralFlywheels));

    controller
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> elevatorFlywheel.runVelocity(1000),
                elevatorFlywheel::stop,
                elevatorFlywheel));

    controller
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> topElevatorFlywheel.runVelocity(-1000),
                topElevatorFlywheel::stop,
                topElevatorFlywheel));

    controller
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> elevatorFlywheel.runVelocity(-1000),
                elevatorFlywheel::stop,
                elevatorFlywheel));

    controller
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> topElevatorFlywheel.runVelocity(1000),
                topElevatorFlywheel::stop,
                topElevatorFlywheel));

    controller
        .a()
        .onTrue(new InstantCommand(() -> algeManipulator.setAngle(AlgeManipulator.Goal.PROCESSOR)));

    controller.x().onTrue(algeManipulator.homingSequence());

    /*ontroller
    .x()
    .onTrue(new InstantCommand(() -> algeManipulator.setGoal(AlgeManipulator.Goal.STOW)));*/
    // controller.x().onTrue(new InstantCommand(() -> elevator.staticCharacterization(2.0)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro / odometry
    final Runnable resetGyro =
        ModeSetter.currentMode == ModeSetter.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  public Command CoralIntakeSequence() {
    return Commands.startRun(
            () -> {
              algeManipulator.setAngle(AlgeManipulator.Goal.GROUND_ALGE_INTAKE);
              algeManipulatorFlywheels.runVelocity(-1000);
            },
            () -> {
              // detected =

              // laserCan.laserDistance();
            })
        .until(() -> false)
        .andThen(() -> {})
        .andThen(() -> {});
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (ModeSetter.currentMode != ModeSetter.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (ModeSetter.currentMode != ModeSetter.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
    // Logger.recordOutput(
    //   "FieldSimulation/OpponentRobotPositions", AIRobotInSimulation.getOpponentRobotPoses());
    // Logger.recordOutput(
    //    "FieldSimulation/AlliancePartnerRobotPositions",
    //  AIRobotInSimulation.getAlliancePartnerRobotPoses());
  }

  private void Pathplanner() {

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser
    // built above
    SmartDashboard.putData("Copy of L2 x 2 Net 1.2", new PathPlannerAuto("Copy of L2 x 2 Net 1.2"));
    SmartDashboard.putData("Copy of L2 x 2 Net 1.3", new PathPlannerAuto("Copy of L2 x 2 Net 1.3"));
    SmartDashboard.putData("Copy of L2 x 2 Net 2.2", new PathPlannerAuto("Copy of L2 x 2 Net 2.2"));
    SmartDashboard.putData("Copy of L2 x 2 Net 2.3", new PathPlannerAuto("Copy of L2 x 2 Net 2.3"));
    SmartDashboard.putData("Copy of L2 x 2 Net 2", new PathPlannerAuto("Copy of L2 x 2 Net 2"));
    SmartDashboard.putData("Copy of L2 x 2 Net 3.2", new PathPlannerAuto("Copy of L2 x 2 Net 3.2"));
    SmartDashboard.putData("Copy of L2 x 2 Net 3.3", new PathPlannerAuto("Copy of L2 x 2 Net 3.3"));
    SmartDashboard.putData("Copy of L2 x 2 Net 3", new PathPlannerAuto("Copy of L2 x 2 Net 3"));
    SmartDashboard.putData("Copy of L2 x 2 Net", new PathPlannerAuto("Copy of L2 x 2 Net"));
    SmartDashboard.putData("Copy of L2 x 3 2", new PathPlannerAuto("Copy of L2 x 3 2"));
    SmartDashboard.putData("Copy of L2 x 3 3", new PathPlannerAuto("Copy of L2 x 3 3"));
    SmartDashboard.putData("Copy of L2 x 3", new PathPlannerAuto("Copy of L2 x 3"));
    SmartDashboard.putData("Copy of L2 x 3 2", new PathPlannerAuto("Copy of L2 x 3 2"));
    SmartDashboard.putData("Copy of L3 L4 Net 1.2", new PathPlannerAuto("Copy of L3 L4 Net 1.2"));
    SmartDashboard.putData("Copy of L3 L4 Net 1.3", new PathPlannerAuto("Copy of L3 L4 Net 1.3"));
    SmartDashboard.putData("Copy of L3 L4 Net 2.2", new PathPlannerAuto("Copy of L3 L4 Net 2.2"));
    SmartDashboard.putData("Copy of L3 L4 Net 2", new PathPlannerAuto("Copy of L3 L4 Net 2"));
    SmartDashboard.putData("Copy of L3 L4 Net 3.2", new PathPlannerAuto("Copy of L3 L4 Net 3.2"));
    SmartDashboard.putData("Copy of L3 L4 Net 2.3", new PathPlannerAuto("Copy of L3 L4 Net 2.3"));
    SmartDashboard.putData("Copy of L3 L4 Net 3.3", new PathPlannerAuto("Copy of L3 L4 Net 3"));
    SmartDashboard.putData("Copy of L3 L4 Net 3", new PathPlannerAuto("Copy of L3 L4 Net 3"));
    SmartDashboard.putData("Copy of L3 L4 Net", new PathPlannerAuto("Copy of L3 L4 Net"));
    SmartDashboard.putData(
        "Copy of L3 x 2 L4 x 1 2", new PathPlannerAuto("Copy of L3 x 2 L4 x 1 2"));
    SmartDashboard.putData(
        "Copy of L3 x 1 L4 x 2 3", new PathPlannerAuto("Copy of L3 x 1 L4 x 2 3"));
    SmartDashboard.putData("Copy of L3 x 2 L4 x 1", new PathPlannerAuto("Copy of L3 x 2 L4 x 1"));
    SmartDashboard.putData(
        "Copy of L3 x 2 L4 x 1 3", new PathPlannerAuto("Copy of L3 x 2 L4 x 1 3"));
    SmartDashboard.putData(
        "Copy of L3 x 1 L4 x 2 2", new PathPlannerAuto("Copy of L3 x 1 L4 x 2 2"));
    SmartDashboard.putData("Copy of L3 x 1 L4 x 2", new PathPlannerAuto("Copy of L3 x 1 L4 x 2"));
    SmartDashboard.putData("Copy of L4 x 2 Net 2.2", new PathPlannerAuto("Copy of L4 x 2 Net 2.2"));
    SmartDashboard.putData("Copy of L4 x 2 Net 1.2", new PathPlannerAuto("Copy of L4 x 2 Net 1.2"));
    SmartDashboard.putData("Copy of L4 x 2 Net 1.3", new PathPlannerAuto("Copy of L4 x 2 Net 1.3"));
    SmartDashboard.putData("Copy of L4 x 2 Net 2.2", new PathPlannerAuto("Copy of L4 x 2 Net 2.2"));
    SmartDashboard.putData("Copy of L4 x 2 Net 1.2", new PathPlannerAuto("Copy of L4 x 2 Net 1.2"));
    SmartDashboard.putData("Copy of L4 x 2 Net 1.3", new PathPlannerAuto("Copy of L4 x 2 Net 1.3"));
    SmartDashboard.putData("Copy of L3 x 3 3", new PathPlannerAuto("Copy of L3 x 3 3"));
    SmartDashboard.putData("Copy of L3 x 3", new PathPlannerAuto("Copy of L3 x 3"));
    SmartDashboard.putData("Copy of L3 x 3 2", new PathPlannerAuto("Copy of L3 x 3 2"));
    SmartDashboard.putData("L2 x 2 Net 2.2", new PathPlannerAuto("L2 x 2 Net 2.2"));
    SmartDashboard.putData("L2 x 2 Net 1.3", new PathPlannerAuto("L2 x 2 Net 1.3"));
    SmartDashboard.putData("L2 x 2 Net 1.2", new PathPlannerAuto("L2 x 2 Net 1.2"));
    SmartDashboard.putData("Copy of L4 x 3", new PathPlannerAuto("Copy of L4 x 3"));
    SmartDashboard.putData("Copy of L4 x 3 3", new PathPlannerAuto("Copy of L4 x 3 3"));
    SmartDashboard.putData("Copy of L4 x 2 Net 2.3", new PathPlannerAuto("Copy of L4 x 2 Net 2.3"));
    SmartDashboard.putData("Copy of L4 x 3 2", new PathPlannerAuto("Copy of L4 x 3 2"));
    SmartDashboard.putData("L2 x 3 2", new PathPlannerAuto("L2 x 3 2"));
    SmartDashboard.putData("L2 x 2 Net", new PathPlannerAuto("L2 x 2 Net"));
    SmartDashboard.putData("L2 x 2 Net 3.3", new PathPlannerAuto("L2 x 2 Net 3.3"));
    SmartDashboard.putData("L2 x 2 Net 3", new PathPlannerAuto("L2 x 2 Net 3"));
    SmartDashboard.putData("L2 x 2 Net 3.2", new PathPlannerAuto("L2 x 2 Net 3.2"));
    SmartDashboard.putData("L2 x 2 Net 2", new PathPlannerAuto("L2 x 2 Net 2"));
    SmartDashboard.putData("L2 x 2 Net 2.3", new PathPlannerAuto("L2 x 2 Net 2.3"));
    SmartDashboard.putData("L3 L4 Net 2", new PathPlannerAuto("L3 L4 Net 2"));
    SmartDashboard.putData("L3 L4 Net 2.3", new PathPlannerAuto("L3 L4 Net 2.3"));
    SmartDashboard.putData("L3 L4 Net 2.2", new PathPlannerAuto("L3 L4 Net 2.2"));
    SmartDashboard.putData("L2 x 3", new PathPlannerAuto("L2 x 3"));
    SmartDashboard.putData("L3 L4 Net 1.2", new PathPlannerAuto("L3 L4 Net 1.2"));
    SmartDashboard.putData("L2 x 3 3", new PathPlannerAuto("L2 x 3 3"));
    SmartDashboard.putData("L3 L4 Net 1.3", new PathPlannerAuto("L3 L4 Net 1.3"));
    SmartDashboard.putData("L3 L4 Net 3.2", new PathPlannerAuto("L3 L4 Net 3.2"));
    SmartDashboard.putData("L3 L4 Net 3.3", new PathPlannerAuto("L3 L4 Net 3.3"));
    SmartDashboard.putData("L3 L4 Net 3", new PathPlannerAuto("L3 L4 Net 3"));
    SmartDashboard.putData("L3 L4 Net", new PathPlannerAuto("L3 L4 Net"));
    SmartDashboard.putData("L3 x 1 L4 x 2 2", new PathPlannerAuto("L3 x 1 L4 x 2 2"));
    SmartDashboard.putData("L3 x 1 L4 x 2 3", new PathPlannerAuto("L3 x 1 L4 x 2 3"));
    SmartDashboard.putData("L3 x 1 L4 x 2", new PathPlannerAuto("L3 x 1 L4 x 2"));
    SmartDashboard.putData("L3 x 2 L4 x 1 2", new PathPlannerAuto("L3 x 2 L4 x 1 2"));
    SmartDashboard.putData("L3 x 2 L4 x 1 3", new PathPlannerAuto("L3 x 2 L4 x 1 3"));
    SmartDashboard.putData("L3 x 2 L4 x 1", new PathPlannerAuto("L3 x 2 L4 x 1"));
    SmartDashboard.putData("L3 x 3 2", new PathPlannerAuto("L3 x 3 2"));
    SmartDashboard.putData("L3 x 3 3", new PathPlannerAuto("L3 x 3 3"));
    SmartDashboard.putData("L3 x 3", new PathPlannerAuto("L3 x 3"));
    SmartDashboard.putData("L3 x 3 3", new PathPlannerAuto("L3 x 3 3"));
    SmartDashboard.putData("L3 x 3", new PathPlannerAuto("L3 x 3"));
    SmartDashboard.putData("L4 x 2 Net 1.2", new PathPlannerAuto("L4 x 2 Net 1.2"));
    SmartDashboard.putData("L4 x 2 Net 1.3", new PathPlannerAuto("L4 x 2 Net 1.3"));
    SmartDashboard.putData("L4 x 2 Net 2.3", new PathPlannerAuto("L4 x 2 Net 2.3"));
    SmartDashboard.putData("L4 X 2 Net 2", new PathPlannerAuto("L4 X 2 Net 2"));
    SmartDashboard.putData("L4 x 2 Net 3.2", new PathPlannerAuto("L4 x 2 Net 3.2"));
    SmartDashboard.putData("L4 x 2 Net 3.3", new PathPlannerAuto("L4 x 2 Net 3.3"));
    SmartDashboard.putData("L4 x 2 Net 3", new PathPlannerAuto("L4 x 2 Net 3"));
    SmartDashboard.putData("L4 x 3 2", new PathPlannerAuto("L4 x 3 2"));
    SmartDashboard.putData("L4 x 3 2", new PathPlannerAuto("L4 x 3 3"));
    SmartDashboard.putData("L4 x 3", new PathPlannerAuto("L4 x 3"));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m forward of its current position
    SmartDashboard.putData(
        "custom trajectory",
        Commands.runOnce(
            () -> {
              Pose2d currentPose = drive.getPose();

              // The rotation component in these poses represents the direction of travel
              Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
              Pose2d endPos =
                  new Pose2d(new Translation2d(1.823, 0.672), Rotation2d.fromRotations(0));

              List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
              PathPlannerPath path =
                  new PathPlannerPath(
                      waypoints,
                      new PathConstraints(
                          4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                      new IdealStartingState(0.0, Rotation2d.fromRotations(0)),
                      new GoalEndState(0.0, Rotation2d.fromRotations(0)));

              // Prevent this path from being flipped on the red alliance, since the given positions
              // CHANGED TO FALSE
              // are already correct
              path.preventFlipping = false;
              AutoBuilder.followPath(path).schedule();
            }));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData(
        "Pathfind to Pickup Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(14.77, 1.25, Rotation2d.fromDegrees(-140)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0));

    SmartDashboard.putData(
        "Pathfind to Scoring Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0));
  }
}
