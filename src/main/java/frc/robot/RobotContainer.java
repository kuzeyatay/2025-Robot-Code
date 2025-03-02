package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIO;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIOSim;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIOSparkMax;
import frc.robot.subsystems.laserCan.LaserCanIO;
import frc.robot.subsystems.laserCan.LaserCanIOLaserCan;
import frc.robot.subsystems.laserCan.LaserCanIOSim;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulator;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIO;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIOKrakenFOC;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorIOSim;
import frc.robot.subsystems.superstructure.coralWrist.CoralWrist;
import frc.robot.subsystems.superstructure.coralWrist.CoralWristIO;
import frc.robot.subsystems.superstructure.coralWrist.CoralWristIOKrakenFOC;
import frc.robot.subsystems.superstructure.coralWrist.CoralWristIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.util.OverrideSwitches;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
  private final Superstructure superstructure;

  // private final CoralFlywheel coralFlywheel;
  private final CoralWrist coralWrist;
  // ivate final Flywheels flywheels;

  private SwerveDriveSimulation driveSimulation = null;

  private final CommandPS5Controller driver = new CommandPS5Controller(1);
  // Controller
  private final CommandXboxController operator = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Controllers
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger superstructureCoast = overrides.driverSwitch(2);
  private final Trigger superstructureDisable = overrides.driverSwitch(1);
  private boolean superstructureCoastOverride = false;
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

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
                drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(),
                new GenericFlywheelsIOSparkMax(34, 24 / 15, false),
                new GenericFlywheelsIOSparkMax(33, 24 / 15, false));
        algeManipulator =
            new AlgeManipulator(
                new AlgeManipulatorIOKrakenFOC(),
                new GenericFlywheelsIOSparkMax(18, 42 / 18, false),
                new LaserCanIOLaserCan(42));

        coralWrist =
            new CoralWrist(
                new CoralWristIOKrakenFOC(),
                new GenericFlywheelsIOSparkMax(17, 20 / 15, true),
                new LaserCanIOLaserCan(41),
                new LaserCanIOLaserCan(40));
        // ralFlywheel = new CoralFlywheel(new CoralFlywheelIOSparkMax());
        // ywheels = new Flywheels(new FlywheelsIOSparkMax());

        superstructure = new Superstructure(elevator, algeManipulator, coralWrist);
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

        elevator =
            new Elevator(
                new ElevatorIOSim(), new GenericFlywheelsIOSim(), new GenericFlywheelsIOSim());
        algeManipulator =
            new AlgeManipulator(
                new AlgeManipulatorIOSim(), new GenericFlywheelsIOSim(), new LaserCanIOSim());

        coralWrist =
            new CoralWrist(
                new CoralWristIOSim(),
                new GenericFlywheelsIOSim(),
                new LaserCanIOSim(),
                new LaserCanIOSim());
        // ralFlywheel = new CoralFlywheel(new CoralFlywheelIOSim());
        // ywheels = new Flywheels(new FlywheelsIOSim());
        // AIRobotInSimulation.startOpponentRobotSimulations();

        superstructure = new Superstructure(elevator, algeManipulator, coralWrist);

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
        elevator =
            new Elevator(
                new ElevatorIO() {}, new GenericFlywheelsIO() {}, new GenericFlywheelsIO() {});

        algeManipulator =
            new AlgeManipulator(
                new AlgeManipulatorIO() {}, new GenericFlywheelsIO() {}, new LaserCanIO() {});

        coralWrist =
            new CoralWrist(
                new CoralWristIO() {},
                new GenericFlywheelsIO() {},
                new LaserCanIO() {},
                new LaserCanIO() {});

        superstructure = new Superstructure(elevator, algeManipulator, coralWrist);

        break;
    }
    // Set up overrides
    superstructure.setDisabledOverride(superstructureDisable);

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
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRawAxis(2)));

    // Lock to 0° when A button is held
    /*    driver
    .circle()
    .whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> new Rotation2d(0))); */

    // Switch to X pattern when X button is pressed
    driver.triangle().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    driver.square().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // autoChooser.addOption("Elevator static", elevator.staticCharacterization(2.0));

    operator.a().onTrue(superstructure.runGoal(SuperstructureState.CORAL_INTAKE));
    operator.b().onTrue(superstructure.runGoal(SuperstructureState.CORAL_STOW));
    operator.x().onTrue(superstructure.runGoal(SuperstructureState.L2_CORAL_EJECT));
    operator.leftBumper().onTrue(superstructure.runGoal(SuperstructureState.L3_CORAL_EJECT));
    operator.rightBumper().onTrue(superstructure.runGoal(SuperstructureState.L4_CORAL_EJECT));

    operator.button(7).whileTrue(coralWrist.homingSequence());
    operator.y().onTrue(superstructure.runGoal(SuperstructureState.GROUND_ALGE_INTAKE));
    operator.povDown().onTrue(superstructure.runGoal(SuperstructureState.SYSTEM_STOW));
    operator.povUp().onTrue(superstructure.runGoal(SuperstructureState.PROCESSOR));
    operator.povLeft().onTrue(superstructure.runGoal(SuperstructureState.LIMBO_1_ALGE_INTAKE));
    operator.povRight().onTrue(superstructure.runGoal(SuperstructureState.LIMBO_2_ALGE_INTAKE));
    superstructureCoast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        superstructureCoastOverride = true;
                        Leds.getInstance().superstructureCoast = true;
                      }
                    })
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                      Leds.getInstance().superstructureCoast = false;
                    })
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                      Leds.getInstance().superstructureCoast = false;
                    })
                .ignoringDisable(true));

    /*  // Strobe Leds.getInstance() at human player
       operator
           .b()
           .whileTrue(
               Commands.startEnd(
                   () -> Leds.getInstance().hpAttentionAlert = true, () -> Leds.getInstance().hpAttentionAlert = false));
    */
    // Endgame Alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .beforeStarting(() -> Leds.getInstance().endgameAlert = true)
                .finallyDo(() -> Leds.getInstance().endgameAlert = false)); // Rumble three times
  }
  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
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
            new Pose2d(1.501, 7.0906, Rotation2d.fromDegrees(162)),
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
