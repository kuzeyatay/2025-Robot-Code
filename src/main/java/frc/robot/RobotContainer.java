package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  List<Waypoint> inWay =
      PathPlannerPath.waypointsFromPoses(new Pose2d(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  PathConstraints inCo = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  public PathPlannerPath path =
      new PathPlannerPath(
          inWay,
          inCo,
          null, // The ideal starting state, this is only relevant for pre-planned
          // paths, so can be null for on-the-fly paths.

          new GoalEndState(
              0.0,
              Rotation2d.fromDegrees(
                  0)) // Goal end state. You can set a holonomic rotation here. If
          // using a differential drivetrain, the rotation will have no
          // effect.
          );

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
        this.vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

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
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
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
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        superstructure = new Superstructure(elevator, algeManipulator, coralWrist);

        break;
    }
    // Set up overrides
    superstructure.setDisabledOverride(superstructureDisable);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // NamedCommands.registerCommand("gyro reset", new InstantCommand(() -> ));

    NamedCommands.registerCommand(
        "coralStow", superstructure.runGoal(SuperstructureState.CORAL_STOW));

    NamedCommands.registerCommand(
        "coralL2", superstructure.runGoal(SuperstructureState.L2_CORAL_EJECT_AUTO));
    NamedCommands.registerCommand(
        "coralIntake", superstructure.runGoal(SuperstructureState.CORAL_INTAKE));
    NamedCommands.registerCommand(
        "coralL3Eject", superstructure.runGoal(SuperstructureState.L3_CORAL_EJECT));
    NamedCommands.registerCommand(
        "algeIntake", superstructure.runGoal(SuperstructureState.LIMBO_1_ALGE_INTAKE));
    NamedCommands.registerCommand(
        "eject", (new InstantCommand(() -> coralWrist.runFlywheelVelocity(-6500))));
    ;

    /* // registering commands for pathplanner
    NamedCommands.registerCommand("shootAtStart", shooter.launchAutoCommand());

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
    /*
       // Set up SysId routines
       autoChooser.addOption(           "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
       autoChooser.addOption(
           "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));

       // Set up SysId routines
       autoChooser.addOption("Elevatorerization", elevator.staticCharacterization(2.0));
    */
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () ->
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red
                    ? driver.getRawAxis(2)
                    : driver.getRawAxis(2)));
    /*
       driver
           .povUp()
           .toggleOnTrue(
               AutoBuilder.pathfindToPose(
                   new Pose2d(3.25, 4, Rotation2d.fromDegrees(360)),
                   new PathConstraints(
                       4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                   0));
    */
    /* driver
    .povLeft()
    .toggleOnTrue(
         AutoBuilder.followPath(path)
            new Pose2d(1.0, 7.0, Rotation2d.fromDegrees(120)),
            new PathConstraints(
                4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0); */

    /*  driver
    .button(1)
    .whileTrue(
        DriveCommands.aligningDrive(
            drive,
            () -> driver.getLeftY(), // X supplier for translation
            () -> driver.getLeftX(), // Y supplier for translation
            () -> driver.getRawAxis(2)));  */
    // Rotation from vision

    // Create a command that uses joystick inputs for translation,
    // but uses vision data to control the rotation.
    // Lock to 0° when A button is held
    /*
    driver
        .circle()
        .whileTrue(DriveCommands.driveToApriltag(drive, vision, () -> drive.getPose(), () -> 6.0)); */

    /*  driver
    .circle()
    .whileTrue(
        DriveCommands.alignDrive(
            drive, vision, () -> -driver.getLeftY(), () -> -driver.getLeftX())); */

    // Switch to X pattern when X button is pressed
    /* driver.a().onTrue(Commands.runOnce(drive::stopWithX, drive));



    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.



    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage


        driver
            .povRight()
            .toggleOnTrue(
                AutoBuilder.pathfindToPose(
                    new Pose2d(1, 1, Rotation2d.fromDegrees(500)),
                    new PathConstraints(
                        4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    0));



        /* driver.povUp().toggleOnTrue(
            Commands.runOnce(
        () -> {
          Pose2d currentPose = drive.getPose();

          // The rotation component in these poses represents the direction of travel
          Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
          Pose2d endPos =
          new Pose2d(3.25, 4, Rotation2d.fromDegrees(360));

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
        })); */

    /*     driver
    .povUp()
    .toggleOnTrue(
        Commands.run(
            () ->
                new DriveToPose(
                    drive,
                    () -> new Pose2d(3.25, 4, Rotation2d.fromDegrees(360)),
                    () -> drive.getPose()))); */

    // Reset gyro / odometry
    final Runnable resetGyro =
        ModeSetter.currentMode == ModeSetter.Mode.SIM
            ? () ->
                drive.resetOdometry(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose
            // during
            // simulation
            : () ->
                drive.resetOdometry(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    driver.circle().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    // Bind the triangle button to the PathGenerationCmd command.
    // When the triangle button is held, the command is continuously scheduled.

    // autoChooser.addOption("Elevator static", elevator.staticCharacterization(2.0));

    operator.a().whileTrue(coralWrist.homingSequence());

    operator.b().onTrue(superstructure.runGoal(SuperstructureState.CORAL_STOW));
    operator.x().onTrue(superstructure.runGoal(SuperstructureState.L2_CORAL_EJECT));

    operator
        .button(7)
        .onTrue(
            superstructure
                .runGoal(SuperstructureState.CORAL_INTAKE)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          Leds.getInstance().hpAttentionLeftAlert = true;
                          Leds.getInstance().hpAttentionRightAlert = false;
                        })));
    operator
        .button(8)
        .onTrue(
            superstructure
                .runGoal(SuperstructureState.CORAL_INTAKE)
                .alongWith(
                    new InstantCommand(
                        () -> {
                          Leds.getInstance().hpAttentionRightAlert = true;
                          Leds.getInstance().hpAttentionLeftAlert = false;
                        })));

    operator.leftBumper().onTrue(superstructure.runGoal(SuperstructureState.L3_CORAL_EJECT));
    operator.rightBumper().onTrue(superstructure.runGoal(SuperstructureState.L4_CORAL_EJECT));
    operator.button(10).onTrue(new InstantCommand(() -> coralWrist.runFlywheelVelocity(-7000)));

    operator.y().onTrue(superstructure.runGoal(SuperstructureState.GROUND_ALGE_INTAKE));
    operator.povDown().onTrue(superstructure.runGoal(SuperstructureState.SYSTEM_STOW));
    operator
        .povUp()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.PROCESSOR)
                .alongWith(new InstantCommand(() -> superstructure.setProcessing(true))));

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

    // driver.povUp().onTrue(Commands.runOnce(() -> pathOnFly(5.740, 4, 180, -180)));
    // driver.povLeft().onTrue(Commands.runOnce(() -> pathOnFly(3.855, 5.1, 120, -60)));
    // driver.povRight().onTrue(Commands.runOnce(() -> pathOnFly(3.855, 2.950, -120, 60)));
    // driver.povDown().onTrue(Commands.runOnce(() -> pathOnFly(3.250, 4, 300 - 180, 0)));
    // driver.R2().onTrue(Commands.runOnce(() -> pathOnFly(6, 0.5, 270, -90)));
    // driver.R1().onTrue(Commands.runOnce(() -> pathOnFly(1, 1, 180, -120)));
    // driver.L1().onTrue(Commands.runOnce(() -> pathOnFly(1, 7, 180, 120)));
    // driver.button(2).onTrue(new InstantCommand(() -> AutoBuilder.followPath(path).cancel()));
    // operator.button(10).whileTrue(new InstantCommand(() ->
    // coralWrist.runFlywheelVelocity(-8000)));
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
    // return new PathPlannerAuto("leave");

    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (ModeSetter.currentMode != ModeSetter.Mode.SIM) return;

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

    SmartDashboard.putData("Bot1c1a", new PathPlannerAuto("Bot1c1a"));
    SmartDashboard.putData("Top1c1a", new PathPlannerAuto("Top1c1a"));
    SmartDashboard.putData("Mid1c1a", new PathPlannerAuto("Mid1c1a"));
    SmartDashboard.putData("RBot1c1a", new PathPlannerAuto("RBot1c1a"));
    SmartDashboard.putData("RTop1c1a", new PathPlannerAuto("RTop1c1a"));
    SmartDashboard.putData("RMid1c1a", new PathPlannerAuto("RMid1c1a"));
    SmartDashboard.putData("RBot1c", new PathPlannerAuto("RBot1c"));
    SmartDashboard.putData("leave", new PathPlannerAuto("leave"));
    SmartDashboard.putData("chillin", new PathPlannerAuto("chillin"));

    autoChooser.addOption("leave", new PathPlannerAuto("leave"));
    autoChooser.addOption("chillin", new PathPlannerAuto("chillin"));
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic
    // rotation.

    /*   SmartDashboard.putData(
    "custom trajectory",
    Commands.runOnce(
        () -> {
          List<Waypoint> waypoints =
              PathPlannerPath.waypointsFromPoses(
                 drive.getPose(),

                  new Pose2d(3.250, 4, Rotation2d.fromDegrees(300-180)));

          PathConstraints constraints =
              new PathConstraints(
                  3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
          // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You
          // can also use unlimited constraints, only limited by motor torque and nominal
          // battery voltage

          // Create the path using the waypoints created above
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  constraints,
                  null, // The ideal starting state, this is only relevant for pre-planned
                  // paths, so can be null for on-the-fly paths.
                  new GoalEndState(
                      0.0,
                      Rotation2d.fromDegrees(
                          -90)) // Goal end state. You can set a holonomic rotation here. If
                  // using a differential drivetrain, the rotation will have no
                  // effect.
                  );

          // Prevent the path from being flipped if the coordinates are already correct
          path.preventFlipping = true;
          AutoBuilder.followPath(path).schedule();
        })); */

    SmartDashboard.putData(
        "cu1 trajectory 2 ",
        Commands.runOnce(
            () -> {
              List<Waypoint> waypoints =
                  PathPlannerPath.waypointsFromPoses(
                      drive.getPose(), new Pose2d(3.250, 4, Rotation2d.fromDegrees(300 - 180)));

              PathConstraints constraints =
                  new PathConstraints(
                      3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
              // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You
              // can also use unlimited constraints, only limited by motor torque and nominal
              // battery voltage

              // Create the path using the waypoints created above
              PathPlannerPath path =
                  new PathPlannerPath(
                      waypoints,
                      constraints,
                      null, // The ideal starting state, this is only relevant for pre-planned
                      // paths, so can be null for on-the-fly paths.

                      new GoalEndState(
                          0.0,
                          Rotation2d.fromDegrees(
                              0)) // Goal end state. You can set a holonomic rotation here. If
                      // using a differential drivetrain, the rotation will have no
                      // effect.
                      );

              // Prevent the path from being flipped if the coordinates are already correct
              path.preventFlipping = true;
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
    SmartDashboard.putData(
        " Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(1.0, 7.0, Rotation2d.fromDegrees(120)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0));
  }
  /*
  double limelight_aim_proportional() {
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control
    // loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = (LimelightHelpers.getTX("limelight")) * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= 5;

    // invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for
  // target ranging rather than "ty"
  double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= 3;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  } */
  public void pathOnFly(double x, double y, double heading, double endAngle) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            drive.getPose(), new Pose2d(x, y, Rotation2d.fromDegrees(heading)));

    PathConstraints constraints =
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
    // Youee"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""fvv
    // can also use unlimited constraints, only limited by motor torque and nominal
    // battery voltage

    // Create the path using the waypoints created above
    path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned
            // paths, so can be null for on-the-fly paths.

            new GoalEndState(
                0.0,
                Rotation2d.fromDegrees(
                    endAngle)) // Goal end state. You can set a holonomic rotation here.
            // If
            // using a differential drivetrain, the rotation will have no
            // effect.
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = false;
    AutoBuilder.followPath(path).schedule();
  }
}
