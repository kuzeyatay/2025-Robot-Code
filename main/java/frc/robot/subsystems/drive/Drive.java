package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSetter;
import frc.robot.ModeSetter.Mode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Define the Drive class as a subsystem and implement the VisionConsumer interface
public class Drive extends SubsystemBase implements Vision.VisionConsumer {
  // Create a static lock for synchronizing odometry updates
  static final Lock odometryLock = new ReentrantLock();

  // Declare a GyroIO instance for interacting with the gyro sensor
  private final GyroIO gyroIO;
  // Create an instance to hold gyro inputs for logging
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // Declare an array of Module instances representing the four swerve modules (Front-Left,
  // Front-Right, Back-Left,
  // Back-Right)
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // Create an Alert to notify if the gyro is disconnected, with an error type
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Initialize the swerve drive kinematics with predefined module translations
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

  // Initialize the raw gyro rotation as a Rotation2d object
  private Rotation2d rawGyroRotation = new Rotation2d();

  // Create an array to store the last known positions of each swerve module for delta calculations
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // Initialize the pose estimator for tracking the robot's position on the field
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Declare a SwerveSetpointGenerator for generating setpoints based on desired chassis speeds
  private final SwerveSetpointGenerator setpointGenerator;
  // Store the previous setpoint to track changes over time
  private SwerveSetpoint previousSetpoint;

  // Constructor for the Drive class, initializing modules and configuring autonomous path planning
  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

    // Assign the provided GyroIO instance to the class's gyroIO variable
    this.gyroIO = gyroIO;
    // Initialize each swerve module with its respective ModuleIO and index
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Report the usage of a swerve drive mechanism to the HAL for telemetry
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start the odometry thread to continuously update the robot's position
    SparkOdometryThread.getInstance().start();

    // Configure the AutoBuilder for autonomous path following using PathPlanner
    AutoBuilder.configure(
        this::getPose, // Method to get the current pose of the robot
        this::resetOdometry, // Method to reset the robot's odometry
        this::getChassisSpeeds, // Method to get the current chassis speeds
        this::runVelocity, // Method to run the drive at desired velocity
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)), // Controller for holonomic drive
        ppConfig, // PathPlanner configuration settings
        () ->
            DriverStation.getAlliance().orElse(Alliance.Blue)
                == Alliance.Red, // Condition to determine alliance color
        this); // The Drive subsystem instance

    // Set the pathfinder to use a custom pathfinding algorithm
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Configure logging callbacks for active paths and target poses in PathPlanner
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Initialize the setpoint generator with PathPlanner configuration and maximum angular speed to
    // reduce
    // skidding.
    setpointGenerator =
        new SwerveSetpointGenerator(
            ppConfig, // The robot configuration used for generating trajectories and path following
            maxAngularSpeedRadiansPerSec // The maximum rotation velocity of a swerve module in
            // radians per second
            );

    // Initialize the previous setpoint with the current chassis speeds and module states, using
    // zero feedforwards
    ChassisSpeeds currentSpeeds =
        getChassisSpeeds(); // Method to get current robot-relative chassis speeds
    SwerveModuleState[] currentStates =
        getModuleStates(); // Method to get the current swerve module states
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds, currentStates, DriveFeedforwards.zeros(ppConfig.numModules));
  }

  // Override the periodic method, which is called regularly by the scheduler
  @Override
  public void periodic() {
    // Acquire the odometry lock to ensure thread safety during updates
    odometryLock.lock(); // Prevents odometry updates while reading data

    // Update gyro inputs from the gyroIO interface
    gyroIO.updateInputs(gyroInputs);
    // Process and log the gyro inputs for telemetry
    Logger.processInputs("Drive/Gyro", gyroInputs);

    // Iterate through each swerve module and perform periodic updates
    for (var module : modules) {
      module.periodic();
    }

    // Release the odometry lock after updates are complete
    odometryLock.unlock();

    // If the robot is disabled, stop all swerve modules
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // If the robot is disabled, log empty setpoint states to indicate no movement
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry by processing each sample timestamp from the modules
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Create arrays to hold the current positions and deltas for each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      // Iterate through each module to retrieve positions and calculate deltas
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        // Update the last known position for the module
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update the raw gyro rotation based on connectivity
      if (gyroInputs.connected) {
        // If the gyro is connected, use its current yaw position
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // If the gyro is disconnected, estimate rotation using kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Update the pose estimator with the current timestamp, gyro rotation, and module positions
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update the gyro disconnected alert based on gyro connectivity and current mode
    gyroDisconnectedAlert.set(!gyroInputs.connected && ModeSetter.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {

    // Generate new setpoints based on the previous setpoint, desired speeds, and control loop
    // period (0.02 seconds)
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    // Retrieve the swerve module states from the generated setpoints
    SwerveModuleState[] setpointStates = previousSetpoint.moduleStates();

    // Uncommented old method for generating setpoints using kinematics directly
    /* speeds.discretize(0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds); */

    // Ensure the wheel speeds do not exceed the maximum allowed speed
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log the unoptimized setpoints for telemetry
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send the calculated setpoints to each swerve module
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log the optimized setpoints after they have been potentially mutated by runSetpoint
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    // Iterate through each swerve module and run characterization with the specified output
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    // Run velocity with zero speeds to stop all modules
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    // Create an array to hold the desired headings for each module to form an X pattern
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      // Set each module's heading based on predefined module translations
      headings[i] = moduleTranslations[i].getAngle();
    }
    // Reset the kinematics with the new headings
    kinematics.resetHeadings(headings);
    // Stop all modules
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    // Create an array to hold the states of all four modules
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // Retrieve the state of each module
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    // Create an array to hold the positions of all four modules
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      // Retrieve the position of each module
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    // Convert the current module states to chassis-relative speeds
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    // Create an array to hold the wheel radius positions for characterization
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      // Retrieve the wheel radius position for each module
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    // Initialize the output to zero
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      // Sum the velocities from each module and average them
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    // Retrieve the estimated position of the robot from the pose estimator
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    // Retrieve the rotation component of the current pose
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    // Reset the pose estimator with the current gyro rotation, module positions, and the new pose
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  @Override
  public void accept(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Add a vision measurement to the pose estimator for improved accuracy
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    // Return the predefined maximum linear speed
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    // Calculate and return the maximum angular speed based on linear speed and drive base radius
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  /**
   * Returns the maximum angular acceleration in radians per second squared. This is derived from
   * the maximum linear acceleration and the drive base radius.
   */
  public double getChassisMaxAngularAccelerationRadPerSecSq() {
    // Return the predefined maximum angular acceleration
    return maxAngularAccelerationRadPerSecSq;
  }

  /** Returns the maximum linear acceleration in meters per second squared. */
  public double getChassisMaxAccelerationMetersPerSecSq() {
    // Return the predefined maximum linear acceleration
    return maxAccelerationMetersPerSecSq;
  }

  /** Returns the maximum angular velocity in radians per second. */
  public double getChassisMaxAngularVelocity() {
    // Return the predefined maximum angular velocity
    return maxAngularSpeedRadiansPerSec;
  }

  /** Returns the path constraints based on a speed multiplier. */
  public PathConstraints getChassisConstrains(double speedMultiplier) {
    // Create and return new path constraints with scaled maximum speeds and accelerations
    return new PathConstraints(
        getMaxLinearSpeedMetersPerSec() * speedMultiplier,
        getChassisMaxAccelerationMetersPerSecSq(),
        getChassisMaxAngularVelocity() * speedMultiplier,
        getChassisMaxAngularAccelerationRadPerSecSq());
  }
}
