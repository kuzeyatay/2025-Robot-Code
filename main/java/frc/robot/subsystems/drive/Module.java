package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

/** Represents a single swerve module, handling both drive and turn functionalities. */
public class Module {
  // Instance of ModuleIO for interacting with the hardware (drive and turn motors)
  private final ModuleIO io;

  // Instance to hold and log inputs from the module
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  // Index of the module (e.g., Front-Left, Front-Right, etc.)
  private final int index;

  // Alerts to notify if the drive or turn motors are disconnected
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;

  // Array to store odometry positions for this module
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructor for the Module class.
   *
   * @param io An instance of ModuleIO to interface with the module's hardware.
   * @param index The index of the module (0 to 3).
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Initialize alerts for drive and turn motor disconnections
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  /**
   * Periodically updates the module's state, processes inputs, and manages alerts. This method
   * should be called regularly (e.g., in the robot's main loop).
   */
  public void periodic() {
    // Update inputs from the hardware via ModuleIO
    io.updateInputs(inputs);

    // Log the inputs for telemetry and debugging purposes
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry based on the number of samples
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];

    // Iterate through each sample to calculate the module's position
    for (int i = 0; i < sampleCount; i++) {
      // Convert drive position from radians to meters using the wheel radius
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;

      // Retrieve the turn angle for the current sample
      Rotation2d angle = inputs.odometryTurnPositions[i];

      // Create a new SwerveModulePosition with the calculated position and angle
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts based on the connection status of drive and turn motors
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /**
   * Runs the module with the specified setpoint state. This method optimizes the state before
   * applying it to the motors.
   *
   * @param state The desired state (speed and angle) for the swerve module.
   */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize the velocity setpoint based on the current angle to minimize rotation
    state.optimize(getAngle());

    // Apply cosine scaling to adjust for the current turn position (if applicable)
    state.cosineScale(inputs.turnPosition);

    // Set the drive velocity by converting meters per second to radians per second
    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);

    // Set the turn position to the desired angle
    io.setTurnPosition(state.angle);
  }

  /**
   * Runs the module with the specified output while controlling the turn angle to zero degrees.
   * This is typically used during characterization or testing.
   *
   * @param output The desired open-loop output for the drive motor.
   */
  public void runCharacterization(double output) {
    // Set the drive motor to open-loop control with the specified output
    io.setDriveOpenLoop(output);

    // Reset the turn motor to zero degrees
    io.setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to the motors, effectively stopping the module. */
  public void stop() {
    // Set the drive motor to zero output
    io.setDriveOpenLoop(0.0);

    // Set the turn motor to zero output
    io.setTurnOpenLoop(0.0);
  }

  /**
   * Retrieves the current turn angle of the module.
   *
   * @return The current Rotation2d angle of the module.
   */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /**
   * Retrieves the current drive position of the module in meters.
   *
   * @return The current drive position in meters.
   */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  /**
   * Retrieves the current drive velocity of the module in meters per second.
   *
   * @return The current drive velocity in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  /**
   * Retrieves the current position of the module as a SwerveModulePosition object.
   *
   * @return The current SwerveModulePosition.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * Retrieves the current state of the module as a SwerveModuleState object.
   *
   * @return The current SwerveModuleState.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Retrieves the array of module positions received during the current cycle.
   *
   * @return An array of SwerveModulePosition objects.
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * Retrieves the array of timestamps corresponding to the odometry samples received during the
   * current cycle.
   *
   * @return An array of timestamp doubles.
   */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * Retrieves the module's drive position in radians for characterization purposes.
   *
   * @return The drive position in radians.
   */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /**
   * Retrieves the module's drive velocity in radians per second for characterization purposes.
   *
   * @return The drive velocity in radians per second.
   */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
