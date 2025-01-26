package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface for swerve module IO, defining methods for interacting with drive and turn motors. */
public interface ModuleIO {

  /**
   * Nested static class to hold input data from the module. Contains fields for drive and turn
   * motor states, as well as odometry information.
   */
  @AutoLog
  public static class ModuleIOInputs {
    // Drive motor connection status
    public boolean driveConnected = false;

    // Drive motor position in radians
    public double drivePositionRad = 0.0;

    // Drive motor velocity in radians per second
    public double driveVelocityRadPerSec = 0.0;

    // Drive motor applied voltage in volts
    public double driveAppliedVolts = 0.0;

    // Drive motor current in amperes
    public double driveCurrentAmps = 0.0;

    // Turn motor connection status
    public boolean turnConnected = false;

    // Turn motor position as a Rotation2d object
    public Rotation2d turnPosition = new Rotation2d();

    // Turn motor Encoder absolute position as a Rotation2d object
    public Rotation2d turnAbsolutePosition = new Rotation2d();

    // Turn motor velocity in radians per second
    public double turnVelocityRadPerSec = 0.0;

    // Turn motor applied voltage in volts
    public double turnAppliedVolts = 0.0;

    // Turn motor current in amperes
    public double turnCurrentAmps = 0.0;

    // Array of timestamps for odometry updates
    public double[] odometryTimestamps = new double[] {};

    // Array of drive positions in radians for odometry updates
    public double[] odometryDrivePositionsRad = new double[] {};

    // Array of turn positions as Rotation2d objects for odometry updates
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /**
   * Updates the set of loggable inputs with the current state of the module.
   *
   * @param inputs An instance of ModuleIOInputs to be populated with current motor data.
   */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Runs the drive motor at the specified open loop output.
   *
   * @param output Desired drive motor output (e.g., voltage or percentage).
   */
  public default void setDriveOpenLoop(double output) {}

  /**
   * Runs the turn motor at the specified open loop output.
   *
   * @param output Desired turn motor output (e.g., voltage or percentage).
   */
  public default void setTurnOpenLoop(double output) {}

  /**
   * Runs the drive motor at the specified velocity.
   *
   * @param velocityRadPerSec Desired drive velocity in radians per second.
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /**
   * Runs the turn motor to the specified rotation.
   *
   * @param rotation Desired turn motor position as a Rotation2d object.
   */
  public default void setTurnPosition(Rotation2d rotation) {}
}
