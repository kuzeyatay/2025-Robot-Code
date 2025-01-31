package frc.robot.subsystems.elevatorFlywheels;

import org.littletonrobotics.junction.AutoLog;

/**
 * FlywheelIO interface defines the hardware interactions for the flywheel subsystem. It allows for
 * updating sensor data, running the flywheel in open or closed loop, stopping the flywheel, and
 * configuring PID constants.
 */
public interface ElevatorFlywheelIO {
  // AutoLog annotation indicates that the FlywheelIOInputs class fields will be automatically
  // logged
  @AutoLog
  public static class ElevatorFlywheelIOInputs {
    // The current angular position of the top flywheel in radians
    public double topPositionRad = 0.0;

    // The current angular velocity of the top flywheel in radians per second
    public double topVelocityRadPerSec = 0.0;

    // The current applied voltage to the top flywheel motor
    public double topAppliedVolts = 0.0;

    // An array representing the current draw of the top flywheel motor(s) in amps
    public double[] topCurrentAmps = new double[] {};

    // The current angular position of the bottom flywheel in radians
    public double bottomPositionRad = 0.0;

    // The current angular velocity of the bottom flywheel in radians per second
    public double bottomVelocityRadPerSec = 0.0;

    // The current applied voltage to the bottom flywheel motor
    public double bottomAppliedVolts = 0.0;

    // An array representing the current draw of the bottom flywheel motor(s) in amps
    public double[] bottomCurrentAmps = new double[] {};
  }

  /**
   * Updates the set of loggable inputs with the current state of the flywheel hardware.
   *
   * @param inputs An instance of FlywheelIOInputs to populate with sensor and status data.
   */
  public default void updateInputs(ElevatorFlywheelIOInputs inputs) {}

  /**
   * Runs the flywheel in open loop mode at the specified voltage. This method directly applies a
   * voltage to the motor without feedback control.
   *
   * @param volts The voltage command to apply to the flywheel motor.
   */
  public default void setBottomVoltage(double volts) {}

  public default void setTopVoltage(double volts) {}
  /**
   * Runs the flywheel in closed loop mode at the specified velocity. This method uses velocity
   * control (e.g., PID) and an optional feedforward voltage to achieve a target speed.
   *
   * @param velocityRadPerSec The target angular velocity in radians per second.
   * @param ffVolts The feedforward voltage to assist with reaching the desired velocity.
   */
  public default void setBottomVelocity(double velocityRadPerSec, double ffVolts) {}

  public default void setTopVelocity(double velocityRadPerSec, double ffVolts) {}
  /** Stops the flywheel by setting the output to zero, halting the motor output in open loop. */
  public default void topStop() {}

  public default void bottomStop() {}

  /**
   * Configures the PID constants for the velocity control loop.
   *
   * @param kP The proportional gain for the velocity PID controller.
   * @param kI The integral gain for the velocity PID controller.
   * @param kD The derivative gain for the velocity PID controller.
   */
  public default void configureBottomPID(double kP, double kI, double kD) {}

  public default void configureTopPID(double kP, double kI, double kD) {}
}
