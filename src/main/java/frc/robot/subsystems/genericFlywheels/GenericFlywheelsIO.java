package frc.robot.subsystems.genericFlywheels;

import org.littletonrobotics.junction.AutoLog;

/**
 * FlywheelIO interface defines the hardware interactions for the flywheel subsystem. It allows for
 * updating sensor data, running the flywheel in open or closed loop, stopping the flywheel, and
 * configuring PID constants.
 */
public interface GenericFlywheelsIO {
  // AutoLog annotation indicates that the FlywheelIOInputs class fields will be automatically
  // logged
  @AutoLog
  public static class GenericFlywheelsIOInputs {

    // The current angular position of the flywheel in radians
    public double positionRad = 0.0;

    public double encoderPositionRad = 0.0;

    // The current angular velocity of the flywheel in radians per second
    public double velocityRadPerSec = 0.0;

    // The current applied voltage to the flywheel motor
    public double appliedVolts = 0.0;

    // An array representing the current draw of the flywheel motor(s) in amps
    public double[] currentAmps = new double[] {};
  }

  /**
   * Updates the set of loggable inputs with the current state of the flywheel hardware.
   *
   * @param inputs An instance of FlywheelIOInputs to populate with sensor and status data.
   */
  public default void updateInputs(GenericFlywheelsIOInputs inputs) {}

  /**
   * Runs the flywheel in open loop mode at the specified voltage. This method directly applies a
   * voltage to the motor without feedback control.
   *
   * @param volts The voltage command to apply to the flywheel motor.
   */
  public default void setVoltage(double volts) {}

  /**
   * Runs the flywheel in closed loop mode at the specified velocity. This method uses velocity
   * control (e.g., PID) and an optional feedforward voltage to achieve a target speed.
   *
   * @param velocityRadPerSec The target angular velocity in radians per second.
   * @param ffVolts The feedforward voltage to assist with reaching the desired velocity.
   */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stops the flywheel by setting the output to zero, halting the motor output in open loop. */
  public default void stop() {}

  /**
   * Configures the PID constants for the velocity control loop.
   *
   * @param kP The proportional gain for the velocity PID controller.
   * @param kI The integral gain for the velocity PID controller.
   * @param kD The derivative gain for the velocity PID controller.
   */
  public default void configurePID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
