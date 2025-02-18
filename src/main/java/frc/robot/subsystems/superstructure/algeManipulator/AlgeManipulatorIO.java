package frc.robot.subsystems.superstructure.algeManipulator;

import org.littletonrobotics.junction.AutoLog;

// Define the ArmIO interface for input/output operations related to the arm subsystem
public interface AlgeManipulatorIO {

  // Define a nested class to hold inputs for the ArmIO, with auto-logging capability
  @AutoLog
  class AlgeManipulatorIOInputs {
    // Indicates if the leader motor is connected
    public boolean leaderMotorConnected = true;

    // Current position of the arm in radians from the primary encoder
    public double positionRads = 0.0;
    // Relative encoder position in radians
    public double relativeEncoderPositionRads = 0.0;
    // Current velocity of the arm in radians per second
    public double velocityRadsPerSec = 0.0;
    // Array of voltages applied to the motors
    public double appliedVolts = 0.0;
    // Array of supply currents in amps for the motors
    public double supplyCurrentAmps = 0.0;
    // Array of torque currents in amps for the motors
    public double torqueCurrentAmps = 0.0;
    // Array of temperatures in Celsius for the motors
    public double tempCelcius = 0.0;
  }

  // Default method to update the ArmIOInputs; to be implemented by concrete classes
  default void updateInputs(AlgeManipulatorIOInputs inputs) {}

  /**
   * Default method to run the arm to a specified setpoint angle in radians with a feedforward
   * voltage.
   *
   * @param setpointRads The target angle in radians.
   * @param feedforward The feedforward voltage to apply.
   */
  default void runSetpoint(double setpointRads, double feedforward) {}

  /**
   * Default method to run the arm motors at specified voltages.
   *
   * @param volts An array of voltages to apply to the motors.
   */
  default void runVolts(double volts) {}

  /**
   * Default method to run the arm motors with specified current in amps.
   *
   * @param amps An array of currents in amps to apply to the motors.
   */
  default void runCurrent(double amps) {}

  /**
   * Default method to enable or disable brake mode on the arm motors.
   *
   * @param enabled If true, brake mode is enabled; otherwise, it is disabled.
   */
  default void setBrakeMode(boolean enabled) {}

  /**
   * Default method to set the PID controller values for the arm motors.
   *
   * @param p The proportional gain.
   * @param i The integral gain.
   * @param d The derivative gain.
   */
  default void setPID(double p, double i, double d) {}

  default void runOpenLoop(double output) {}

  default void runTorqueCurrent(double current) {}

  /** Default method to stop all arm motors immediately. */
  default void stop() {}

  default void zero() {}
}
