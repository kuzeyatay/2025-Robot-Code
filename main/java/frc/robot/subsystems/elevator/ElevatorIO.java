package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

// Defines the ElevatorIO interface which acts as the hardware interface for the Elevator subsystem
/* Elevator subsystem hardware interface */
public interface ElevatorIO {

  // AutoLog annotation indicates that the ElevatorIOInputs class will have its fields automatically
  // logged
  @AutoLog
  // Defines a static nested class to hold the inputs from the Elevator hardware
  public static class ElevatorIOInputs {

    // Indicates whether the leader motor is connected
    public boolean leaderMotorConnected = false;

    // Indicates whether the follower motor is connected
    public boolean followerMotorConnected = false;

    // Current position of the elevator in meters
    public double positionMeters = 0.0;

    // Current velocity of the elevator in meters per second
    public double velocityMetersPerSecond = 0.0;

    // Array of applied voltages to the elevator motors
    public double[] appliedVolts = new double[] {};

    // Array of current (amps) drawn by the elevator motors
    public double[] currentAmps = new double[] {};

    // Array of temperatures in Celsius for the elevator motors
    public double[] tempCelcius = new double[] {};
  }

  // Comment describing the method updateInputs
  /* Updates the set of loggable inputs */
  // Default method to update the ElevatorIOInputs with current hardware states
  public default void updateInputs(ElevatorIOInputs inputs) {}

  // Comment describing the method setHeight
  /* Sets the climber to a height setpoint via motion magic */
  // Default method to set the elevator to a specific height with feedforward
  public default void setHeight(double height, double feedforward) {}

  // Javadoc comment describing the stop method
  /** Stop the control loop and motor output. */
  // Default method to stop all elevator motors and control loops
  public default void stop() {}

  // Default method to run the elevator motors with a specified voltage
  public default void runVolts(double volts) {}

  // Javadoc comment describing the setBrakeMode method
  /** Set brake mode enabled */
  // Default method to enable or disable brake mode on the elevator motors
  default void setBrakeMode(boolean enabled) {}

  // Javadoc comment describing the setPID method
  /** Set PID values */
  // Default method to set PID controller gains for the elevator
  default void setPID(double p, double i, double d) {}
}
