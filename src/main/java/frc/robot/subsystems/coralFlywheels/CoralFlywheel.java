package frc.robot.subsystems.coralFlywheels;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSetter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The Flywheel subsystem controls a flywheel mechanism which may be used for shooting or launching
 * game pieces. It supports both open-loop voltage control and closed-loop velocity control, and can
 * be tuned differently based on the robot's mode.
 */
public class CoralFlywheel extends SubsystemBase {
  // IO interface for controlling and reading from the flywheel hardware
  private final CoralFlywheelIO io;
  // Structure holding inputs from the FlywheelIO, automatically logged for telemetry
  private final CoralFlywheelIOInputsAutoLogged inputs = new CoralFlywheelIOInputsAutoLogged();

  // A feedforward model for the flywheel's motor(s), used in closed-loop control
  private final SimpleMotorFeedforward ffModel;

  /**
   * Constructs a new Flywheel subsystem.
   *
   * @param io The FlywheelIO implementation to interact with the flywheel hardware.
   */
  public CoralFlywheel(CoralFlywheelIO io) {
    this.io = io;

    // Configure PID and feedforward gains based on the robot's current mode
    // The simulation mode may require different tuning than the real hardware
    switch (ModeSetter.currentMode) {
      case REAL:
      case REPLAY:
        // On real hardware or replay mode, use these gains
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        // In simulation, use different feedforward and PID gains to achieve stable behavior in the
        // model
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        // Default or unrecognized mode uses zero gains
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  /**
   * This method is called periodically by the scheduler. Updates the flywheel inputs and logs them.
   */
  @Override
  public void periodic() {
    // Update inputs from the hardware implementation
    io.updateInputs(inputs);
    // Process and log these inputs for debugging and telemetry
    Logger.processInputs("Flywheel", inputs);
  }

  /**
   * Runs the flywheel open-loop at the specified voltage.
   *
   * @param volts The voltage command to apply to the flywheel motor(s).
   */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Runs the flywheel closed-loop to achieve a specified velocity in RPM.
   *
   * @param velocityRPM The target flywheel velocity in rotations per minute (RPM).
   */
  public void runVelocity(double velocityRPM) {
    // Convert velocity from RPM to radians per second for internal calculations
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    // Calculate the feedforward voltage using the feedforward model and the current velocity
    // This helps the motor controller achieve the desired speed more effectively
    io.setVelocity(
        velocityRadPerSec, ffModel.calculateWithVelocities(velocityRadPerSec, velocityRadPerSec));

    // Log the commanded setpoint RPM for telemetry
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel by halting motor outputs. */
  public void stop() {
    io.stop();
  }

  /**
   * Returns the current velocity of the flywheel in RPM.
   *
   * @return The current flywheel speed in rotations per minute.
   */
  @AutoLogOutput
  public double getVelocityRPM() {
    // Converts radians per second to RPM for more human-readable units
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /**
   * Returns the current velocity of the flywheel in radians per second. Useful for characterization
   * and system identification purposes.
   *
   * @return The current flywheel speed in radians per second.
   */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
