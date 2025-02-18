package frc.robot.subsystems.superstructure.elevatorFlywheels;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * FlywheelIOSim is a simulated implementation of the FlywheelIO interface. It uses WPILib's
 * FlywheelSim class to model the behavior of a flywheel in a simulation environment, allowing for
 * testing and tuning of control logic without actual hardware.
 */
public class TopElevatorFlywheelsIOSim implements TopElevatorFlywheelsIO {
  // Creates a FlywheelSim instance, modeling a flywheel with NEO motor and given moment of inertia
  // and gearing
  private FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 1), DCMotor.getNEO(1));

  // PIDController for closed-loop velocity control of the simulated flywheel
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  // Flag indicating whether the flywheel is running in closed-loop mode
  private boolean closedLoop = true;

  // Variables for storing feedforward voltage and currently applied volts to the simulated motor
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  /**
   * Updates the FlywheelIOInputs with the latest simulated state of the flywheel. If closed-loop
   * mode is enabled, calculates the required volts using PID and feedforward.
   *
   * @param inputs The FlywheelIOInputs instance to populate with current simulation data.
   */
  @Override
  public void updateInputs(TopElevatorFlywheelsIOInputs inputs) {
    // If running in closed-loop mode, compute the output using PID and feedforward, then apply the
    // voltage
    if (closedLoop) {
      // Calculates the required voltage by combining PID output and feedforward
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec() + ffVolts), -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    // Advances the simulation by 20ms to simulate one control loop iteration
    sim.update(0.02);

    // Since this simulation might not track absolute position, we set position to 0.0
    inputs.positionRad = 0.0;

    // Reports the current angular velocity in radians per second from the simulation
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();

    // Reports the currently applied voltage to the simulated motor
    inputs.appliedVolts = appliedVolts;

    // Reports the current draw of the simulated motor in amps
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  /**
   * Runs the flywheel in open loop mode at the specified voltage. Disables closed-loop control and
   * directly applies a voltage command to the simulation.
   *
   * @param volts The voltage command to apply to the simulated flywheel.
   */
  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  /**
   * Runs the flywheel in closed-loop mode to reach a specified velocity. Sets the PID controller's
   * setpoint and includes a feedforward term to assist.
   *
   * @param velocityRadPerSec The target angular velocity in radians per second.
   * @param ffVolts The feedforward voltage to apply alongside PID output.
   */
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  /**
   * Stops the flywheel by setting the applied voltage to zero and disabling closed-loop control.
   */
  @Override
  public void stop() {
    setVoltage(0.0);
  }

  /**
   * Configures the PID constants used for closed-loop velocity control of the flywheel.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
