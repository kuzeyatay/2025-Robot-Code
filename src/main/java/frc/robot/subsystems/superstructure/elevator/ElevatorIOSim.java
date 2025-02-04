package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * ElevatorIOSim is a simulated implementation of the ElevatorIO interface. It uses WPILib's
 * ElevatorSim class to model the behavior of the elevator without physical hardware, allowing for
 * testing and tuning in a simulated environment.
 */
public class ElevatorIOSim implements ElevatorIO {
  // Creates a simulation object representing the elevator mechanism.
  // Takes into account gearbox, gearing, carriage mass, drum radius, and min/max heights.
  private final ElevatorSim sim =
      new ElevatorSim(
          ElevatorConstants.elevatorGearbox,
          ElevatorConstants.elevatorGearRatio,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeightMeters,
          ElevatorConstants.kMaxElevatorHeightMeters,
          true,
          0.0);

  // PIDController for running closed-loop position control on the simulated elevator
  private final PIDController controller;

  // Variable to store the applied voltage output to the simulated elevator motor
  private double appliedVoltage = 0.0;

  // Flags to handle state transitions and control modes
  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  // Constructor initializes the PIDController with gains from ElevatorConstants and sets initial
  // simulation state
  public ElevatorIOSim() {
    controller =
        new PIDController(
            ElevatorConstants.gains.kP(),
            ElevatorConstants.gains.kI(),
            ElevatorConstants.gains.kD());
    // Sets the initial state of the simulated elevator at position 0.0 meters and 0.0 m/s velocity
    sim.setState(0.0, 0.0);
  }

  /**
   * Periodically updates the ElevatorIOInputs with the simulated elevator's state.
   *
   * @param inputs The ElevatorIOInputs object to populate with current simulation data.
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // If the robot is disabled, reset the PID controller when it re-enables
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset the elevator at the start of autonomous if it was not previously in auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(0, 0.0);
      wasNotAuto = false;
    }
    // Track if the robot was not in autonomous mode
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Advance the simulation by 20ms, simulating one control loop iteration
    sim.update(0.02);

    // Populate the ElevatorIOInputs with the simulated elevator states
    inputs.positionMeters = sim.getPositionMeters(); // Current elevator position in meters
    inputs.velocityMetersPerSecond =
        sim.getVelocityMetersPerSecond(); // Current elevator speed in m/s
    inputs.appliedVolts =
        new double[] {appliedVoltage}; // The currently applied voltage to the simulated motor
    inputs.currentAmps =
        new double[] {sim.getCurrentDrawAmps()}; // Current draw of the simulated motor
    inputs.tempCelcius =
        new double[] {0.0}; // Temperature data is not simulated here, default to 0.0

    // Reset the input voltage after processing to simulate a closed-loop update cycle
    sim.setInputVoltage(0.0);
  }

  /**
   * Commands the simulated elevator to move to a specified height using closed-loop position
   * control.
   *
   * @param height The target height in meters.
   * @param feedforward The feedforward voltage to apply in addition to the PID output.
   */
  @Override
  public void runPosition(double height, double feedforward) {
    // If we previously were not in closed-loop mode, reset the controller before re-enabling
    // closed-loop control
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }

    // If the controller needs a reset (e.g., after being disabled), reset its internal state
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }

    // Set the PID controller's setpoint to the target height
    controller.setSetpoint(height);

    // Calculate PID output based on current position
    double pidOutput = controller.calculate(sim.getPositionMeters());

    // Apply the combined PID and feedforward voltages to the simulated motor
    runVolts(pidOutput + feedforward);
  }

  /**
   * Runs the elevator at a specified open-loop voltage, disabling closed-loop control.
   *
   * @param volts The voltage command to apply to the simulated motor.
   */
  @Override
  public void runVolts(double volts) {
    // Switch to open-loop mode since we are directly applying voltage
    closedLoop = false;

    // Clamp the voltage between -12.0 and 12.0 volts to match real hardware constraints
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

    // Apply the clamped voltage to the simulated elevator input
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void runOpenLoop(double output) {
    runVolts(output);
  }
  /** Stops the elevator by setting the applied voltage to zero, halting motion. */
  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }
}
