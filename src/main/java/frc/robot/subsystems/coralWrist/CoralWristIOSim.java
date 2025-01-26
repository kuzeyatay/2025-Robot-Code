package frc.robot.subsystems.coralWrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Define the ArmIOSim class, which implements the ArmIO interface for simulation purposes
public class CoralWristIOSim implements CoralWristIO {
  // Define a constant for the automatic start angle, converted from degrees to radians
  private static final double autoStartAngle = Units.degreesToRadians(80.0);

  // Initialize the SingleJointedArmSim with specific parameters:
  // - DCMotor: A single Kraken X60 FOC motor
  // - Gear ratio from ArmConstants
  // - MOI (Moment of Inertia): 1.06328 kg·m²
  // - Arm length from ArmConstants
  // - Minimum and maximum angles from ArmConstants
  // - Use gravity (false means gravity is not considered)
  // - Initial angle offset converted from degrees to radians
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          CoralWristConstants.kArmGearRatio,
          1.06328,
          CoralWristConstants.armLength,
          CoralWristConstants.minAngle,
          CoralWristConstants.maxAngle,
          false,
          Units.degreesToRadians(0.0));

  // Initialize a PIDController with initial gains set to 0.0
  private final PIDController controller;
  // Variable to store the applied voltage to the simulated arm
  private double appliedVoltage = 0.0;
  // Offset to adjust the arm's position, initialized to 30.0 radians
  private double positionOffset = 30.0;

  // Flag indicating whether the PID controller needs to be reset
  private boolean controllerNeedsReset = false;
  // Flag indicating whether the system is in closed-loop control mode
  private boolean closedLoop = true;
  // Flag to track if the robot was not previously in autonomous mode
  private boolean wasNotAuto = true;

  // Constructor for the ArmIOSim class
  public CoralWristIOSim() {
    // Instantiate the PIDController with initial P, I, D gains set to 0.0
    controller = new PIDController(0.0, 0.0, 0.0);
    // Set the initial state of the simulated arm to position 0.0 radians and velocity 0.0 rad/s
    sim.setState(0.0, 0.0);
    // Adjust the position offset based on the initial position
    setPosition(0.0);
  }

  // Override the updateInputs method from the ArmIO interface to update sensor inputs
  @Override
  public void updateInputs(CoralWristIOInputs inputs) {
    // If the robot is disabled, mark that the controller needs to be reset
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // If the robot was not in autonomous mode and now is, reset the simulation state
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    // Update the wasNotAuto flag based on the current autonomous state
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Update the simulation by advancing it by 0.02 seconds (20 ms loop time)
    sim.update(0.02);

    // Update the position in radians, adding the position offset
    inputs.positionRads = sim.getAngleRads() + positionOffset;
    // Update the velocity in radians per second
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    // Update the applied voltages array with the current applied voltage
    inputs.appliedVolts = new double[] {appliedVoltage};
    // Update the supply currents array with the simulated current draw in amps
    inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    // Update the torque currents array with the simulated torque current draw in amps
    inputs.torqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    // Update the temperatures array with a fixed value of 0.0 Celsius (no simulation of
    // temperature)
    inputs.tempCelcius = new double[] {0.0};

    // Reset the input voltage to 0.0 volts after applying it to the simulation
    sim.setInputVoltage(0.0);
  }

  // Override the runSetpoint method from the ArmIO interface to move the arm to a target setpoint
  // using PID control
  // and feedforward
  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    // If the system is not in closed-loop mode, mark that the controller needs to be reset and
    // enable closed-loop
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    // If the controller needs to be reset, reset it and clear the flag
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    // Calculate the voltage output using the PID controller based on the current position and
    // target setpoint with
    // offset
    runVolts(controller.calculate(sim.getAngleRads(), setpointRads + positionOffset) + feedforward);
  }

  // Override the runVolts method from the ArmIO interface to apply a specific voltage to the arm
  // motors
  @Override
  public void runVolts(double volts) {
    // Mark the system as not being in closed-loop control mode
    closedLoop = false;
    // Clamp the applied voltage between -12.0 and 12.0 volts for safety
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    // Apply the clamped voltage to the simulation
    sim.setInputVoltage(appliedVoltage);
  }

  // Override the setPID method from the ArmIO interface to update the PID controller gains
  @Override
  public void setPID(double p, double i, double d) {
    // Set the proportional gain of the PID controller
    controller.setPID(p, i, d);
  }

  // Override the stop method from the ArmIO interface to immediately stop the arm motors
  @Override
  public void stop() {
    // Set the applied voltage to 0.0 volts to stop the simulation
    appliedVoltage = 0.0;
    // Apply 0.0 volts to the simulation to halt the arm movement
    sim.setInputVoltage(appliedVoltage);
  }

  // Private method to set the position offset based on a given position
  private void setPosition(double position) {
    // Calculate the new position offset by subtracting the current simulated angle from the desired
    // position
    positionOffset = position - sim.getAngleRads();
  }
}
