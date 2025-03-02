package frc.robot.subsystems.superstructure.coralWrist;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

// Define the ArmIOKrakenFOC class, which implements the ArmIO interface
public class CoralWristIOKrakenFOC implements CoralWristIO {
  // Hardware Components

  // Define the leader TalonFX motor controller with the leaderID from ArmConstants and connected to
  // the "rio" CAN bus
  private final TalonFX talon;

  // Status Signals

  // Status signal for the internal position of the arm in rotations, retrieved from the leader
  // TalonFX
  private final StatusSignal<Angle> internalPositionRotations;
  // Status signal for the arm's velocity in rotations per second from the leader TalonFX
  private final StatusSignal<AngularVelocity> velocityRps;
  // List of status signals for the voltages applied to the motors
  private final StatusSignal<Voltage> appliedVoltage;
  // List of status signals for the supply currents in amps for the motors
  private final StatusSignal<Current> supplyCurrent;
  // List of status signals for the torque currents in amps for the motors
  private final StatusSignal<Current> torqueCurrent;
  // List of status signals for the motor temperatures in Celsius
  private final StatusSignal<Temperature> tempCelsius;

  // Define a torque current control mode with initial torque of 0.0 amps and update frequency of
  // 0.0 Hz

  // Define a voltage control mode with initial voltage of 0.0 volts, enabling FOC (Field-Oriented
  // Control), and
  // update frequency of 0.0 Hz
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // Control Requests
  private final TorqueCurrentFOC torqueCurrentRequest =
      new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  // Configuration

  // Create a new TalonFX configuration object to hold various settings
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Constructor for the ArmIOKrakenFOC class
  public CoralWristIOKrakenFOC() {
    // Initialize the leader TalonFX motor controller with the specified CAN ID and CAN bus
    talon = new TalonFX(CoralWristConstants.leaderID, "rio");

    // Configure the leader TalonFX motor controller settings

    // Set the peak forward torque current to 80.0 amps
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    // Set the peak reverse torque current to -80.0 amps
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    // Configure motor output settings based on inversion from ArmConstants
    config.MotorOutput.Inverted =
        CoralWristConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Set the neutral mode to brake
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set the sensor to mechanism gear ratio to 1.0 (no additional gearing)
    config.Feedback.SensorToMechanismRatio = CoralWristConstants.kArmGearRatio;
    // Apply the TalonFX configuration to the leader Talon with a timeout of 1.0 seconds
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Initialize Status Signals

    talon.setPosition(0);

    // Get the internal position from the leader TalonFX
    internalPositionRotations = talon.getPosition();
    // Get the absolute encoder position from the CANcoder

    // Get the velocity in rotations per second from the leader TalonFX
    velocityRps = talon.getVelocity();
    // Get the applied voltages to the motors from the leader TalonFX
    appliedVoltage = talon.getMotorVoltage();
    // Get the supply currents in amps from the leader TalonFX
    supplyCurrent = talon.getSupplyCurrent();
    // Get the torque currents in amps from the leader TalonFX
    torqueCurrent = talon.getTorqueCurrent();
    // Get the temperatures in Celsius from the leader TalonFX
    tempCelsius = talon.getDeviceTemp();

    // Set the update frequency for various status signals to optimize data transmission

    // Set the update frequency for internal position, velocity, applied voltage, supply current,
    // torque current,
    // and temperature to 100 Hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    // Set the update frequency for absolute and relative encoder positions to 500 Hz
    BaseStatusSignal.setUpdateFrequencyForAll(500, internalPositionRotations);

    // Optimize CAN bus utilization by scheduling message updates efficiently
    talon.optimizeBusUtilization(0, 1.0);

    // Set Motion Magic control parameters
    // set slot 0 gains
    var slot0Configs = config.Slot0;
    slot0Configs.kS =
        CoralWristConstants.gains.ffkS(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV =
        CoralWristConstants.gains.ffkV(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = CoralWristConstants.gains.ffkA();
    ; // An acceleration of 1 rps/s requires 0.01 V output

    // Retrieve the MotionMagic configuration from the TalonFX configuration
    var motionMagicConfigs = config.MotionMagic;
    // Set the cruise velocity for Motion Magic to 80 rotations per second
    motionMagicConfigs.MotionMagicCruiseVelocity = 320; // Target cruise velocity of 80 rps
    // Set the acceleration for Motion Magic to 160 rotations per second squared (0.5 seconds to
    // reach target)
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.42; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)
  }

  // Method to update the inputs for the ArmIO interface
  public void updateInputs(CoralWristIOInputs inputs) {
    // Refresh and check if the leader motor's signals are OK (connected and responsive)
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();

    // Update the arm's position in radians by converting internal rotations to radians
    inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());

    // Update the arm's velocity in radians per second by converting rotations per second to radians
    // per second
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocityRps.getValue().in(RotationsPerSecond));
    // Update the applied voltages by converting status signals to a double array
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    // Update the supply currents by converting status signals to a double array
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    // Update the torque currents by converting status signals to a double array
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    // Update the temperatures in Celsius by converting status signals to a double array
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }

  // Override the runSetpoint method from the ArmIO interface to control the arm's position using
  // Motion Magic with
  // feedforward voltage
  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    // Create a new MotionMagicVoltage control request with initial output of 0 volts
    final MotionMagicExpoVoltage request = new MotionMagicExpoVoltage(0);
    // Set the control parameters: position (converted from radians to rotations), feedforward
    // voltage, and enable
    // FOC

    talon.setControl(
        request
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(0.015)
            .withSlot(0)
            .withEnableFOC(true));
  }

  // Override the runVolts method from the ArmIO interface to apply specific voltages to the arm
  // motors
  @Override
  public void runVolts(double volts) {
    // Apply the specified voltages using the predefined voltage control mode
    talon.setControl(voltageControl.withOutput(volts));
  }

  // Override the runCurrent method from the ArmIO interface to apply specific currents to the arm
  // motors
  @Override
  public void runCurrent(double amps) {
    // Apply the specified currents using the predefined torque current control mode
    talon.setControl(currentControl.withOutput(amps));
  }

  // Override the setBrakeMode method from the ArmIO interface to enable or disable brake mode on
  // the arm motors
  @Override
  public void setBrakeMode(boolean enabled) {
    // Set the neutral mode to Brake if enabled is true, otherwise set to Coast
    talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  // Override the setPID method from the ArmIO interface to update the PID controller gains
  @Override
  public void setPID(double p, double i, double d) {
    // Update the proportional gain in the configuration
    config.Slot0.kP = p;
    // Update the integral gain in the configuration
    config.Slot0.kI = i;
    // Update the derivative gain in the configuration
    config.Slot0.kD = d;
    // Apply the updated configuration to the leader TalonFX with a timeout of 0.01 seconds
    talon.getConfigurator().apply(config, 0.01);
  }

  // Override the stop method from the ArmIO interface to immediately stop all arm motors
  @Override
  public void stop() {
    // Send a NeutralOut control command to stop the leader TalonFX motor
    talon.stopMotor();
  }

  @Override
  public void zero() {

    talon.setPosition(0);
  }
}
