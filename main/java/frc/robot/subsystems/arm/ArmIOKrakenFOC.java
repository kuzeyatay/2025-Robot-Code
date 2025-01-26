package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;

// Define the ArmIOKrakenFOC class, which implements the ArmIO interface
public class ArmIOKrakenFOC implements ArmIO {
  // Hardware Components

  // Define the leader TalonFX motor controller with the leaderID from ArmConstants and connected to
  // the "rio" CAN bus
  private final TalonFX leaderTalon;
  // Define the absolute CANcoder encoder with the armEncoderID from ArmConstants and connected to
  // the "rio" CAN bus
  private final CANcoder absoluteEncoder;

  // Status Signals

  // Status signal for the internal position of the arm in rotations, retrieved from the leader
  // TalonFX
  private final StatusSignal<Angle> internalPositionRotations;
  // Status signal for the absolute encoder position in rotations from the CANcoder
  private final StatusSignal<Angle> encoderAbsolutePositionRotations;
  // Status signal for the relative encoder position in rotations from the CANcoder
  private final StatusSignal<Angle> encoderRelativePositionRotations;
  // Status signal for the arm's velocity in rotations per second from the leader TalonFX
  private final StatusSignal<AngularVelocity> velocityRps;
  // List of status signals for the voltages applied to the motors
  private final List<StatusSignal<Voltage>> appliedVoltage;
  // List of status signals for the supply currents in amps for the motors
  private final List<StatusSignal<Current>> supplyCurrent;
  // List of status signals for the torque currents in amps for the motors
  private final List<StatusSignal<Current>> torqueCurrent;
  // List of status signals for the motor temperatures in Celsius
  private final List<StatusSignal<Temperature>> tempCelsius;

  // Control Modes

  // Define a voltage control mode with initial voltage of 0.0 volts, enabling FOC (Field-Oriented
  // Control), and
  // update frequency of 0.0 Hz
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  // Define a torque current control mode with initial torque of 0.0 amps and update frequency of
  // 0.0 Hz
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  // Configuration

  // Create a new TalonFX configuration object to hold various settings
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Constructor for the ArmIOKrakenFOC class
  public ArmIOKrakenFOC() {
    // Initialize the leader TalonFX motor controller with the specified CAN ID and CAN bus
    leaderTalon = new TalonFX(ArmConstants.leaderID, "rio");

    // Initialize the absolute CANcoder encoder with the specified CAN ID and CAN bus
    absoluteEncoder = new CANcoder(ArmConstants.armEncoderID, "rio");

    // Configure the absolute encoder settings
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    // Set the sensor direction to clockwise positive
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // Set the magnet offset by converting the arm's zero cosine offset from radians to rotations
    armEncoderConfig.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(ArmConstants.kArmZeroCosineOffset);
    // Apply the configuration to the absolute encoder with a timeout of 1.0 seconds
    absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1.0);

    // Configure the leader TalonFX motor controller settings

    // Set the static friction voltage (kS) to 0.25 volts to overcome static friction
    config.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    // Set the velocity gain (kV) to 0.12 volts per rotation per second
    config.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    // Set the acceleration gain (kA) to 0.01 volts per rotation per second squared
    config.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // Set the proportional gain (kP) from ArmConstants
    config.Slot0.kP = ArmConstants.gains.kP();
    // Set the integral gain (kI) from ArmConstants
    config.Slot0.kI = ArmConstants.gains.kI();
    // Set the derivative gain (kD) from ArmConstants
    config.Slot0.kD = ArmConstants.gains.kD();
    // Set the peak forward torque current to 80.0 amps
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    // Set the peak reverse torque current to -80.0 amps
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    // Configure motor output settings based on inversion from ArmConstants
    config.MotorOutput.Inverted =
        ArmConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Set the neutral mode to brake
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Set the remote sensor ID to the arm encoder's CAN ID
    config.Feedback.FeedbackRemoteSensorID = ArmConstants.armEncoderID;
    // Set the feedback sensor source to the fused CANcoder
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // Set the rotor to sensor gear ratio from ArmConstants
    config.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;
    // Set the sensor to mechanism gear ratio to 1.0 (no additional gearing)
    config.Feedback.SensorToMechanismRatio = 1.0;
    // Apply the TalonFX configuration to the leader Talon with a timeout of 1.0 seconds
    leaderTalon.getConfigurator().apply(config, 1.0);

    // Set Motion Magic control parameters

    // Retrieve the MotionMagic configuration from the TalonFX configuration
    var motionMagicConfigs = config.MotionMagic;
    // Set the cruise velocity for Motion Magic to 80 rotations per second
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    // Set the acceleration for Motion Magic to 160 rotations per second squared (0.5 seconds to
    // reach target)
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Target acceleration of 160 rps/s (0.5 seconds)
    // Set the jerk for Motion Magic to 1600 rotations per second cubed (0.1 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // Initialize Status Signals

    // Get the internal position from the leader TalonFX
    internalPositionRotations = leaderTalon.getPosition();
    // Get the absolute encoder position from the CANcoder
    encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    // Get the relative encoder position from the CANcoder
    encoderRelativePositionRotations = absoluteEncoder.getPosition();
    // Get the velocity in rotations per second from the leader TalonFX
    velocityRps = leaderTalon.getVelocity();
    // Get the applied voltages to the motors from the leader TalonFX
    appliedVoltage = List.of(leaderTalon.getMotorVoltage());
    // Get the supply currents in amps from the leader TalonFX
    supplyCurrent = List.of(leaderTalon.getSupplyCurrent());
    // Get the torque currents in amps from the leader TalonFX
    torqueCurrent = List.of(leaderTalon.getTorqueCurrent());
    // Get the temperatures in Celsius from the leader TalonFX
    tempCelsius = List.of(leaderTalon.getDeviceTemp());

    // Set the update frequency for various status signals to optimize data transmission

    // Set the update frequency for internal position, velocity, applied voltage, supply current,
    // torque current,
    // and temperature to 100 Hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage.get(0),
        appliedVoltage.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        tempCelsius.get(0),
        tempCelsius.get(1));

    // Set the update frequency for absolute and relative encoder positions to 500 Hz
    BaseStatusSignal.setUpdateFrequencyForAll(
        500, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

    // Optimize CAN bus utilization by scheduling message updates efficiently
    leaderTalon.optimizeBusUtilization(0, 1.0);
    absoluteEncoder.optimizeBusUtilization(0, 1.0);
  }

  // Method to update the inputs for the ArmIO interface
  public void updateInputs(ArmIOInputs inputs) {
    // Refresh and check if the leader motor's signals are OK (connected and responsive)
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();
    // Refresh and check if the follower motor's signals are OK (connected and responsive)
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
            .isOK();
    // Refresh and check if the absolute encoder's signals are OK (connected and responsive)
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(
                encoderAbsolutePositionRotations, encoderRelativePositionRotations)
            .isOK();

    // Update the arm's position in radians by converting internal rotations to radians
    inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
    // Update the absolute encoder's position in radians, adjusting for the zero cosine offset
    inputs.absoluteEncoderPositionRads =
        Units.rotationsToRadians(encoderAbsolutePositionRotations.getValueAsDouble())
            - Units.degreesToRadians(ArmConstants.kArmZeroCosineOffset); // Negate internal offset
    // Update the relative encoder's position in radians, adjusting for the zero cosine offset
    inputs.relativeEncoderPositionRads =
        Units.rotationsToRadians(encoderRelativePositionRotations.getValueAsDouble())
            - Units.degreesToRadians(ArmConstants.kArmZeroCosineOffset);
    // Update the arm's velocity in radians per second by converting rotations per second to radians
    // per second
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocityRps.getValue().in(RotationsPerSecond));
    // Update the applied voltages by converting status signals to a double array
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    // Update the supply currents by converting status signals to a double array
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    // Update the torque currents by converting status signals to a double array
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    // Update the temperatures in Celsius by converting status signals to a double array
    inputs.tempCelcius = tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  // Override the runSetpoint method from the ArmIO interface to control the arm's position using
  // Motion Magic with
  // feedforward voltage
  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    // Create a new MotionMagicVoltage control request with initial output of 0 volts
    final MotionMagicVoltage request = new MotionMagicVoltage(0);
    // Set the control parameters: position (converted from radians to rotations), feedforward
    // voltage, and enable
    // FOC
    leaderTalon.setControl(
        request
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward)
            .withEnableFOC(true));
  }

  // Override the runVolts method from the ArmIO interface to apply specific voltages to the arm
  // motors
  @Override
  public void runVolts(double volts) {
    // Apply the specified voltages using the predefined voltage control mode
    leaderTalon.setControl(voltageControl.withOutput(volts));
  }

  // Override the runCurrent method from the ArmIO interface to apply specific currents to the arm
  // motors
  @Override
  public void runCurrent(double amps) {
    // Apply the specified currents using the predefined torque current control mode
    leaderTalon.setControl(currentControl.withOutput(amps));
  }

  // Override the setBrakeMode method from the ArmIO interface to enable or disable brake mode on
  // the arm motors
  @Override
  public void setBrakeMode(boolean enabled) {
    // Set the neutral mode to Brake if enabled is true, otherwise set to Coast
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
    leaderTalon.getConfigurator().apply(config, 0.01);
  }

  // Override the stop method from the ArmIO interface to immediately stop all arm motors
  @Override
  public void stop() {
    // Send a NeutralOut control command to stop the leader TalonFX motor
    leaderTalon.setControl(new NeutralOut());
  }
}
