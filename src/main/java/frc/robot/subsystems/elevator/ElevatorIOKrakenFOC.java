package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import java.util.List;

// Declares the ElevatorIOKrakenFOC class, which implements the ElevatorIO interface, handling the
// elevator using Kraken
// FOC
public class ElevatorIOKrakenFOC implements ElevatorIO {
  // Hardware instances for the leader and follower TalonFX motors
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;

  // Status signals for elevator position and velocity
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;

  // Status signals for applied voltages, supply currents, torque currents, and motor temperatures
  private final List<StatusSignal<Voltage>> appliedVoltage;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> tempCelsius;

  // Voltage control mode output initialized to 0 volts, enabling FOC, and zero update frequency
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);

  // Request object for Motion Magic control mode
  private MotionMagicVoltage request;

  // Configuration object for the TalonFX motors
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Constructor sets up the elevator motors and their configurations
  public ElevatorIOKrakenFOC() {

    // Initializes the leader TalonFX with the leader ID from ElevatorConstants on the "rio" bus
    leaderTalon = new TalonFX(ElevatorConstants.leaderID, "rio");
    // Initializes the follower TalonFX with the same ID (this will follow the leader)
    followerTalon = new TalonFX(ElevatorConstants.leaderID, "rio");
    // Sets the follower to follow the leader motor
    followerTalon.setControl(new Follower(ElevatorConstants.leaderID, true));

    // Leader motor configurations: PID and feedforward gains, current limits, inversion, and
    // neutral mode
    config.Slot0.kS =
        ElevatorConstants.gains.ffkS(); // Static feedforward gain to overcome friction
    config.Slot0.kV = ElevatorConstants.gains.ffkV(); // Velocity feedforward gain
    config.Slot0.kA = ElevatorConstants.gains.ffkA(); // Acceleration feedforward gain
    config.Slot0.kP = ElevatorConstants.gains.kP(); // Proportional PID gain
    config.Slot0.kI = ElevatorConstants.gains.kI(); // Integral PID gain
    config.Slot0.kD = ElevatorConstants.gains.kD(); // Derivative PID gain
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0; // Peak forward current limit
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0; // Peak reverse current limit

    // Configures motor output direction and neutral mode based on constants
    config.MotorOutput.Inverted =
        ElevatorConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Sets feedback sensor source and gear ratios for correct position/velocity readings
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.RotorToSensorRatio = ElevatorConstants.elevatorGearRatio;
    config.Feedback.SensorToMechanismRatio = 1.0;

    // Applies the configuration to the leader TalonFX with a 1-second timeout
    leaderTalon.getConfigurator().apply(config, 1.0);

    // Configures Motion Magic parameters: cruise velocity, acceleration, and jerk
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Cruise velocity in rotations per second
    motionMagicConfigs.MotionMagicAcceleration =
        160; // Acceleration in rotations per second squared
    motionMagicConfigs.MotionMagicJerk = 1600; // Jerk in rotations per second cubed

    // Retrieves position and velocity status signals from the leader TalonFX
    internalPositionRotations = leaderTalon.getPosition();
    velocityRps = leaderTalon.getVelocity();

    // Retrieves applied voltage, supply current, torque current, and temperature signals from the
    // leader TalonFX
    appliedVoltage = List.of(leaderTalon.getMotorVoltage());
    supplyCurrent = List.of(leaderTalon.getSupplyCurrent());
    torqueCurrent = List.of(leaderTalon.getTorqueCurrent());
    tempCelsius = List.of(leaderTalon.getDeviceTemp());

    // Sets the update frequency for all these signals to 100 Hz for timely telemetry
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

    // Optimizes CAN bus utilization for both leader and follower TalonFX controllers
    leaderTalon.optimizeBusUtilization(0, 1.0);
    followerTalon.optimizeBusUtilization(0, 1.0);
  }

  // Updates ElevatorIOInputs with the latest hardware data from the motors and sensors
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refreshes all relevant signals for the leader motor and checks their status
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                supplyCurrent.get(0),
                torqueCurrent.get(0),
                tempCelsius.get(0))
            .isOK();

    // Refreshes signals for the follower motor and checks their status
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVoltage.get(1),
                supplyCurrent.get(1),
                torqueCurrent.get(1),
                tempCelsius.get(1))
            .isOK();

    // Converts the internal position from rotations directly to meters (1:1 if configured properly)
    inputs.positionMeters = internalPositionRotations.getValueAsDouble();
    // Converts velocity from rotations per second to radians per second, then to meters/second if
    // setup accounts
    // for gearing
    inputs.velocityMetersPerSecond =
        Units.rotationsToRadians(velocityRps.getValue().in(RotationsPerSecond));

    // Extracts applied voltage values and stores them in the inputs array
    inputs.appliedVolts =
        appliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();

    // Extracts supply current values and stores them in the inputs array
    inputs.currentAmps =
        supplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();

    // Extracts temperature values and stores them in the inputs array
    inputs.tempCelcius = tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  // Sets the elevator height using a Motion Magic profile with a given feedforward voltage
  @Override
  public void runPosition(double setpointRads, double feedforward) {
    // Creates a new MotionMagicVoltage request starting at 0 volts
    request = new MotionMagicVoltage(0);
    // Converts the setpoint from radians to rotations and applies feedforward voltage
    leaderTalon.setControl(
        request
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward)
            .withEnableFOC(true));
  }

  // Runs the elevator motors at a specified open-loop voltage
  @Override
  public void runVolts(double volts) {
    // Sets the elevator to run directly at the given voltage in open-loop mode
    leaderTalon.setControl(voltageControl.withOutput(volts));
  }

  // Stops all motor output, halting the elevator's motion
  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
  }

  // Enables or disables brake mode on the elevator motor
  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
