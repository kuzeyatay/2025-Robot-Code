package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefLevel;
import frc.robot.ModeSetter;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIO;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIOInputsAutoLogged;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem that controls the robot's elevator mechanism. Utilizes PID control and
 * trapezoidal motion profiling for precise positioning.
 *
 * @param <GenericFlywheelsIO>
 */
public class Elevator extends SubsystemBase {
  private final SysIdRoutine sysId;
  // Tunable PID and Feedforward gains for the elevator, allowing real-time adjustments
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/Gains/kG", gains.ffkG());

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Elevator/HomingTimeSecs", 0.25);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber(
          "Elevator/HomingVelocityThresh", 5.0 * ElevatorConstants.kElevatorDrumRadius);

  private static final LoggedTunableNumber maxVelocityMetersPerSec =
      new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 5);
  private static final LoggedTunableNumber maxAccelerationMetersPerSec2 =
      new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 4);

  // A debouncer to delay downward movements
  private final Debouncer downDebouncer = new Debouncer(0.5);

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

  @AutoLogOutput @Getter private boolean homed = true;

  @AutoLogOutput(key = "Elevator/HomedPosition")
  private double homedPosition = 0.0;

  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.04);

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);

  // Suppliers to handle various override conditions
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = DriverStation::isDisabled;
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get()));

  @AutoLogOutput(key = "Elevator/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;

  @AutoLogOutput(key = "Elevator/isDebouncerNeeded")
  public boolean isDebouncerNeeded = false;

  private double previousGoalPosition = 0.0;
  public double currentGoalPosition = 0.0;

  // IO interface for controlling and reading from the flywheel hardware
  private final GenericFlywheelsIO bottomFlywheelsIO;
  private final GenericFlywheelsIO topFlywheelsIO;
  // Structure holding inputs from the FlywheelIO, automatically logged for telemetry
  private final GenericFlywheelsIOInputsAutoLogged bottomFlywheelsInputs =
      new GenericFlywheelsIOInputsAutoLogged();
  private final GenericFlywheelsIOInputsAutoLogged topFlywheelsInputs =
      new GenericFlywheelsIOInputsAutoLogged();

  // A feedforward model for the flywheel's motor(s), used in closed-loop control
  private final SimpleMotorFeedforward bottomFlywheelsFFModel;
  private final SimpleMotorFeedforward topFlywheelsFFModel;

  /**
   * Enum representing predefined goal positions for the elevator. Each goal has an associated
   * height supplier.
   */
  @RequiredArgsConstructor
  // FIXME
  public enum Goal {
    STOW(new LoggedTunableNumber("Elevator/STOW", 0.0)),
    CHUTE(new LoggedTunableNumber("Elevator/CHUTE", 0.12)),
    L1(new LoggedTunableNumber("Elevator/L1", 0.0)),
    L2(new LoggedTunableNumber("Elevator/L2", 0)),
    LIMBO_1(
        new LoggedTunableNumber(
            "Elevator/L2_L3",
            ReefLevel.L2.height + 0.3 + ((ReefLevel.L3.height - ReefLevel.L2.height) / 2))),
    L3(new LoggedTunableNumber("Elevator/L3", ReefLevel.L3.height)),
    L4(new LoggedTunableNumber("Elevator/L4", 1.2)),
    LIMBO_2(
        new LoggedTunableNumber(
            "Elevator/L3_L4", ReefLevel.L3.height + 0.17 + ((1.2 - ReefLevel.L3.height) / 2))),
    PROCESSOR(new LoggedTunableNumber("Elevator/PROCESSOR", 0.0)),
    NET(new LoggedTunableNumber("Elevator/NET", 1.29)),
    EJECT(new LoggedTunableNumber("Elevator/EJECT", 0.0)),
    CA(new LoggedTunableNumber("Elevator/Coral on top of alge", 0.0));

    // Supplier that provides the setpoint height for the goal
    private final DoubleSupplier elevatorSetpointSupplier;

    /**
     * Retrieves the height associated with the goal.
     *
     * @return The height in meters.
     */
    public double get() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  @RequiredArgsConstructor
  public enum flywheelGoal {
    // Define the STOW goal with an angle of 0 degrees
    STOW(() -> 0, () -> 0),
    // Define ANGLE1 goal with a tunable setpoint of 45 degrees
    INTAKE(
        new LoggedTunableNumber("Top Elevator Flywheel/Intake", -3000),
        new LoggedTunableNumber("Bottom Elevator Flywheel/Intake", 3000)),
    NET(
        new LoggedTunableNumber("Top Elevator Flywheel/Eject", 3000),
        new LoggedTunableNumber("Bottom Elevator Flywheel/Eject", -3000)),
    EJECT(
        new LoggedTunableNumber("Top Elevator Flywheel/Eject", 5000),
        new LoggedTunableNumber("Bottom Elevator Flywheel/Eject", -5000));

    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier topFlywheelRPM;
    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier bottomFlywheelRPM;

    // Method to get the setpoint in radians
    public DoubleSupplier getTopRPM() {
      return topFlywheelRPM;
    }
    // Method to get the setpoint in radians
    public DoubleSupplier getBottomRPM() {
      return bottomFlywheelRPM;
    }
  }

  public void setGoal(Goal goal) {
    stopProfile = false;
    double newGoalPosition = goal.get();
    if (newGoalPosition > previousGoalPosition) {
      // New goal is higher than the previous goal.
      isDebouncerNeeded = false;
    } else if (newGoalPosition < previousGoalPosition) {
      // New goal is lower than the previous goal.
      isDebouncerNeeded = true;
    } else {
      isDebouncerNeeded = false;
    }
    previousGoalPosition = newGoalPosition; // Update for next comparison. if ve ever need these
    currentGoalPosition = newGoalPosition;
    setGoal(() -> new State(newGoalPosition, 0.0));
  }

  public void setGoal(Supplier<State> goal) {
    this.goal = goal;
  }

  /**
   * Checks if the elevator has reached its goal position within a small epsilon tolerance.
   *
   * @return True if at goal, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean atGoal() {
    if (EqualsUtil.epsilonEquals(setpoint.position, 0, 1e-3)) return false;
    else return EqualsUtil.epsilonEquals(inputs.positionMeters, goal.get().position, 0.05);
  }

  // Current goal of the elevator, with getter and setter

  // Interface to interact with the elevator hardware
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Feedforward controller for the elevator
  public ElevatorFeedforward ff;

  /**
   * Constructor for the Elevator subsystem.
   *
   * @param io An implementation of the ElevatorIO interface to interact with hardware.
   */
  public Elevator(
      ElevatorIO io, GenericFlywheelsIO bottomFlywheelsIO, GenericFlywheelsIO topFlywheelsIO) {
    this.io = io;
    this.bottomFlywheelsIO = bottomFlywheelsIO;
    this.topFlywheelsIO = topFlywheelsIO;
    io.setBrakeMode(true); // Engage brake mode by default

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> staticCharacterization(voltage.in(Volts)), null, this));

    // Set PID gains for the elevator
    io.setPID(kP.get(), kI.get(), kD.get());

    // Initialize the feedforward controller with current gains
    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Flywheels

    // Configure PID and feedforward gains based on the robot's current mode
    // The simulation mode may require different tuning than the real hardware
    switch (ModeSetter.currentMode) {
      case REAL:
        bottomFlywheelsFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        bottomFlywheelsIO.configurePID(0.8, 0.0, 0.0);
        topFlywheelsFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        topFlywheelsIO.configurePID(0.8, 0.0, 0.0);
        break;
      case REPLAY:
        // On real hardware or replay mode, use these gains
        bottomFlywheelsFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        bottomFlywheelsIO.configurePID(1.0, 0.0, 0.0);
        topFlywheelsFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        topFlywheelsIO.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        // In simulation, use different feedforward and PID gains to achieve stable behavior in the
        // model
        bottomFlywheelsFFModel = new SimpleMotorFeedforward(0.01, 0.00103, 0.0);
        bottomFlywheelsIO.configurePID(0.4, 0.0, 0.0);
        topFlywheelsFFModel = new SimpleMotorFeedforward(0.01, 0.00103, 0.0);
        topFlywheelsIO.configurePID(0.4, 0.0, 0.0);
        break;
      default:
        bottomFlywheelsFFModel = new SimpleMotorFeedforward(0.01, 0.00103, 0.0);
        bottomFlywheelsIO.configurePID(0.4, 0.0, 0.0);
        topFlywheelsFFModel = new SimpleMotorFeedforward(0.01, 0.00103, 0.0);
        topFlywheelsIO.configurePID(0.4, 0.0, 0.0);
        break;
    }
  }

  public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
    this.coastOverride = coastOverride;
    this.disabledOverride =
        () -> this.disabledOverride.getAsBoolean() && disabledOverride.getAsBoolean();
  }

  /**
   * Enables or disables brake mode for the elevator.
   *
   * @param enabled True to enable brake mode, false to disable.
   */
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  /**
   * Periodic method called regularly by the scheduler to update the elevator's state. Handles PID
   * control, feedforward calculations, and state updates.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    motorDisconnectedAlert.set(!(inputs.leaderMotorConnected && inputs.followerMotorConnected));

    // Update inputs from the hardware implementation
    bottomFlywheelsIO.updateInputs(bottomFlywheelsInputs);
    // Process and log these inputs for debugging and telemetry
    Logger.processInputs("Bottom Elevator Flywheel", bottomFlywheelsInputs);

    // Update inputs from the hardware implementation
    topFlywheelsIO.updateInputs(topFlywheelsInputs);
    // Process and log these inputs for debugging and telemetry
    Logger.processInputs("Top Elevator Flywheel", topFlywheelsInputs);

    // Set coast mode
    setBrakeMode(!coastOverride.getAsBoolean());

    // Update PID and feedforward controllers if gains have changed
    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), 0.0, kD.get());
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    final boolean shouldRunProfile =
        !stopProfile && !coastOverride.getAsBoolean() && !disabledOverride.getAsBoolean() && homed;

    Logger.recordOutput("Elevator/RunningProfile", shouldRunProfile);

    // Run profile
    if (shouldRunProfile) {
      // Clamp goal
      var goalState =
          new State(
              MathUtil.clamp(
                  goal.get().position,
                  ElevatorConstants.kMinElevatorHeightMeters,
                  ElevatorConstants.kMaxElevatorHeightMeters),
              0.0);

      if (isDebouncerNeeded) {
        // If a downward move is commanded, update only after the debouncer fires.
        if (downDebouncer.calculate(true)) {
          setpoint = profile.calculate(0.02, setpoint, goalState);
        }
        // Else: retain the current setpoint (i.e. do not update it yet)
      } else {
        // For upward moves or when no debounce is needed, update normally.
        setpoint = profile.calculate(0.02, setpoint, goalState);
      }

      double feedforwardOutput = ff.calculateWithVelocities(setpoint.velocity, setpoint.velocity);
      io.runPosition(
          setpoint.position,
          kS.get() * Math.signum(setpoint.velocity) // Magnitude irrelevant
              + kG.get());

      // Log state
      // Log various elevator states for telemetry
      Logger.recordOutput("Elevator/SetpointHeight", setpoint.position);
      Logger.recordOutput("Elevator/SetpointVelocity", setpoint.velocity);
      Logger.recordOutput("Elevator/currentHeight", inputs.positionMeters);
      Logger.recordOutput("Elevator/Goal", goal.get().position);
    } else {
      Logger.recordOutput("Elevator/SetpointHeight", 0.0);
      Logger.recordOutput("Elevator/SetpointVelocity", 0.0);
      Logger.recordOutput("Elevator/currentHeight", 0.0);
      Logger.recordOutput("Elevator/Goal", 0.0);
    }
    // Log state
    Logger.recordOutput("Elevator/CoastOverride", coastOverride.getAsBoolean());
    Logger.recordOutput("Elevator/DisabledOverride", disabledOverride.getAsBoolean());
    Logger.recordOutput(
        "Elevator/MeasuredVelocityMetersPerSec",
        inputs.velocityMetersPerSecond * ElevatorConstants.kElevatorDrumRadius);
  }

  public Command staticCharacterization(double outputRampRate) {
    final StaticCharacterizationState state = new StaticCharacterizationState();
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              stopProfile = true;
              timer.restart();
            },
            () -> {
              state.characterizationOutput = outputRampRate * timer.get();
              io.runOpenLoop(state.characterizationOutput);
              Logger.recordOutput(
                  "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
        .until(() -> inputs.velocityMetersPerSecond >= staticCharacterizationVelocityThresh.get())
        .finallyDo(
            () -> {
              stopProfile = false;
              timer.stop();
              Logger.recordOutput("Elevator/CharacterizationOutput", state.characterizationOutput);
            });
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              stopProfile = true;
              homed = false;
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityMetersPerSecond) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              homed = true;
            })
        .finallyDo(
            () -> {
              homedPosition = inputs.positionMeters;
              stopProfile = true;
            });
  }

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }

  public double getHeight() {
    return inputs.positionMeters;
  }

  /**
   * Runs the flywheel open-loop at the specified voltage.
   *
   * @param volts The voltage command to apply to the flywheel motor(s).
   */
  public void runBottomFlywheelVolts(double volts) {
    bottomFlywheelsIO.setVoltage(volts);
  }

  public void runTopFlywheelVolts(double volts) {
    topFlywheelsIO.setVoltage(volts);
  }

  public void runFlywheelsVelocity(double topVelocityRPM, double bottomVelocityRPM) {
    // Convert velocity from RPM to radians per second for internal calculations
    var topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRPM);
    var bottomVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(bottomVelocityRPM);
    // Calculate the feedforward voltage using the feedforward model and the current velocity
    // This helps the motor controller achieve the desired speed more effectively
    bottomFlywheelsIO.setVelocity(
        bottomVelocityRadPerSec,
        bottomFlywheelsFFModel.calculateWithVelocities(
            bottomFlywheelsInputs.velocityRadPerSec, bottomVelocityRadPerSec));

    // Log the commanded setpoint RPM for telemetry
    Logger.recordOutput("Botttom Elevator Flywheel/SetpointRPM", bottomVelocityRPM);

    topFlywheelsIO.setVelocity(
        topVelocityRadPerSec,
        topFlywheelsFFModel.calculateWithVelocities(
            topFlywheelsInputs.velocityRadPerSec, topVelocityRadPerSec));

    // Log the commanded setpoint RPM for telemetry
    Logger.recordOutput("Top Elevator Flywheel/SetpointRPM", topVelocityRPM);
    // Log the commanded setpoint RPM for telemetry
    Logger.recordOutput("Bottom Elevator Flywheel/SetpointRPM", bottomVelocityRPM);
  }

  /** Stops the flywheel by halting motor outputs. */
  public void stopFlywheels() {
    topFlywheelsIO.stop();
    bottomFlywheelsIO.stop();
  }

  /**
   * Returns the current velocity of the flywheel in RPM.
   *
   * @return The current flywheel speed in rotations per minute.
   */
  @AutoLogOutput
  public double getBottomFlywheelsVelocityRPM() {
    // Converts radians per second to RPM for more human-readable units
    return Units.radiansPerSecondToRotationsPerMinute(bottomFlywheelsInputs.velocityRadPerSec);
  }

  @AutoLogOutput
  public double getTopFlywheelsVelocityRPM() {
    // Converts radians per second to RPM for more human-readable units
    return Units.radiansPerSecondToRotationsPerMinute(topFlywheelsInputs.velocityRadPerSec);
  }
}
