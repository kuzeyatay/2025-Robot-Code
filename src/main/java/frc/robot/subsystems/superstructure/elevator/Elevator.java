package frc.robot.subsystems.superstructure.elevator;

import static frc.robot.subsystems.superstructure.elevator.ElevatorConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
 */
public class Elevator extends SubsystemBase {

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

  private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
      new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.0675);

  private final Alert motorDisconnectedAlert =
      new Alert("Elevator motor disconnected!", Alert.AlertType.kWarning);

  // Suppliers to handle various override conditions
  private BooleanSupplier coastOverride = () -> false;
  private BooleanSupplier disabledOverride = DriverStation::isDisabled;
  private final ExponentialProfile profile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              ElevatorConstants.kElevatorMaxV, kV.getAsDouble(), kA.getAsDouble()));

  @AutoLogOutput(key = "Elevator/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @Getter private State setpoint = new State();
  private Supplier<State> goal = State::new;
  private boolean stopProfile = false;

  /**
   * Enum representing predefined goal positions for the elevator. Each goal has an associated
   * height supplier.
   */
  @RequiredArgsConstructor
  // FIXME
  public enum Goal {
    STOW(new LoggedTunableNumber("Elevator/STOW", 0.0)),
    L1(new LoggedTunableNumber("Elevator/L1", 0.0)),
    L2(new LoggedTunableNumber("Elevator/L2", 0.8)),
    L3(new LoggedTunableNumber("Elevator/L3", 0.0)),
    L4(new LoggedTunableNumber("Elevator/L4", 0.0)),
    PROCESSOR(new LoggedTunableNumber("Elevator/PROCESSOR", 0.0)),
    NET(new LoggedTunableNumber("Elevator/NET", 0.0)),
    EJECT(new LoggedTunableNumber("Elevator/EJECT", 0.0)),
    CA(new LoggedTunableNumber("Elevator/Coral on top of alge", 0.0));

    // Supplier that provides the setpoint height for the goal
    private final DoubleSupplier elevatorSetpointSupplier;

    /**
     * Retrieves the height associated with the goal.
     *
     * @return The height in meters.
     */
    private double get() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public void setGoal(Goal goal) {
    setGoal(() -> new State(goal.get(), 0.0));
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
    return EqualsUtil.epsilonEquals(setpoint.position, goal.get().position, 1e-3);
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
  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true); // Engage brake mode by default

    // Set PID gains for the elevator
    io.setPID(kP.get(), kI.get(), kD.get());

    // Initialize the feedforward controller with current gains
    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
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
        !stopProfile && !coastOverride.getAsBoolean() && !disabledOverride.getAsBoolean();

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
      setpoint = profile.calculate(0.02, setpoint, goalState);
      double feedforwardOutput = ff.calculateWithVelocities(setpoint.velocity, setpoint.velocity);
      io.runPosition(setpoint.position, feedforwardOutput);

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

  private static class StaticCharacterizationState {
    public double characterizationOutput = 0.0;
  }
}
