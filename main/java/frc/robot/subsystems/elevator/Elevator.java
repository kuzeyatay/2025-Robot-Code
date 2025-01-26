package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSetter;
import frc.robot.ModeSetter.Mode;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
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

  // Motion constraints for the elevator's trapezoidal profile
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Elevator/Velocity", ElevatorConstants.elevatorMotionConstraint.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Elevator/Acceleration", ElevatorConstants.elevatorMotionConstraint.maxAcceleration);

  // Physical limits for the elevator's height to prevent overextension
  private static final LoggedTunableNumber lowerLimitHeight =
      new LoggedTunableNumber(
          "Elevator/LowerLimitMeters", ElevatorConstants.kMinElevatorHeightMeters);
  private static final LoggedTunableNumber upperLimitHeight =
      new LoggedTunableNumber(
          "Elevator/UpperLimitMeters", ElevatorConstants.kMaxElevatorHeightMeters);

  // Suppliers to handle various override conditions
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  public BooleanSupplier coastSupplier = () -> false;
  public BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  // Flag to indicate if the elevator is in characterization mode
  private boolean characterizing = false;

  // Desired goal height for the elevator
  private double goalHeight;

  /**
   * Enum representing predefined goal positions for the elevator. Each goal has an associated
   * height supplier.
   */
  @RequiredArgsConstructor
  public enum Goal {
    STOW(() -> 0),
    HIGH(new LoggedTunableNumber("Elevator/HIGH", 1)),
    MID(new LoggedTunableNumber("Elevator/MID", 0.5));

    // Supplier that provides the setpoint height for the goal
    private final DoubleSupplier elevatorSetpointSupplier;

    /**
     * Retrieves the height associated with the goal.
     *
     * @return The height in meters.
     */
    private double getHeight() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  /**
   * Checks if the elevator has reached its goal position within a small epsilon tolerance.
   *
   * @return True if at goal, false otherwise.
   */
  @AutoLogOutput(key = "Elevator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalHeight, 1e-3);
  }

  // Current goal of the elevator, with getter and setter
  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;

  // Interface to interact with the elevator hardware
  private ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Trapezoidal motion profile for smooth elevator movement
  public TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  // Visualizers for monitoring elevator states
  private final ElevatorVisualizer measuredVisualizer;
  private final ElevatorVisualizer setpointVisualizer;
  private final ElevatorVisualizer goalVisualizer;

  // Flag to track if the robot was not in autonomous mode
  private boolean wasNotAuto = false;

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

    // Initialize the trapezoidal profile with current motion constraints
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Set PID gains for the elevator
    io.setPID(kP.get(), kI.get(), kD.get());

    // Initialize the feedforward controller with current gains
    ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Re-initialize the trapezoidal profile (redundant, can be removed)
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Initialize visualizers with distinct colors for clarity
    measuredVisualizer = new ElevatorVisualizer("Measured", Color.kPurple);
    setpointVisualizer = new ElevatorVisualizer("Setpoint", Color.kBrown);
    goalVisualizer = new ElevatorVisualizer("Goal", Color.kKhaki);
  }

  /**
   * Sets override suppliers for disabling, coasting, and half-stowing the elevator.
   *
   * @param disableOverride Supplier that returns true to disable the elevator.
   * @param coastOverride Supplier that returns true to enable coasting mode.
   * @param halfStowOverride Supplier that returns true to half-stow the elevator.
   */
  public void setOverrides(
      BooleanSupplier disableOverride,
      BooleanSupplier coastOverride,
      BooleanSupplier halfStowOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    coastSupplier = coastOverride;
    halfStowSupplier = halfStowOverride;
  }

  /**
   * Retrieves the stow height for the elevator.
   *
   * @return The stow height in meters.
   */
  private double getStowHeight() {
    return ElevatorConstants.kMinElevatorHeightMeters;
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
    // Update inputs from the elevator hardware
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update brake mode based on the coast supplier
    setBrakeMode(!coastSupplier.getAsBoolean());

    // Update PID and feedforward controllers if gains have changed
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    // Check if the elevator should be disabled (e.g., robot is disabled or in simulation auto mode)
    if (disableSupplier.getAsBoolean()
        || (ModeSetter.currentMode == Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto)) {
      io.stop(); // Stop all elevator motors
      // Reset the motion profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.positionMeters, 0);
    }

    // Track if the robot was not in autonomous mode
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Only run the motion profile if not characterizing, not in coast mode, and not disabled
    if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
      // Determine the goal height based on the current goal
      goalHeight = goal.getHeight();
      if (goal == Goal.STOW) {
        goalHeight = getStowHeight();
      }

      // Calculate the next state in the motion profile
      setpointState =
          profile.calculate(
              0.02, // Time step (assuming 20ms periodic rate)
              setpointState, // Current state
              new TrapezoidProfile.State(
                  MathUtil.clamp(goalHeight, lowerLimitHeight.get(), upperLimitHeight.get()),
                  0.0)); // Goal state

      // If stowing and at the minimum height, stop the elevator
      if (goal == Goal.STOW
          && EqualsUtil.epsilonEquals(goalHeight, ElevatorConstants.kMinElevatorHeightMeters)
          && atGoal()) {
        io.stop();
      } else {
        // Apply height control with feedforward
        io.setHeight(setpointState.position, ff.calculate(setpointState.velocity) / 12.0);
      }

      // Update visualizers with the current goal height
      goalVisualizer.update(goalHeight);
      Logger.recordOutput("Elevator/GoalHeight", goalHeight);
    }

    // Update visualizers with measured and setpoint heights
    measuredVisualizer.update(inputs.positionMeters);
    setpointVisualizer.update(setpointState.position);

    // Log various elevator states for telemetry
    Logger.recordOutput("Elevator/SetpointHeight", setpointState.position);
    Logger.recordOutput("Elevator/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Elevator/currentHeight", inputs.positionMeters);
    Logger.recordOutput("Elevator/Goal", goal);
  }
}
