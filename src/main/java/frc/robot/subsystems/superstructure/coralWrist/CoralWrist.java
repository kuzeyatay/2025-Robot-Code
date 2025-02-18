package frc.robot.subsystems.superstructure.coralWrist;

import static frc.robot.subsystems.superstructure.coralWrist.CoralWristConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

// Define the Arm class as a generic subsystem
public class CoralWrist extends SubsystemBase {

  // Define tunable PID and feedforward constants with default values from CoralWristConstants.gains
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Coral Wrist/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Coral Wrist/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Coral Wrist/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Coral Wrist/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Coral Wrist/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Coral Wrist/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Coral Wrist/Gains/kG", gains.ffkG());

  // Define tunable maximum velocity and acceleration for arm motion constraints
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Coral Wrist/Velocity", CoralWristConstants.kArmMotionConstraint.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Coral Wrist/Acceleration", CoralWristConstants.kArmMotionConstraint.maxAcceleration);

  // Define a tunable upper limit for partial stow in degrees
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Coral Wrist/PartialStowUpperLimitDegrees", 30.0);

  // Define tunable lower and upper angle limits in degrees
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Coral Wrist/LowerLimitDegrees", CoralWristConstants.minAngle);
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Coral Wrist/UpperLimitDegrees", CoralWristConstants.maxAngle);

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Coral Wrist/HomingVolts", -1.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Coral Wrist/HomingTimeSecs", 0.35);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Coral Wrist/ Homing Velocity Thresh", 0.1);

  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());
  // Define suppliers to determine if the arm should be disabled, in coast mode, or half stowed
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  public BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  // Flag to indicate if brake mode is enabled
  private boolean brakeModeEnabled = true;

  // Flag to indicate if the arm is in characterization mode
  private boolean characterizing = false;

  // Variable to store the goal angle in radians
  private double goalAngle;

  // Define an enumeration for different arm goals with corresponding setpoints
  @RequiredArgsConstructor
  public enum Goal {
    // Define the STOW goal with an angle of 0 degrees
    STOW(() -> 0),
    // Define ANGLE1 goal with a tunable setpoint of 45 degrees
    L1(new LoggedTunableNumber("Coral Wrist/L1", 45.0)),
    L2(new LoggedTunableNumber("Coral Wrist/L2", 45.0)),
    L3(new LoggedTunableNumber("Coral Wrist/L3", 45.0)),
    L4(new LoggedTunableNumber("Coral Wrist/L4", 45.0)),
    CHUTE(new LoggedTunableNumber("Coral Wrist/Chute", 45.0)),
    EJECT(new LoggedTunableNumber("Coral Wrist/Eject", 45.0));

    // Define ANGLE2 goal with a tunable setpoint of 90 degrees

    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier armSetpointSupplier;

    // Method to get the setpoint in radians
    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  // Method to determine if the arm is at its goal position within a small epsilon
  @AutoLogOutput(key = "Cora lWrist/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
  }

  // Getter and Setter for the current goal with auto-logging
  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;

  // Interface for Arm IO operations
  private CoralWristIO io;
  // Inputs for logging purposes
  private final CoralWristIOInputsAutoLogged inputs = new CoralWristIOInputsAutoLogged();

  // Trapezoidal motion profile for smooth arm movements
  public TrapezoidProfile profile;
  // Current state of the motion profile
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  // Visualizers for measuring, setpoint, and goal positions
  private final CoralWristVisualizer measuredVisualizer;
  private final CoralWristVisualizer setpointVisualizer;
  private final CoralWristVisualizer goalVisualizer;

  // Flag to track if the arm was not in autonomous mode
  private boolean wasNotAuto = false;
  public static double homedPosition = 0.0;
  public static boolean homed = true;
  // Arm feedforward controller
  public ArmFeedforward ff;

  // Constructor for the Arm class, initializing IO and controllers
  public CoralWrist(CoralWristIO io) {
    this.io = io;
    // Set brake mode to enabled initially
    io.setBrakeMode(true);
    // Initialize the motion profile with current max velocity and acceleration
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    // Set initial PID constants
    io.setPID(kP.get(), kI.get(), kD.get());
    // Initialize the feedforward controller with current constants
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Re-initialize the motion profile (redundant line, can be removed if not needed)
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

    // Initialize visualizers with specific colors
    measuredVisualizer = new CoralWristVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new CoralWristVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new CoralWristVisualizer("Goal", Color.kBlue);
  }

  // Method to set override suppliers for disabling, coasting, and half stowing the arm
  public void setOverrides(
      BooleanSupplier disableOverride,
      BooleanSupplier coastOverride,
      BooleanSupplier halfStowOverride) {
    // Combine disableOverride with DriverStation's disabled state
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    // Set the coast and half stow suppliers
    coastSupplier = coastOverride;
    halfStowSupplier = halfStowOverride;
  }

  // Method to calculate the stow angle based on current mode and half stow supplier
  private double getStowAngle() {
    // Check if in teleop mode and not half stowed
    if (DriverStation.isTeleopEnabled() && !halfStowSupplier.getAsBoolean()) {
      // Clamp the setpoint position between min angle and partial stow upper limit
      return MathUtil.clamp(
          setpointState.position,
          Units.degreesToRadians(CoralWristConstants.minAngle),
          Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
    } else {
      // Return the minimum angle in radians
      return Units.degreesToRadians(CoralWristConstants.minAngle);
    }
  }

  // Method to enable or disable brake mode
  public void setBrakeMode(boolean enabled) {
    // If the desired state is the same as current, do nothing
    if (brakeModeEnabled == enabled) return;
    // Update the brake mode flag
    brakeModeEnabled = enabled;
    // Set the brake mode on the IO
    io.setBrakeMode(brakeModeEnabled);
  }

  // Periodic method called regularly to update the subsystem
  @Override
  public void periodic() {
    // Update IO inputs
    io.updateInputs(inputs);
    // Process inputs for logging
    Logger.processInputs("Coral Wrist", inputs);

    // Update brake mode based on coast supplier
    setBrakeMode(!coastSupplier.getAsBoolean());

    // Update PID constants if they have changed
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    // Update feedforward controller if constants have changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    // Check if the arm should be disabled or if in autonomous mode and reset was not auto
    if (disableSupplier.getAsBoolean()
        || (ModeSetter.currentMode) == Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto) {
      // Stop the arm motor
      io.stop();
      // Reset the motion profile setpoint state to current position with zero velocity
      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
    }
    // Track if the robot was not previously in autonomous mode
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Determine if the profile should run based on various flags
    if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
      // Set the goal angle based on the current goal
      goalAngle = goal.getRads();
      // If the goal is to stow, calculate the stow angle
      if (goal == Goal.STOW) {
        goalAngle = getStowAngle();
      }
      // Calculate the next state in the motion profile
      setpointState =
          profile.calculate(
              0.02,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goalAngle,
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));
      // If stowing and at the minimum angle and at goal, stop the motor
      if (goal == Goal.STOW
          && EqualsUtil.epsilonEquals(
              goalAngle, Units.degreesToRadians(CoralWristConstants.minAngle))
          && atGoal()) {
        io.stop();
      } else {

        // Run the setpoint with calculated feedforward
        io.runSetpoint(
            setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
      }

      // Update the goal visualizer with the current goal angle
      goalVisualizer.update(goalAngle);
      // Record the goal angle for logging
      Logger.recordOutput("Coral Wrist/GoalAngle", goalAngle);
    }

    // Update the measured visualizer with the current position
    measuredVisualizer.update(inputs.positionRads);
    // Update the setpoint visualizer with the current setpoint position
    setpointVisualizer.update(setpointState.position);

    // Record various outputs for logging
    Logger.recordOutput("Coral Wrist/SetpointAngle", setpointState.position);
    Logger.recordOutput("Coral Wrist/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Coral Wrist/currentDeg", Units.radiansToDegrees(inputs.positionRads));
    Logger.recordOutput("Coral Wrist/Goal", goal);
  }

  public Command homingSequence() {
    return Commands.startRun(
            () -> {
              homingDebouncer = new Debouncer(homingTimeSecs.get());
              homingDebouncer.calculate(false);
            },
            () -> {
              io.runVolts(homingVolts.get());
              homed =
                  homingDebouncer.calculate(
                      Math.abs(inputs.velocityRadsPerSec) <= homingVelocityThresh.get());
            })
        .until(() -> homed)
        .andThen(
            () -> {
              homed = true;
            })
        .andThen(
            () -> {
              io.runSetpoint(homedPosition, 0.0);
              homedPosition = inputs.positionRads;
            })
        .finallyDo(() -> io.zero());
  }

  public void setAngle(Goal goal) {
    io.runSetpoint(goal.getRads() + goalAngle, 0);
  }

  public void setAngle(double goal) {
    io.runSetpoint(Units.degreesToRadians(goal) + homedPosition, 0);
  }
}
