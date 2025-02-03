package frc.robot.subsystems.superstructure.algeManipulator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

// Define the Alge Manipulator class
public class AlgeManipulator extends SubsystemBase {

  // Define tunable PID and feedforward constants with default values from
  // AlgeManipulatorConstants.gains
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Alge Manipulator/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Alge Manipulator/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Alge Manipulator/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Alge Manipulator/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Alge Manipulator/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Alge Manipulator/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Alge Manipulator/Gains/kG", gains.ffkG());

  // Define tunable maximum velocity and acceleration for arm motion constraints
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Alge Manipulator/Velocity", AlgeManipulatorConstants.kArmMotionConstraint.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Alge Manipulator/Acceleration",
          AlgeManipulatorConstants.kArmMotionConstraint.maxAcceleration);

  // Define a tunable upper limit for partial stow in degrees
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Alge Manipulator/PartialStowUpperLimitDegrees", 30.0);

  // Define tunable lower and upper angle limits in degrees
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber(
          "Alge Manipulator/LowerLimitDegrees", AlgeManipulatorConstants.minAngle);
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber(
          "Alge Manipulator/UpperLimitDegrees", AlgeManipulatorConstants.maxAngle);

  // Define suppliers to determine if the arm should be disabled, in coast mode, or half stowed
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  public BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  // Flag to indicate if brake mode is enabled
  private boolean brakeModeEnabled = true;

  // Flag to indicate if the arm is in characterization mode
  private boolean characterizing = false;
  // Interface for Arm IO operations
  private AlgeManipulatorIO io;
  // Creates a SysIdRoutine
  SysIdRoutine routine = // Configure SysId
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              null,
              null,
              (state) -> Logger.recordOutput("Alge Manipulator/SysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
              (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  ;

  // Variable to store the goal angle in radians
  private double goalAngle;

  // Define an enumeration for different arm goals with corresponding setpoints
  @RequiredArgsConstructor
  public enum Goal {
    // Define the STOW goal with an angle of 0 degrees
    STOW(() -> 0),
    // Define ANGLE1 goal with a tunable setpoint of 45 degrees
    GROUND_ALGE_INTAKE(new LoggedTunableNumber("Alge Manipulator/Ground intake for alge", 0.0)),
    L1(new LoggedTunableNumber("Alge Manipulator/L1", 0.0)),
    PROCESSOR(new LoggedTunableNumber("Alge Manipulator/Processor", 75.0)),
    NET(new LoggedTunableNumber("Alge Manipulator/Net", 0.0)),
    EJECT(new LoggedTunableNumber("Alge Manipulator/Eject", 0.0)),
    SKYFALL(new LoggedTunableNumber("Alge Manipulator/Skyfall", 0.0)), // Drop alge from reef
    CA(new LoggedTunableNumber("Alge Manipulator/Coral on top of alge", 0.0)),
    GROUND_CORAL_INTAKE(new LoggedTunableNumber("Alge Manipulator/Ground intake for coral", 0.0));

    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier armSetpointSupplier;

    // Method to get the setpoint in radians
    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  // Method to determine if the arm is at its goal position within a small epsilon
  @AutoLogOutput(key = "Alge Manipulator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
  }

  // Getter and Setter for the current goal with auto-logging
  @Getter @Setter private Goal goal = Goal.STOW;

  // Inputs for logging purposes
  private final AlgeManipulatorIOInputsAutoLogged inputs = new AlgeManipulatorIOInputsAutoLogged();

  // Trapezoidal motion profile for smooth arm movements
  public TrapezoidProfile profile;
  // Current state of the motion profile
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  // Visualizers for measuring, setpoint, and goal positions
  private final AlgeManipulatorVisualizer measuredVisualizer;
  private final AlgeManipulatorVisualizer setpointVisualizer;
  private final AlgeManipulatorVisualizer goalVisualizer;

  // Flag to track if the arm was not in autonomous mode
  private boolean wasNotAuto = false;

  // Arm feedforward controller
  public ArmFeedforward ff;

  // Constructor for the Arm class, initializing IO and controllers
  public AlgeManipulator(AlgeManipulatorIO io) {
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
    measuredVisualizer = new AlgeManipulatorVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new AlgeManipulatorVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new AlgeManipulatorVisualizer("Goal", Color.kBlue);
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
          Units.degreesToRadians(AlgeManipulatorConstants.minAngle),
          Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
    } else {
      // Return the minimum angle in radians
      return Units.degreesToRadians(AlgeManipulatorConstants.minAngle);
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
    Logger.processInputs("Alge Manipulator", inputs);

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
              goalAngle, Units.degreesToRadians(AlgeManipulatorConstants.minAngle))
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
      Logger.recordOutput("Alge Manipulator/GoalAngle", goalAngle);
    }

    // Update the measured visualizer with the current position
    measuredVisualizer.update(inputs.positionRads);
    // Update the setpoint visualizer with the current setpoint position
    setpointVisualizer.update(setpointState.position);

    // Record various outputs for logging
    Logger.recordOutput("Alge Manipulator/SetpointAngle", setpointState.position);
    Logger.recordOutput("Alge Manipulator/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Alge Manipulator/currentDeg", Units.radiansToDegrees(inputs.positionRads));
    Logger.recordOutput("Alge Manipulator/Goal", goal);
  }

  /** Runs the arm with the specified output */
  public void runCharacterization(double output) {
    io.runOpenLoop(output);
  }

  @AutoLogOutput(key = "Alge Manipulator/motorHasReachedForwardLimit")
  public boolean motorHasReachedForwardLimit() {
    return EqualsUtil.epsilonEquals(
        inputs.positionRads, Units.degreesToRadians(AlgeManipulator.upperLimitDegrees.get()), 1e-3);
  }

  @AutoLogOutput(key = "Alge Manipulator/motorHasReachedBackwardsLimit")
  public boolean motorHasReachedBackwardsLimit() {
    return EqualsUtil.epsilonEquals(
        inputs.positionRads, Units.degreesToRadians(AlgeManipulator.lowerLimitDegrees.get()), 1e-3);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public void quastaticTest() {
    sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        .until(() -> motorHasReachedForwardLimit())
        .andThen(
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                .until(() -> motorHasReachedForwardLimit()));
  }

  public void dynamicTest() {
    sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        .until(() -> motorHasReachedForwardLimit())
        .andThen(
            sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> motorHasReachedBackwardsLimit()));
  }
}
