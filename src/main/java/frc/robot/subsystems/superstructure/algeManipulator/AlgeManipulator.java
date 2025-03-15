package frc.robot.subsystems.superstructure.algeManipulator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulatorConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ModeSetter;
import frc.robot.ModeSetter.Mode;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIO;
import frc.robot.subsystems.genericFlywheels.GenericFlywheelsIOInputsAutoLogged;
import frc.robot.subsystems.laserCan.LaserCanIO;
import frc.robot.subsystems.laserCan.LaserCanIOInputsAutoLogged;
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
  @AutoLogOutput @Getter private boolean homed = true;
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
  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Alge Manipulatorr/HomingVolts", -1.0);
  private static final LoggedTunableNumber homingTimeSecs =
      new LoggedTunableNumber("Alge Manipulator/HomingTimeSecs", 0.35);
  private static final LoggedTunableNumber homingVelocityThresh =
      new LoggedTunableNumber("Alge Manipulator", 0.1);
  private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());
  private Goal currentGoal = Goal.STOW;

  // Define tunable maximum velocity and acceleration for arm motion constraints
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Alge Manipulator/Velocity", AlgeManipulatorConstants.kArmMotionConstraint.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Alge Manipulator/Acceleration",
          AlgeManipulatorConstants.kArmMotionConstraint.maxAcceleration);

  // Define tunable lower and upper angle limits in degrees
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber(
          "Alge Manipulator/LowerLimitDegrees", AlgeManipulatorConstants.minAngle);
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber(
          "Alge Manipulator/UpperLimitDegrees", AlgeManipulatorConstants.maxAngle);

  private static final LoggedTunableNumber slamUpCurrent =
      new LoggedTunableNumber("Slam/SlamUpCurrent", -40.0);

  // Define suppliers to determine if the arm should be disabled, in coast mode, or half stowed
  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  public BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  // Flag to indicate if brake mode is enabled
  private boolean brakeModeEnabled = true;

  public static double homedPosition = 0.0;
  // IO interface for controlling and reading from the flywheel hardware
  private final GenericFlywheelsIO flywheelIO;
  private final GenericFlywheelsIOInputsAutoLogged flywheelInputs =
      new GenericFlywheelsIOInputsAutoLogged();

  private final SimpleMotorFeedforward flywheelFFModel;
  private LaserCanIO laserIO;
  private final LaserCanIOInputsAutoLogged laserInputs = new LaserCanIOInputsAutoLogged();

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
    STOW(() -> -3),
    BACKWARDS_STOW(() -> 5),
    // Define ANGLE1 goal with a tunable setpoint of 45 degrees
    GROUND_INTAKE(new LoggedTunableNumber("Alge Manipulator/Ground intake for alge", 105)),
    LIMBO_1(new LoggedTunableNumber("Alge Manipulator/L2_L3", 45)),
    LIMBO_2(new LoggedTunableNumber("Alge Manipulator/L3_L4", 50.0)),
    PROCESSOR(new LoggedTunableNumber("Alge Manipulator/Processor", 65)),
    NET(new LoggedTunableNumber("Alge Manipulator/Net", 48)),
    EJECT(new LoggedTunableNumber("Alge Manipulator/Eject", 0.0)),
    SKYFALL(new LoggedTunableNumber("Alge Manipulator/Skyfall", 0.0)), // Drop alge from reef
    CA(new LoggedTunableNumber("Alge Manipulator/Coral on top of alge", 0.0));

    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier armSetpointSupplier;

    // Method to get the setpoint in radians
    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  // Define an enumeration for different arm goals with corresponding setpoints
  @RequiredArgsConstructor
  public enum flywheelGoal {
    // Define the STOW goal with an angle of 0 degrees
    STOW(() -> 0),
    // Define ANGLE1 goal with a tunable setpoint of 45 degrees
    INTAKE(
        new LoggedTunableNumber(
            "Alge Manipulator Flywheels/Ground intake for alge", -4500)), // 2000'di
    PROCESSOR(new LoggedTunableNumber("Alge Manipulator Flywheels/Processor", 4000)),
    NET(new LoggedTunableNumber("Alge Manipulator Flywheels/Net", -6000)),
    EJECT(new LoggedTunableNumber("Alge Manipulator Flywheels/Eject", 3000)),
    SKYFALL(
        new LoggedTunableNumber("Alge Manipulator Flywheels/Skyfall", 3000)); // Drop alge from reef

    // Supplier to provide the arm setpoint in degrees
    private final DoubleSupplier flywheelRPM;

    // Method to get the setpoint in radians
    public DoubleSupplier getRPM() {
      return flywheelRPM;
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

  // Flag to track if the arm was not in autonomous mode
  private boolean wasNotAuto = false;

  // Arm feedforward controller
  public ArmFeedforward ff;

  // Constructor for the Arm class, initializing IO and controllers
  public AlgeManipulator(AlgeManipulatorIO io, GenericFlywheelsIO flywheelIO, LaserCanIO laserIO) {
    io.zero();
    this.io = io;
    this.flywheelIO = flywheelIO;
    this.laserIO = laserIO;

    setAngle(homedPosition);
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

    // Flywheel Setup

    // Configure PID and feedforward gains based on the robot's current mode
    // The simulation mode may require different tuning than the real hardware
    switch (ModeSetter.currentMode) {
      case REAL:
        flywheelFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        flywheelIO.configurePID(0.8, 0.0, 0.0);
        break;
      case REPLAY:
        // On real hardware or replay mode, use these gains
        flywheelFFModel = new SimpleMotorFeedforward(0.0001, 0.000195, 0.0003);
        flywheelIO.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        // In simulation, use different feedforward and PID gains to achieve stable behavior in the
        // model
        flywheelFFModel = new SimpleMotorFeedforward(0.01, 0.00103, 0.0);
        flywheelIO.configurePID(0.4, 0.0, 0.0);
        break;
      default:
        // Default or unrecognized mode uses zero gains
        flywheelFFModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
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
          Units.degreesToRadians(AlgeManipulatorConstants.maxAngle));
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
    flywheelIO.updateInputs(flywheelInputs);
    laserIO.updateInputs(laserInputs);

    // Process inputs for logging
    Logger.processInputs("Alge Manipulator", inputs);
    Logger.processInputs("Alge Flywheels", flywheelInputs);
    Logger.processInputs("Alge LaserCan", laserInputs);

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

      // Run the setpoint with calculated feedforward
      io.runSetpoint(currentGoal.getRads(), 0);
      // Record the goal angle for logging
      Logger.recordOutput("Alge Manipulator/GoalAngle", goalAngle);
    }

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

  public void setAngle(Goal goal) {
    setGoal(goal);
    if (EqualsUtil.epsilonEquals(goal.getRads(), 0, 0.009)) {
      homed = true;
    } else homed = false;
    currentGoal = goal;
  }

  public void setAngle(double goal) {
    io.runSetpoint(Units.degreesToRadians(goal) + homedPosition, 0);
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

  // Flywheels

  /**
   * Runs the flywheel open-loop at the specified voltage.
   *
   * @param volts The voltage command to apply to the flywheel motor(s).
   */
  public void runFlywheelVolts(double volts) {
    flywheelIO.setVoltage(volts);
  }

  /**
   * Runs the flywheel closed-loop to achieve a specified velocity in RPM.
   *
   * @param velocityRPM The target flywheel velocity in rotations per minute (RPM).
   */
  public void runFlywheelVelocity(double velocityRPM) {
    // Convert velocity from RPM to radians per second for internal calculations
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    // Calculate the feedforward voltage using the feedforward model and the current velocity
    // This helps the motor controller achieve the desired speed more effectively
    flywheelIO.setVelocity(
        velocityRadPerSec,
        flywheelFFModel.calculateWithVelocities(
            flywheelInputs.velocityRadPerSec, velocityRadPerSec));

    // Log the commanded setpoint RPM for telemetry
    Logger.recordOutput("Coral Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel by halting motor outputs. */
  public void stopFlywheels() {
    flywheelIO.stop();
  }

  /**
   * Returns the current velocity of the flywheel in RPM.
   *
   * @return The current flywheel speed in rotations per minute.
   */
  @AutoLogOutput
  public double getFlywheelVelocityRPM() {
    // Converts radians per second to RPM for more human-readable units
    return Units.radiansPerSecondToRotationsPerMinute(flywheelInputs.velocityRadPerSec);
  }

  public double getLaserDistance() {
    return laserInputs.m_distance;
  }
}
