// Copybottom (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LinearProfile;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  private static final LoggedTunableNumber shootingtopRpm =
      new LoggedTunableNumber("Flywheels/ShootingtopRpm", 5066.0);
  private static final LoggedTunableNumber shootingbottomRpm =
      new LoggedTunableNumber("Flywheels/ShootingbottomRpm", 7733.0);
  private static final LoggedTunableNumber prepareShootMultiplier =
      new LoggedTunableNumber("Flywheels/PrepareShootMultiplier", 1.0);
  private static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Flywheels/IntakingRpm", -3000.0);
  private static final LoggedTunableNumber demoIntakingRpm =
      new LoggedTunableNumber("Flywheels/DemoIntakingRpm", -250.0);
  private static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
  private static final LoggedTunableNumber poopingRpm =
      new LoggedTunableNumber("Flywheels/PoopingRpm", 3000.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile topProfile;
  private final LinearProfile bottomProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  @Setter private BooleanSupplier prepareShootSupplier = () -> true;

  // Disconnected alerts
  private final Alert topDisconnected =
      new Alert("top flywheel disconnected!", Alert.AlertType.kWarning);
  private final Alert bottomDisconnected =
      new Alert("bottom flywheel disconnected!", Alert.AlertType.kWarning);

  @RequiredArgsConstructor
  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    SHOOT(shootingtopRpm, shootingbottomRpm),
    INTAKE(intakingRpm, intakingRpm),
    DEMO_INTAKE(demoIntakingRpm, demoIntakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier topGoal;
    private final DoubleSupplier bottomGoal;

    private double gettopGoal() {
      return topGoal.getAsDouble();
    }

    private double getbottomGoal() {
      return bottomGoal.getAsDouble();
    }
  }

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  @Getter
  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal goal = Goal.IDLE;

  @AutoLogOutput(key = "Flywheels/CURRENT TO MUCH DUMBASS")
  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.topSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.bottomSupplyCurrentAmps) > 50.0;
  }

  public Flywheels(FlywheelsIO io) {
    this.io = io;

    topProfile = new LinearProfile(maxAcceleration.get(), 0.02);
    bottomProfile = new LinearProfile(maxAcceleration.get(), 0.02);

    setDefaultCommand(runOnce(() -> setGoal(Goal.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Set alerts
    topDisconnected.set(!inputs.topMotorConnected);
    bottomDisconnected.set(!inputs.bottomMotorConnected);

    // Check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          topProfile.setMaxAcceleration(maxAcceleration.get());
          bottomProfile.setMaxAcceleration(maxAcceleration.get());
        },
        maxAcceleration);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Check if profile needs to be reset
    if (!closedLoop && wasClosedLoop) {
      topProfile.reset();
      bottomProfile.reset();
      wasClosedLoop = false;
    }

    // Get goal
    double topGoal = goal.gettopGoal();
    double bottomGoal = goal.getbottomGoal();
    boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      topGoal = Goal.SHOOT.gettopGoal() * prepareShootMultiplier.get();
      bottomGoal = Goal.SHOOT.getbottomGoal() * prepareShootMultiplier.get();
    }

    // Run to setpoint
    if (closedLoop || idlePrepareShoot) {
      // Update goals
      topProfile.setGoal(topGoal);
      bottomProfile.setGoal(bottomGoal);
      double topSetpoint = topProfile.calculateSetpoint();
      double bottomSetpoint = bottomProfile.calculateSetpoint();
      io.runVelocity(
          topSetpoint, bottomSetpoint, ff.calculate(topSetpoint), ff.calculate(bottomSetpoint));

    } else if (goal == Goal.IDLE) {

      io.stop();
    }

    Logger.recordOutput("Flywheels/SetpointtopRpm", topProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointbottomRpm", bottomProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoaltopRpm", topGoal);
    Logger.recordOutput("Flywheels/GoalbottomRpm", bottomGoal);
  }

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    if (goal == Goal.CHARACTERIZING || goal == Goal.IDLE) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      this.goal = goal;
      return; // Don't set a goal
    }
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.goal == goal;
    // Enable close loop
    if (!closedLoop) {
      topProfile.setGoal(goal.gettopGoal(), inputs.topVelocityRpm);
      bottomProfile.setGoal(goal.getbottomGoal(), inputs.bottomVelocityRpm);
      closedLoop = true;
    }
    this.goal = goal;
  }

  /** Runs flywheels at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setGoal(Goal.CHARACTERIZING);
    io.runCharacterizationtop(input);
    io.runCharacterizationbottom(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.topVelocityRpm + inputs.bottomVelocityRpm) / 2.0;
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return goal == Goal.IDLE
        || (topProfile.getCurrentSetpoint() == goal.gettopGoal()
            && bottomProfile.getCurrentSetpoint() == goal.getbottomGoal());
  }

  public Command shootCommand() {
    return startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Intake");
  }

  public Command demoIntakeCommand() {
    return startEnd(() -> setGoal(Goal.DEMO_INTAKE), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Demo Intake");
  }

  public Command ejectCommand() {
    return startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE))
        .withName("Flywheels Eject");
  }

  public Command poopCommand() {
    return startEnd(() -> setGoal(Goal.POOP), () -> setGoal(Goal.IDLE)).withName("Flywheels Poop");
  }
}
