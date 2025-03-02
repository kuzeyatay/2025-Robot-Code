package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.led.Leds;
import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulator;
import frc.robot.subsystems.superstructure.coralWrist.CoralWrist;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.util.EqualsUtil;
import frc.robot.util.TimeDelayedBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final AlgeManipulator algeManipulator;
  private final CoralWrist coralWrist;
  private TimeDelayedBoolean mNotHasGamePiece = new TimeDelayedBoolean();

  @AutoLogOutput(key = "Superstructure/test")
  private boolean test = false;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  @AutoLogOutput(key = "Superstructure/HasAlge")
  private boolean hasAlge = false;

  @AutoLogOutput(key = "Superstructure/HasCoral")
  private boolean hasCoral = false;

  @Setter private BooleanSupplier disabledOverride = () -> false;
  private final Alert driverDisableAlert =
      new Alert("Superstructure disabled due to driver override.", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
          Alert.AlertType.kError);

  @AutoLogOutput(key = "Superstructure/Current State")
  public SuperstructureState currentState = SuperstructureState.STOW;

  public Superstructure(Elevator elevator, AlgeManipulator algeManipulator, CoralWrist coralWrist) {
    this.elevator = elevator;
    this.algeManipulator = algeManipulator;
    this.coralWrist = coralWrist;

    // Updating E Stop based on disabled override
    new Trigger(() -> disabledOverride.getAsBoolean())
        .onFalse(Commands.runOnce(() -> isEStopped = false).ignoringDisable(true));
  }

  Timer stowDelay = new Timer();
  double stowStartTime = -1; // Initialize to an invalid time (or 0, if that fits your logic)

  @Override
  public void periodic() {

    Leds.getInstance().hpAttentionLeftAlert = getHasLeftCoral();
    Leds.getInstance().hpAttentionRightAlert = getHasRightCoral();
    Leds.getInstance().hasAlge = getHasAlge();

    hasCoral = getHasLeftCoral() || getHasRightCoral();
    hasAlge = getHasAlge();

    // E Stop Dispenser and Elevator if Necessary
    isEStopped = DriverStation.isEStopped();

    driverDisableAlert.set(disabledOverride.getAsBoolean());
    emergencyDisableAlert.set(isEStopped);
    Leds.getInstance().superstructureEstopped = isEStopped;

    // Log state
    Logger.recordOutput("Superstructure/State", SuperstructureState.values());

    SuperstructureStateData state = currentState.getValue();

    switch (currentState) {
      case STOW -> {
        runHomingSequences();
      }
      case L3_CORAL_EJECT, L4_CORAL_EJECT -> {
        if (hasCoral) {
          runElevator(state.getElevatorGoal());
          runIf(() -> elevator.atGoal(), () -> runCoralWrist(state.getCoralWristGoal()));
          runIf(
              () -> coralWrist.atGoal() && elevator.atGoal(),
              () -> {
                runCoralWristFlywheel(state.getCoralWristFlywheelGoal());
              });

        } else {
          runCoralWristFlywheel(CoralWrist.flywheelGoal.STOW);
          runCoralWrist(CoralWrist.Goal.CORAL_STOW);
          runIf(
              () -> coralWrist.atGoal(),
              () -> {
                runElevator(Elevator.Goal.STOW);
              });
        }
      }

      case L1_CORAL_EJECT -> {
        if (hasCoral) {
          runElevator(state.getElevatorGoal());
          runIf(() -> elevator.isHomed(), () -> runCoralWrist(state.getCoralWristGoal()));
          runIf(
              () -> coralWrist.atGoal(),
              () -> {
                runCoralWristFlywheel(state.getCoralWristFlywheelGoal());
              });

        } else {
          runCoralWristFlywheel(CoralWrist.flywheelGoal.STOW);
          runIf(
              () -> EqualsUtil.epsilonEquals(coralWrist.getFlywheelVelocityRPM(), 0, 100),
              () -> runCoralWrist(CoralWrist.Goal.STOW));
        }
      }

      case L2_CORAL_EJECT -> {
        if (hasCoral) {
          runElevator(state.getElevatorGoal());
          runIf(() -> elevator.isHomed(), () -> runCoralWrist(state.getCoralWristGoal()));
          runIf(
              () -> coralWrist.atGoal(),
              () -> {
                runCoralWristFlywheel(state.getCoralWristFlywheelGoal());
              });

        } else {
          runCoralWristFlywheel(CoralWrist.flywheelGoal.STOW);
          runIf(
              () -> EqualsUtil.epsilonEquals(coralWrist.getFlywheelVelocityRPM(), 0, 100),
              () -> runCoralWrist(CoralWrist.Goal.STOW));
        }
      }

      case LIMBO_1_ALGE_INTAKE, LIMBO_2_ALGE_INTAKE -> {
        runCoralWrist(state.getCoralWristGoal());
        runElevator(state.getElevatorGoal());
        runIf(
            () -> elevator.atGoal(),
            () ->
                runAlgeManipulator(state.getAlgeManipulatorGoal())); // Sends the AlgeMan to LIMBO 1

        runIf(
            () -> algeManipulator.atGoal(),
            () ->
                runAlgeManipulatorFlywheel(
                    state.getAlgeManipulatorFlywheelGoal())); // Sends the AlgeMan to LIMBO 1
      }
        /* case LIMBO_1_ALGE_INTAKE, LIMBO_2_ALGE_INTAKE -> {
          if (hasAlge) {

            runAlgeManipulator(state.getAlgeManipulatorGoal());
            runIf(() -> algeManipulator.atGoal(), () -> runElevator(state.getElevatorGoal()));
            runIf(
                () -> elevator.atGoal(),
                () -> {
                  runAlgeManipulatorFlywheel(state.getAlgeManipulatorFlywheelGoal());
                  runElevatorFlywheel(state.getElevatorFlywheelGoal());
                });
            runIf(
                () -> !hasAlge,
                () -> {
                  hasAlge = false;
                });

          } else {

          }
        } */
      case CORAL_INTAKE -> {
        if (!hasCoral) {
          runAlgeManipulator(state.getAlgeManipulatorGoal());
          runCoralWrist(state.getCoralWristGoal());
          runIf(() -> coralWrist.atGoal(), () -> runElevator(state.getElevatorGoal()));
          runIf(
              () -> elevator.atGoal(),
              () -> runCoralWristFlywheel(state.getCoralWristFlywheelGoal()));

          runIf(
              () -> hasCoral,
              () -> {
                coralWrist.runFlywheelVelocity(0);
                hasCoral = false;
              });

        } else if (mNotHasGamePiece.update(true, 0.5)) {
          coralWrist.runFlywheelVelocity(0);
        }
        // Timer.delay(1);

      }
        // Assume you have a variable to hold the time when hasAlge becomes true.

      case GROUND_ALGE_INTAKE -> {
        if (!hasAlge) {
          // Reset the timer if we don't have the game piece.
          stowStartTime = -1;

          // Run the normal intake routines.
          runCoralWrist(state.getCoralWristGoal());
          runIf(
              () -> coralWrist.atGoal(), () -> runAlgeManipulator(state.getAlgeManipulatorGoal()));
          runIf(
              () -> algeManipulator.atGoal(),
              () -> {
                runAlgeManipulatorFlywheel(state.getAlgeManipulatorFlywheelGoal());
                runElevatorFlywheel(state.getElevatorFlywheelGoal());
              });
        } else {
          // When hasAlge becomes true, start the timer if it isn't already started.
          if (stowStartTime < 0) {
            stowStartTime = Timer.getFPGATimestamp();
          }
          // Check if 0.5 seconds have passed.
          if (Timer.getFPGATimestamp() - stowStartTime >= 0.55) {
            runAlgeManipulator(AlgeManipulator.Goal.STOW);
            algeManipulator.runFlywheelVelocity(0);
            runElevatorFlywheel(Elevator.flywheelGoal.STOW);
            runAlgeManipulatorFlywheel(AlgeManipulator.flywheelGoal.STOW);
          }
        }
      }

      case NET -> {
        runElevatorFlywheel(Elevator.flywheelGoal.INTAKE);
        runAlgeManipulator(state.getAlgeManipulatorGoal());
        runIf(
            () -> algeManipulator.atGoal(),
            () -> {
              runElevator(state.getElevatorGoal());
            });
        runIf(
            () -> elevator.atGoal(),
            () -> {
              runCoralWrist(state.getCoralWristGoal());
            });

        runIf(
            () -> coralWrist.atGoal(),
            () -> {
              runAlgeManipulatorFlywheel(state.getAlgeManipulatorFlywheelGoal());
              runCoralWristFlywheel(state.getCoralWristFlywheelGoal());
              runElevatorFlywheel(currentState.getValue().getElevatorFlywheelGoal());
              ;
            });
      }
      case CORAL_STOW -> {
        if (hasCoral) {
          runCoralWristFlywheel(CoralWrist.flywheelGoal.STOW);
          runElevator(state.getElevatorGoal());
          runIf(
              () -> elevator.isHomed(),
              () -> {
                runCoralWrist(state.getCoralWristGoal());
              });

        } else {

        }
      }
      case SYSTEM_STOW -> {
        runAlgeManipulatorFlywheel(AlgeManipulator.flywheelGoal.STOW);
        runElevatorFlywheel(Elevator.flywheelGoal.STOW);
        runCoralWristFlywheel(CoralWrist.flywheelGoal.STOW);

        runCoralWrist(state.getCoralWristGoal());

        runIf(() -> coralWrist.atGoal(), () -> runAlgeManipulator(state.getAlgeManipulatorGoal()));
        runIf(
            () -> algeManipulator.atGoal(),
            () -> {
              runElevator(state.getElevatorGoal());
            });
      }
      case PROCESSOR -> {
        if (hasAlge) {
          runCoralWrist(state.getCoralWristGoal());
          runIf(
              () -> coralWrist.atGoal(), () -> runAlgeManipulator(state.getAlgeManipulatorGoal()));
          runIf(
              () -> algeManipulator.atGoal(),
              () -> {
                runAlgeManipulatorFlywheel(
                    currentState.getValue().getAlgeManipulatorFlywheelGoal());
                runElevatorFlywheel(currentState.getValue().getElevatorFlywheelGoal());
              });
        } else {
          runAlgeManipulator(AlgeManipulator.Goal.STOW);
          runAlgeManipulatorFlywheel(AlgeManipulator.flywheelGoal.STOW);
          runElevatorFlywheel(Elevator.flywheelGoal.STOW);
        }
      }

      default -> {
        // Handling an unknown or unhandled state
        System.out.println("Unhandled state: " + currentState);
      }
    }

    // other cases...
  }

  private void setGoal(SuperstructureState goal) {
    currentState = goal;
  }

  public Command runGoal(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal));
  }

  public Command runGoal(Supplier<SuperstructureState> goal) {
    return run(() -> setGoal(goal.get()));
  }

  public void runHomingSequences() {
    algeManipulator.homingSequence();
    coralWrist.runFlywheelVelocity(0);
    algeManipulator.runFlywheelVelocity(0);
    elevator.runFlywheelsVelocity(0, 0);
    runIf(() -> algeManipulator.isHomed(), () -> coralWrist.setAngle(CoralWrist.Goal.STOW));
    runIf(
        () -> coralWrist.isHomed(),
        () -> {
          elevator.homingSequence();
        });
  }

  public void runElevator(Elevator.Goal goal) {
    elevator.setGoal(goal);
  }

  private void runAlgeManipulator(AlgeManipulator.Goal goal) {
    algeManipulator.setAngle(goal);
  }

  private void runCoralWristFlywheel(CoralWrist.flywheelGoal goal) {
    coralWrist.runFlywheelVelocity(goal.getRPM().getAsDouble());
  }

  private void runElevatorFlywheel(Elevator.flywheelGoal goal) {

    elevator.runFlywheelsVelocity(
        goal.getTopRPM().getAsDouble(), goal.getBottomRPM().getAsDouble());
  }

  private void runAlgeManipulatorFlywheel(AlgeManipulator.flywheelGoal goal) {
    algeManipulator.runFlywheelVelocity(goal.getRPM().getAsDouble());
  }

  private void runCoralWrist(CoralWrist.Goal goal) {
    coralWrist.setAngle(goal);
  }

  private boolean getHasAlge() {
    return algeManipulator.getLaserDistance() < 80;
  }

  private boolean getHasLeftCoral() {
    return coralWrist.getLeftLaserDistance() < 60;
  }

  private boolean getHasRightCoral() {
    return coralWrist.getRightLaserDistance() < 60;
  }

  // FUCK YES
  private void runIf(Supplier<Boolean> condition, Runnable action) {
    if (condition.get()) {
      action.run();
    }
  }
}
