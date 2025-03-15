package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulator;
import frc.robot.subsystems.superstructure.coralWrist.CoralWrist;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  STOW(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.STOW)
          .coralWristGoal(CoralWrist.Goal.STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),
  CORAL_STOW(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.STOW)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),

  L1_CORAL_EJECT(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.L1)
          .coralWristGoal(CoralWrist.Goal.L1)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build()),

  GROUND_ALGE_INTAKE(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.GROUND_INTAKE)
          .elevatorGoal(Elevator.Goal.STOW)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.INTAKE)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.INTAKE)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),

  SYSTEM_STOW(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.STOW)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),
  LIMBO_1_ALGE_INTAKE(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.LIMBO_1)
          .elevatorGoal(Elevator.Goal.LIMBO_1)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.INTAKE)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.INTAKE)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),
  L2_CORAL_EJECT_AUTO(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.L2)
          .coralWristGoal(CoralWrist.Goal.L2)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build()),
  L2_CORAL_EJECT(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.L2)
          .coralWristGoal(CoralWrist.Goal.L2)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build()),
  L3_CORAL_EJECT(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.L3)
          .coralWristGoal(CoralWrist.Goal.L3)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build()),
  LIMBO_2_ALGE_INTAKE(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.LIMBO_2)
          .elevatorGoal(Elevator.Goal.LIMBO_2)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.INTAKE)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.INTAKE)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),
  L4_CORAL_EJECT(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.STOW)
          .elevatorGoal(Elevator.Goal.L4)
          .coralWristGoal(CoralWrist.Goal.L4)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build()),

  CORAL_INTAKE(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.BACKWARDS_STOW)
          .elevatorGoal(Elevator.Goal.CHUTE)
          .coralWristGoal(CoralWrist.Goal.CHUTE)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.STOW)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.STOW)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.INTAKE)
          .build()),
  PROCESSOR(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.PROCESSOR)
          .elevatorGoal(Elevator.Goal.EJECT)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.PROCESSOR)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.EJECT)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.STOW)
          .build()),
  NET(
      SuperstructureStateData.builder()
          .algeManipulatorGoal(AlgeManipulator.Goal.NET)
          .elevatorGoal(Elevator.Goal.NET)
          .coralWristGoal(CoralWrist.Goal.CORAL_STOW)
          .algeManipulatorFlywheelGoal(AlgeManipulator.flywheelGoal.NET)
          .elevatorFlywheelGoal(Elevator.flywheelGoal.NET)
          .coralWristFlywheelGoal(CoralWrist.flywheelGoal.EJECT)
          .build());

  private final SuperstructureStateData value;
}
