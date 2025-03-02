// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.superstructure.algeManipulator.AlgeManipulator;
import frc.robot.subsystems.superstructure.coralWrist.CoralWrist;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;

@Builder(toBuilder = true, access = AccessLevel.PACKAGE)
@Getter
public class SuperstructureStateData {
  private final Elevator.Goal elevatorGoal;
  private final AlgeManipulator.Goal algeManipulatorGoal;
  private final CoralWrist.Goal coralWristGoal;
  private final Elevator.flywheelGoal elevatorFlywheelGoal;
  private final CoralWrist.flywheelGoal coralWristFlywheelGoal;
  private final AlgeManipulator.flywheelGoal algeManipulatorFlywheelGoal;
}
