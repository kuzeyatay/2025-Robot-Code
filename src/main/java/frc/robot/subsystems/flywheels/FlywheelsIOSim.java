// Copybottom (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim implements FlywheelsIO {

  private FlywheelSim topSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 1), DCMotor.getNEO(1));
  private FlywheelSim bottomSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.003, 1), DCMotor.getNEO(1));

  private final PIDController topController = new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController bottomController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private Double topSetpointRpm = null;
  private Double bottomSetpointRpm = null;
  private double topFeedforward = 0.0;
  private double bottomFeedforward = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    topSim.update(0.02);
    bottomSim.update(0.02);
    // control to setpoint
    if (topSetpointRpm != null && bottomSetpointRpm != null) {
      runVolts(
          topController.calculate(topSim.getAngularVelocityRPM(), topSetpointRpm) + topFeedforward,
          bottomController.calculate(bottomSim.getAngularVelocityRPM(), bottomSetpointRpm)
              + bottomFeedforward);
    }

    inputs.topPositionRads += Units.radiansToRotations(topSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.topVelocityRpm = topSim.getAngularVelocityRPM();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topSupplyCurrentAmps = topSim.getCurrentDrawAmps();

    inputs.bottomPositionRads +=
        Units.radiansToRotations(bottomSim.getAngularVelocityRadPerSec() * 0.02);
    inputs.bottomVelocityRpm = bottomSim.getAngularVelocityRPM();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomSupplyCurrentAmps = bottomSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double topVolts, double bottomVolts) {
    topSetpointRpm = null;
    bottomSetpointRpm = null;
    topAppliedVolts = MathUtil.clamp(topVolts, -12.0, 12.0);
    bottomAppliedVolts = MathUtil.clamp(bottomVolts, -12.0, 12.0);
    topSim.setInputVoltage(topAppliedVolts);
    bottomSim.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void runVelocity(
      double topRpm, double bottomRpm, double topFeedforward, double bottomFeedforward) {
    topSetpointRpm = topRpm;
    bottomSetpointRpm = bottomRpm;
    this.topFeedforward = topFeedforward;
    this.bottomFeedforward = bottomFeedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    topController.setPID(kP, kI, kD);
    bottomController.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    runVolts(0.0, 0.0);
  }

  @Override
  public void runCharacterizationtop(double input) {
    topSetpointRpm = null;
    bottomSetpointRpm = null;
    runVolts(input, 0.0);
  }

  @Override
  public void runCharacterizationbottom(double input) {
    topSetpointRpm = null;
    bottomSetpointRpm = null;
    runVolts(0.0, input);
  }
}
