package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
  @AutoLog
  class FlywheelsIOInputs {
    public boolean topMotorConnected = true;
    public boolean bottomMotorConnected = true;

    public double topPositionRads = 0.0;
    public double topVelocityRpm = 0.0;
    public double topAppliedVolts = 0.0;
    public double topSupplyCurrentAmps = 0.0;
    public double topTorqueCurrentAmps = 0.0;
    public double topTempCelsius = 0.0;

    public double bottomPositionRads = 0.0;
    public double bottomVelocityRpm = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double bottomSupplyCurrentAmps = 0.0;
    public double bottomTorqueCurrentAmps = 0.0;
    public double bottomTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double topVolts, double bottomVolts) {}

  /** Stop both flywheels */
  default void stop() {}

  /** Run top and bottom flywheels at velocity in rpm */
  default void runVelocity(
      double topRpm, double bottomRpm, double topFeedforward, double bottomFeedforward) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run top flywheels at voltage */
  default void runCharacterizationtop(double input) {}

  /** Run bottom flywheels at voltage */
  default void runCharacterizationbottom(double input) {}
}
