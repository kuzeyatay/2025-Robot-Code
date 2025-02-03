package frc.robot.subsystems.flywheels;

import frc.robot.ModeSetter;

public class FlywheelConstants {
  public static final FlywheelConfig flywheelConfig =
      switch (ModeSetter.currentMode) {
        case REAL, REPLAY -> new FlywheelConfig(4, 0, (1.0 / 2.0), 9000.0);
        case SIM -> new FlywheelConfig(0, 0, (1.0 / 2.0), 9000.0);
      };

  public static final Gains gains =
      switch (ModeSetter.currentMode) {
        case REAL, REPLAY -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case SIM -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  public record FlywheelConfig(
      int topID, int bottomID, double reduction, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
