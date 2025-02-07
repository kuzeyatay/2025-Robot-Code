package frc.robot.subsystems.laserCan;

import org.littletonrobotics.junction.AutoLog;

public interface LaserCanIO {

  // AutoLog annotation indicates that the laserCanIOInputs class fields will be automatically
  // logged
  @AutoLog
  public class LaserCanIOInputs {

    public double m_distance;
  }
  /**
   * Updates the set of loggable inputs with the current state of the laserCan hardware.
   *
   * @param inputs An instance of laserCanIOInputs to populate with sensor and status data.
   */
  public default void updateInputs(LaserCanIOInputs inputs) {}
}
