package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * subsystem management. Based on {@link SubsystemBase} from WPILib, we added on-disable/on-enable
 * function calls as well as precise dt calculations
 */
public abstract class GenericSubsystem extends SubsystemBase {
  public static final List<GenericSubsystem> instances = new ArrayList<>();
  private double previousUpdateTimeStamp = 0;

  public static void register(GenericSubsystem instance) {
    instances.add(instance);
  }

  public static void cancelRegister(GenericSubsystem instance) {
    instances.remove(instance);
  }

  private static boolean wasEnabled = false;

  public static void checkForOnDisableAndEnable() {
    // periodic() is called from CommandScheduler, we only need to check for enable/disable
    if (DriverStation.isEnabled() && (!wasEnabled)) enableAlllSubsystems();
    else if (DriverStation.isDisabled() && (wasEnabled)) disableAllSubsystems();
    wasEnabled = DriverStation.isEnabled();
  }

  public static void enableAlllSubsystems() {
    for (GenericSubsystem instance : instances) instance.onEnable();
  }

  public static void disableAllSubsystems() {
    for (GenericSubsystem instance : instances) instance.onDisable();
  }

  public GenericSubsystem(String name) {
    super(name);
    register(this);
  }

  public void onEnable() {}

  public void onDisable() {}

  public abstract void periodic(double dt, boolean enabled);

  @Override
  public void periodic() {
    final long t0 = System.nanoTime();
    periodic(getDt(), DriverStation.isEnabled());
    final double cpuTimeMS = (System.nanoTime() - t0) / 1_000_000.0;
    Logger.recordOutput(LogPaths.SYSTEM_PERFORMANCE_PATH + getName() + "-CPUTimeMS", cpuTimeMS);
  }

  private double getDt() {
    if (previousUpdateTimeStamp == 0) {
      previousUpdateTimeStamp = TimeUtils.getLogTimeSeconds();
      return Robot.defaultPeriodSecs;
    }
    final double dt = TimeUtils.getLogTimeSeconds() - previousUpdateTimeStamp;
    previousUpdateTimeStamp = Logger.getTimestamp();
    return dt;
  }
}
