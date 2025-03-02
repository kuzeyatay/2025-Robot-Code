package frc.robot.subsystems.laserCan;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;

public class LaserCanIOSim implements LaserCanIO {

  private final SimDevice simDevice;
  private final SimEnum m_status;
  private final SimInt m_distancemm;
  private final SimInt m_ambient;
  private final SimBoolean m_islong;
  private final SimEnum m_timingBudget;
  private final SimInt m_roiX, m_roiY, m_roiW, m_roiH;

  private Measurement _measurement;

  /**
   * Create a new LaserCAN sensor.
   *
   * @param can_id The CAN ID for the LaserCAN sensor. This ID is unique, and set in GrappleHook.
   *     Note: one ID should be mapped to only one sensor, or else measurements will conflict.
   */
  public LaserCanIOSim() {
    _measurement =
        new Measurement(
            0,
            0,
            0,
            false,
            TimingBudget.TIMING_BUDGET_20MS.asMilliseconds(),
            new RegionOfInterest(0, 0, 16, 16));

    simDevice = new SimDevice(SimDeviceJNI.createSimDevice("LaserCAN "));
    m_distancemm = simDevice.createInt("distance_mm", Direction.kBidir, _measurement.distance_mm);
    m_ambient = simDevice.createInt("ambient", Direction.kBidir, _measurement.ambient);
    m_islong = simDevice.createBoolean("is_long", Direction.kOutput, _measurement.is_long);
    m_timingBudget =
        simDevice.createEnum(
            "Timing Budget",
            Direction.kOutput,
            new String[] {
              "TIMING_BUDGET_20MS",
              "TIMING_BUDGET_33MS",
              "TIMING_BUDGET_50MS",
              "TIMING_BUDGET_100MS"
            },
            0);
    m_status =
        simDevice.createEnum(
            "status",
            Direction.kBidir,
            new String[] {
              "LASERCAN_STATUS_VALID_MEASUREMENT", "LASERCAN_STATUS_NOISE_ISSUE",
              "LASERCAN_STATUS_WEAK_SIGNAL", "LASERCAN_STATUS_OUT_OF_BOUNDS",
              "LASERCAN_STATUS_WRAPAROUND"
            },
            0);
    m_roiX = simDevice.createInt("ROI X", Direction.kOutput, _measurement.roi.x);
    m_roiY = simDevice.createInt("ROI Y", Direction.kOutput, _measurement.roi.y);
    m_roiW = simDevice.createInt("ROI W", Direction.kOutput, _measurement.roi.w);
    m_roiH = simDevice.createInt("ROI H", Direction.kOutput, _measurement.roi.h);
  }

  public void setMeasurementFullSim(Measurement measurement) {
    _measurement = measurement;
    m_distancemm.set(measurement.distance_mm);
    m_ambient.set(measurement.ambient);
    m_islong.set(measurement.is_long);
    m_timingBudget.set(measurement.budget_ms);
    m_roiX.set(measurement.roi.x);
    m_roiY.set(measurement.roi.y);
    m_roiW.set(measurement.roi.w);
    m_roiH.set(measurement.roi.h);
    m_status.set(measurement.status);
  }

  /**
   * Set the measurement (only the measurement parameters, not including ranging mode, timing
   * budget, or ROI - these will be set automatically) in simulation mode
   */
  public void setMeasurementPartialSim(int status, int distance_mm, int ambient) {
    _measurement.status = status;
    _measurement.distance_mm = distance_mm;
    _measurement.ambient = ambient;
    m_status.set(status);
    m_distancemm.set(distance_mm);
    m_ambient.set(ambient);
  }
}
