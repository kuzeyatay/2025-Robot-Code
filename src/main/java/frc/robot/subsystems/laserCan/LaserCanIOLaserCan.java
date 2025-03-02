package frc.robot.subsystems.laserCan;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class LaserCanIOLaserCan implements LaserCanIO {
  // Creates a laserCan sensor
  private LaserCan lasercan;

  public LaserCanIOLaserCan(int id) {
    lasercan = new LaserCan(id);
    CanBridge.runWebsocketInBackground(7171);
    try {
      lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
      // lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(8,8,16,16)); // prev
      lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " + e);
    }
  }

  @Override
  public void updateInputs(LaserCanIOInputs inputs) {

    LaserCan.Measurement m_dis = lasercan.getMeasurement();
    if (m_dis != null) {
      inputs.m_distance = m_dis.distance_mm;
    } else {
      inputs.m_distance = 100;
    }
  }
}
