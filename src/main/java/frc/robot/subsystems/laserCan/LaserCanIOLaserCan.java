package frc.robot.subsystems.laserCan;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class LaserCanIOLaserCan implements LaserCanIO {
  // Creates a laserCan sensor
  private LaserCan lasercan = new LaserCan(18); // FIX ME!!!!!! CHANGE TO CORRECT CAN-ID

  public LaserCanIOLaserCan() {
    CanBridge.runWebsocketInBackground(7171);
    try {
      lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
      lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " + e);
    }
  }

  @Override
  public void updateInputs(LaserCanIOInputs inputs) {
    // THERE HAS TO BE AN EASIER WAY TO THIS. I improvised
    LaserCan.Measurement m_dis = lasercan.getMeasurement();
    String m = "" + m_dis;
    Double m_distanc = Double.valueOf(m);

    inputs.m_distance = m_distanc;
  }
}
