package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** Simulation implementation of the GyroIO interface. */
public class GyroIOSim implements GyroIO {
  // Instance of GyroSimulation to simulate gyro readings
  private final GyroSimulation gyroSimulation;

  /**
   * Constructor for GyroIOSim.
   *
   * @param gyroSimulation An instance of GyroSimulation to provide simulated gyro data.
   */
  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  /**
   * Updates the gyro inputs with simulated data.
   *
   * @param inputs An instance of GyroIOInputs to be populated with current gyro data.
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Indicate that the gyro is always connected in simulation
    inputs.connected = true;

    // Set the current yaw position from the simulated gyro reading
    inputs.yawPosition = gyroSimulation.getGyroReading();

    // Set the current yaw velocity by converting simulated angular velocity from degrees/sec to
    // radians/sec
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    // Retrieve simulated odometry yaw timestamps using SparkUtil
    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();

    // Retrieve cached simulated yaw positions from the gyro simulation
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
