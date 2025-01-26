package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

// Define the GyroIO interface for interacting with the gyro sensor
public interface GyroIO {
  // Define a nested static class GyroIOInputs to hold gyro sensor inputs
  @AutoLog
  public static class GyroIOInputs {
    // Indicates whether the gyro sensor is connected
    public boolean connected = false;

    // Current yaw position of the robot as a Rotation2d object
    public Rotation2d yawPosition = new Rotation2d();

    // Current yaw velocity in radians per second
    public double yawVelocityRadPerSec = 0.0;

    // Array of timestamps for odometry yaw updates
    public double[] odometryYawTimestamps = new double[] {};

    // Array of yaw positions corresponding to the odometry timestamps
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /**
   * Updates the gyro inputs.
   *
   * @param inputs An instance of GyroIOInputs to be updated with current gyro data.
   */
  public default void updateInputs(GyroIOInputs inputs) {}
}
