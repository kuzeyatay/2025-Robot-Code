package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import java.util.Queue;

/** IO implementation for NavX gyro sensor. Maybe used in comp as fallback. */
public class GyroIONavX implements GyroIO {
  // Create an instance of the AHRS (NavX) gyro connected via the MXP SPI port with a specified
  // update rate
  private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) odometryFrequency);

  // Queue to store yaw positions for odometry processing
  private final Queue<Double> yawPositionQueue;

  // Queue to store timestamps corresponding to yaw positions for odometry processing
  private final Queue<Double> yawTimestampQueue;

  /**
   * Constructor for GyroIONavX. Initializes the yaw position and timestamp queues by registering
   * signals with the odometry thread.
   */
  public GyroIONavX() {
    // Retrieve the singleton instance of SparkOdometryThread and create a queue for timestamps
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();

    // Register the NavX angle signal with the odometry thread and store the resulting yaw position
    // queue
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  /**
   * Updates the gyro inputs with the current state of the NavX sensor.
   *
   * @param inputs An instance of GyroIOInputs to be populated with current gyro data.
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Set the connected status based on whether the NavX is connected
    inputs.connected = navX.isConnected();

    // Set the current yaw position by converting the NavX angle from degrees to radians and
    // inverting the direction
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());

    // Set the current yaw velocity by converting the NavX raw gyro Z rate from degrees/sec to
    // radians/sec and
    // inverting the direction
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    // Populate the odometry yaw timestamps array by converting the timestamp queue to a double
    // array
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    // Populate the odometry yaw positions array by converting the yaw position queue to a
    // Rotation2d array with
    // inverted angles
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);

    // Clear the queues after processing to prepare for the next update cycle
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
