package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** IO implementation for Pigeon 2 gyro sensor. */
public class GyroIOPigeon2 implements GyroIO {
  // Create an instance of the Pigeon2 gyro connected via its CAN ID
  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);

  // StatusSignal to retrieve the current yaw angle from the Pigeon2
  private final StatusSignal<Angle> yaw = pigeon.getYaw();

  // Queue to store yaw positions for odometry processing
  private final Queue<Double> yawPositionQueue;

  // Queue to store timestamps corresponding to yaw positions for odometry processing
  private final Queue<Double> yawTimestampQueue;

  // StatusSignal to retrieve the current yaw angular velocity from the Pigeon2
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  /**
   * Constructor for GyroIOPigeon2. Initializes the Pigeon2 gyro and registers signals with the
   * odometry thread.
   */
  public GyroIOPigeon2() {
    // Apply the default Pigeon2 configuration
    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    // Reset the yaw angle to zero
    pigeon.getConfigurator().setYaw(0.0);

    // Set the update frequency for yaw and yaw velocity signals
    yaw.setUpdateFrequency(odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);

    // Optimize bus utilization for the Pigeon2 gyro
    pigeon.optimizeBusUtilization();

    // Retrieve the singleton instance of SparkOdometryThread and create a queue for timestamps
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();

    // Register the yaw signal with the odometry thread and store the resulting yaw position queue
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
  }

  /**
   * Updates the gyro inputs with the current state of the Pigeon2 sensor.
   *
   * @param inputs An instance of GyroIOInputs to be populated with current gyro data.
   */
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Refresh all status signals and check if the gyro is connected
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    // Set the current yaw position by converting the Pigeon2 yaw angle from degrees to radians
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble() + 90);

    // Set the current yaw velocity by converting the Pigeon2 yaw angular velocity from degrees/sec
    // to radians/sec
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    // Populate the odometry yaw timestamps array by converting the timestamp queue to a double
    // array
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    // Populate the odometry yaw positions array by converting the yaw position queue to a
    // Rotation2d array
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

    // Clear the queues after processing to prepare for the next update cycle
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
