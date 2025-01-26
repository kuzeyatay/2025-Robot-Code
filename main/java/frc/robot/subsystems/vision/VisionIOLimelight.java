package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * VisionIOLimelight is a concrete implementation of the VisionIO interface for a Limelight camera.
 * It interacts with the NetworkTables interface provided by Limelight, retrieving target angles,
 * pose estimates, and other relevant vision data, then populating the VisionIOInputs structure for
 * use by the Vision subsystem.
 */
public class VisionIOLimelight implements VisionIO {
  // Supplier for the current estimated robot rotation, used to help MegaTag 2 pose estimation
  private final Supplier<Rotation2d> rotationSupplier;

  // Publisher to send the robot's orientation to the Limelight for MegaTag 2 usage
  private final DoubleArrayPublisher orientationPublisher;

  // Subscribers for latency, horizontal/vertical angles (tx, ty), and MegaTag pipeline outputs
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * Constructs a new VisionIOLimelight.
   *
   * @param name The NetworkTables name of the Limelight camera.
   * @param rotationSupplier A supplier providing the current estimated robot rotation, useful for
   *     MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    // Retrieve the NetworkTable for the named Limelight
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;

    // Publisher for sending robot orientation to Limelight for MegaTag 2 alignment
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

    // Subscribers for retrieving vision data from the Limelight
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0); // Pipeline latency
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0); // Horizontal angle to target
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0); // Vertical angle to target
    megatag1Subscriber =
        table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {}); // MegaTag 1 data
    megatag2Subscriber =
        table
            .getDoubleArrayTopic("botpose_orb_wpiblue")
            .subscribe(new double[] {}); // MegaTag 2 data
  }

  /**
   * Updates the VisionIOInputs with the latest data from the Limelight. Reads angles (tx, ty),
   * connection status, and MegaTag pipeline outputs from NetworkTables.
   *
   * @param inputs The VisionIOInputs instance to populate with current vision data.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Determine if the camera is connected by checking the time since last latency update
    // If no update in 250ms, consider the camera disconnected
    inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

    // Update the latest target observation with horizontal (tx) and vertical (ty) angles
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Send current robot orientation to Limelight for MegaTag 2 pose estimation
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    // Flush network updates to ensure Limelight receives orientation data promptly
    NetworkTableInstance.getDefault().flush();

    // Use sets and lists to handle observed tag IDs and pose observations
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Process MegaTag 1 observations
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      // Extract tag IDs from the sample data. Tag IDs appear after certain indices in the array.
      for (int i = 10; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // Create a PoseObservation record from the raw data
      poseObservations.add(
          new PoseObservation(
              // Timestamp calculated by rawSample.timestamp (NT timestamp) minus pipeline latency
              rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,
              parsePose(rawSample.value), // Parse pose from raw LL array
              rawSample.value.length >= 17 ? rawSample.value[16] : 0.0, // Ambiguity (if available)
              (int) rawSample.value[8], // Tag count
              rawSample.value[10], // Average tag distance
              PoseObservationType.MEGATAG_1)); // Observation type
    }

    // Process MegaTag 2 observations
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;

      // Extract tag IDs from the sample data for MegaTag 2
      for (int i = 10; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // Create a PoseObservation for MegaTag 2
      poseObservations.add(
          new PoseObservation(
              // Timestamp calculation similar to above
              rawSample.timestamp * 1.0e-9 - rawSample.value[7] * 1.0e-3,
              parsePose(rawSample.value), // Parse pose
              0.0, // Ambiguity is zeroed for MegaTag 2
              (int) rawSample.value[8], // Tag count
              rawSample.value[10], // Average tag distance
              PoseObservationType.MEGATAG_2));
    }

    // Convert poseObservations list to an array and store in inputs
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Convert tagIds set to an array and store in inputs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /**
   * Parses a Pose3d from a Limelight botpose array. The array is expected to contain [X, Y, Z,
   * roll, pitch, yaw, ...] in the first 6 indices.
   *
   * @param rawLLArray The raw array of double values representing the pose from Limelight.
   * @return A Pose3d constructed from the array values.
   */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0], // X position in meters
        rawLLArray[1], // Y position in meters
        rawLLArray[2], // Z position in meters
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]), // Roll in radians
            Units.degreesToRadians(rawLLArray[4]), // Pitch in radians
            Units.degreesToRadians(rawLLArray[5])) // Yaw in radians
        );
  }
}
