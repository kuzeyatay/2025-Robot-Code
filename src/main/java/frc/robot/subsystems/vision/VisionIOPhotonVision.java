package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/**
 * VisionIOPhotonVision is a concrete implementation of the VisionIO interface that integrates with
 * PhotonVision. It retrieves vision data such as target angles and pose estimates from a
 * PhotonVision camera, processes the data, and updates VisionIOInputs accordingly.
 */
public class VisionIOPhotonVision implements VisionIO {
  // The PhotonCamera object used to interact with the PhotonVision pipeline
  protected final PhotonCamera camera;

  // The transform from the robot's reference frame to the camera's mounting position
  protected final Transform3d robotToCamera;

  /**
   * Constructs a new VisionIOPhotonVision instance.
   *
   * @param name The configured name of the PhotonVision camera.
   * @param robotToCamera The Transform3d representing the camera's position and orientation
   *     relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    // Initialize the PhotonCamera with the given name
    camera = new PhotonCamera(name);

    // Store the robot-to-camera transform for later use when calculating robot pose
    this.robotToCamera = robotToCamera;
  }

  /**
   * Updates the VisionIOInputs with the latest data from the PhotonVision camera. Checks camera
   * connection status, retrieves target observations, and calculates pose observations if
   * available.
   *
   * @param inputs The VisionIOInputs instance to populate with current vision data.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update the connection status of the camera
    inputs.connected = camera.isConnected();

    // Prepare data structures to hold the IDs of detected AprilTags and pose observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Retrieve all unread results from the camera
    for (var result : camera.getAllUnreadResults()) {
      // If there are targets, update the latest target observation (tx, ty) from the best target
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        // If no targets are detected, reset the target observation angles to zero
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // If a multi-tag result is available, it can provide a full pose estimation of the robot on
      // the field
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // fieldToCamera: Transform from field coordinates to the camera
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;

        // fieldToRobot: Transform from field to robot, found by removing the camera offset from
        // fieldToCamera
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());

        // Convert this transform into a full 3D pose for the robot
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate the average distance to all detected tags
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          // bestCameraToTarget gives the transform from camera to target; get its norm as a
          // distance
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add all used fiducial (tag) IDs to the tagIds set
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Create a new PoseObservation record with the collected data
        poseObservations.add(
            new PoseObservation(
                // Timestamp: Use the result's timestamp (in seconds) and account for pipeline
                // latency
                result.getTimestampSeconds(),

                // The 3D pose estimate of the robot in field coordinates
                robotPose,

                // The ambiguity from the multi-tag result, indicating confidence in the pose
                multitagResult.estimatedPose.ambiguity,

                // The number of tags used to produce this observation
                multitagResult.fiducialIDsUsed.size(),

                // Average tag distance computed above
                totalTagDistance / result.targets.size(),

                // Indicate that this observation came from PHOTONVISION
                PoseObservationType.PHOTONVISION));
      }
    }

    // Convert the poseObservations list to an array and assign it to inputs
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Convert the set of tag IDs to an array and assign it to inputs
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
