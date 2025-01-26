package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * VisionIO interface provides a hardware abstraction layer for vision systems used by the robot. It
 * defines data structures for target observations, pose observations, and methods for updating
 * inputs.
 */
public interface VisionIO {
  /**
   * VisionIOInputs is a nested static class holding all sensor and status data related to vision
   * processing. Fields are automatically logged for telemetry using the @AutoLog annotation.
   */
  @AutoLog
  public static class VisionIOInputs {
    // Indicates whether the vision camera/system is connected
    public boolean connected = false;

    // Holds the latest angle observation to a simple target (not used for pose estimation)
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());

    // An array of pose observations for the robot, used for pose estimation
    public PoseObservation[] poseObservations = new PoseObservation[0];

    // An array of detected AprilTag IDs, if any
    public int[] tagIds = new int[0];
  }

  /**
   * TargetObservation record represents the angle to a simple target. 'tx' and 'ty' represent
   * horizontal and vertical angle offsets from the camera's centerline.
   */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * PoseObservation record represents a single robot pose sample inferred from vision data. -
   * timestamp: The time at which the observation was recorded. - pose: The estimated robot Pose3d
   * on the field. - ambiguity: A measure of uncertainty in the observation (e.g., from a single
   * tag). - tagCount: The number of AprilTags used to produce this observation. -
   * averageTagDistance: The average distance to the tags used in the observation. - type: The type
   * of observation, which could be from different vision pipelines (MEGATAG_1, MEGATAG_2,
   * PHOTONVISION).
   */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /**
   * PoseObservationType enum indicates the type of vision data source or pipeline used to generate
   * the pose observation. - MEGATAG_1: Observations from a MegaTag 1 pipeline. - MEGATAG_2:
   * Observations from a MegaTag 2 pipeline. - PHOTONVISION: Observations from PhotonVision or
   * similar pipelines.
   */
  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /**
   * Updates the VisionIOInputs with the latest data from the vision hardware/pipeline.
   * Implementations should fetch sensor data, process it, and populate the inputs object.
   *
   * @param inputs The VisionIOInputs instance to populate with current vision data.
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
