package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The Vision subsystem manages data from one or more vision cameras. It updates and processes pose
 * observations, logging relevant data and feeding accepted poses to a consumer that might integrate
 * them into a pose estimation system.
 */
public class Vision extends SubsystemBase {
  // Functional interface for consuming vision-based robot pose measurements
  private final VisionConsumer consumer;

  // Array of VisionIO objects for interacting with multiple cameras
  private final VisionIO[] io;

  // Array of inputs structs for each camera, automatically logged
  private final VisionIOInputsAutoLogged[] inputs;

  // Array of alerts to display warnings if cameras are disconnected
  private final Alert[] disconnectedAlerts;

  /**
   * Constructs the Vision subsystem.
   *
   * @param consumer A VisionConsumer that accepts pose measurements for further processing.
   * @param io One or more VisionIO implementations that interact with camera hardware or
   *     simulation.
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize the inputs array, one for each camera
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize an alert for each camera to show if it becomes disconnected
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle (horizontal offset angle) to the best target from a specified camera. This
   * angle can be used for servoing or adjusting orientation based on a vision target.
   *
   * @param cameraIndex The index of the camera to query.
   * @return A Rotation2d representing the horizontal angle to the target.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /**
   * The periodic method is called regularly. It updates camera inputs, processes observations,
   * applies filters to reject unrealistic poses, and provides accepted poses to the consumer. It
   * also logs relevant data for debugging and analysis.
   */
  @Override
  public void periodic() {
    // Update inputs from each camera and log them
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize lists to store tag and robot poses for logging
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Process each camera individually
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update the disconnected alert based on camera connection status
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Lists for per-camera logging
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add detected AprilTag poses to the logging lists (if tag IDs are known)
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Process each pose observation from the camera
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Determine if this pose observation should be rejected
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag for a valid pose
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > maxAmbiguity) // Single-tag observation must have low ambiguity
                || Math.abs(observation.pose().getZ()) > maxZError // Z coordinate must be realistic
                // Pose must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add the pose to the logging lists
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // If the pose is rejected, skip further processing
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations for the vision measurement based on observation
        // characteristics
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        // Adjust standard deviations if using MEGATAG_2 observations
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Adjust standard deviations based on camera-specific factors if defined
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Accept and send the vision observation to the consumer
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Record camera-specific data for logging and debugging
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

      // Add camera-specific poses to the summary lists
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data for all cameras combined
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  /**
   * Functional interface for accepting vision-based robot pose measurements. Implementations of
   * VisionConsumer may integrate these measurements into pose estimators or logging systems.
   */
  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Accepts a pose measurement of the robot in field coordinates.
     *
     * @param visionRobotPoseMeters The measured Pose2d of the robot on the field.
     * @param timestampSeconds The timestamp of the measurement, in seconds.
     * @param visionMeasurementStdDevs A Matrix containing the standard deviations for the
     *     measurement in x, y, and rotation.
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
