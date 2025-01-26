package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * VisionConstants class holds constants related to vision-based localization and camera
 * configuration. It includes AprilTag field layouts, camera names, robot-to-camera transforms, and
 * filtering thresholds.
 */
public class VisionConstants {
  // Loads the AprilTag field layout for the default field (e.g., official FRC field)
  // The layout provides the known positions of AprilTags placed around the field.
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // FIXME indicates these camera names should match the configuration on the vision coprocessor or
  // software
  // The camera names here must match what is set on the vision device for correct data association.
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // FIXME Robot-to-camera transforms are placeholders and should be determined accurately.
  // These transforms represent the position and orientation of the camera relative to the robot.
  // The transform is given as a translation (x, y, z) and a rotation (pitch, roll, yaw).
  // Adjust these values to match your robot's actual camera mounting positions.
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds for pose observations:
  // maxAmbiguity: Maximum allowable ambiguity in a single-tag observation.
  // maxZError: Maximum allowable absolute Z coordinate for a pose observation to be considered
  // valid.
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baseline values for vision measurements:
  // These represent the uncertainty in position and orientation estimates when at 1 meter distance
  // and using 1 tag.
  // linearStdDevBaseline: Baseline linear standard deviation in meters.
  // angularStdDevBaseline: Baseline angular standard deviation in radians.
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera:
  // These factors allow you to trust some cameras more than others by scaling their reported
  // uncertainties.
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0 factor
        1.0 // Camera 1 factor
      };

  // Multipliers for MegaTag 2 observations:
  // MegaTag 2 provides more stable linear data but no reliable rotation data, hence infinite
  // angular uncertainty.
  public static double linearStdDevMegatag2Factor = 0.5; // More stable linear estimate
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available, infinite uncertainty
}
