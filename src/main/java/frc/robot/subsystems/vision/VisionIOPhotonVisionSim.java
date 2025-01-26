package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * VisionIOPhotonVisionSim is a simulated version of the VisionIOPhotonVision class. It uses
 * PhotonVision's simulation capabilities to generate synthetic vision data based on the robot's
 * known pose and a layout of AprilTags on the field. This allows for testing and validation of
 * vision-based localization algorithms without needing real hardware.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  // A static VisionSystemSim object representing the simulated vision environment.
  // It's static so that multiple simulated cameras can share the same environment.
  private static VisionSystemSim visionSim;

  // A Supplier that provides the current robot pose in simulation.
  private final Supplier<Pose2d> poseSupplier;

  // A PhotonCameraSim object that simulates the behavior of a physical PhotonVision camera.
  private final PhotonCameraSim cameraSim;

  /**
   * Constructs a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera, matching what would be used on a real robot.
   * @param robotToCamera The transform from the robot's coordinate frame to the camera's mounting
   *     position.
   * @param poseSupplier A supplier that returns the current simulated robot pose on the field.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    // Call the superclass constructor to set up PhotonVision as if it were real hardware
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // If the visionSim static object isn't initialized yet, create a new VisionSystemSim
    // and add all the known AprilTags from the layout. This simulates the field environment.
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Create simulation properties for the simulated camera.
    // These properties can define things like camera resolution, field of view, latency, etc.
    var cameraProperties = new SimCameraProperties();

    // Create a PhotonCameraSim object for this camera and add it to the visionSim environment.
    cameraSim = new PhotonCameraSim(camera, cameraProperties);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Overrides the updateInputs method to run the simulation before updating inputs. This method
   * first updates the vision simulation with the current robot pose, which allows the simulator to
   * generate synthetic camera observations. Then it calls the superclass method to parse and store
   * those observations.
   *
   * @param inputs The VisionIOInputs instance to populate with current simulation data.
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update the vision simulation with the robot's current pose.
    // This will produce synthetic observations that represent what a real camera would see.
    visionSim.update(poseSupplier.get());

    // Now call the superclass's updateInputs method to handle the standard logic of parsing camera
    // data.
    super.updateInputs(inputs);
  }
}
