package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// Define the ArmVisualizer class, responsible for visualizing the robot arm's movement
public class ArmVisualizer {
  // Declare a LoggedMechanism2d object to represent the entire mechanism in 2D
  private final LoggedMechanism2d mechanism;
  // Declare a LoggedMechanismLigament2d object to represent the arm's ligament (the moving part)
  private final LoggedMechanismLigament2d arm;
  // Declare a String to hold the key/name for this visualizer instance
  private final String key;

  // Constructor for the ArmVisualizer class, taking a key and a color as parameters
  public ArmVisualizer(String key, Color color) {
    // Assign the provided key to the class's key variable
    this.key = key;
    // Initialize the LoggedMechanism2d with width 3.0, height 3.0, and a white color
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    // Create the root of the mechanism (the pivot point) named "Arm Pivot" at position (1.0, 0.4)
    LoggedMechanismRoot2d root = mechanism.getRoot("Arm Pivot", 1.0, 0.4);
    // Initialize the arm ligament with:
    // - Name: "arm"
    // - Length: ArmConstants.armLength
    // - Thickness: 20.0
    // - Number of points: 6
    // - Color: The provided color converted to 8-bit
    arm =
        new LoggedMechanismLigament2d("arm", ArmConstants.armLength, 20.0, 6, new Color8Bit(color));
    // Attach the arm ligament to the root of the mechanism
    root.append(arm);
  }

  /**
   * Update the arm visualizer with the current arm angle.
   *
   * @param angleRads The current angle of the arm in radians.
   */
  public void update(double angleRads) {
    // Set the angle of the arm ligament based on the provided angle in radians
    arm.setAngle(Rotation2d.fromRadians(angleRads));
    // Record the current state of the mechanism for visualization purposes
    Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism); // FIXME
  }
}
