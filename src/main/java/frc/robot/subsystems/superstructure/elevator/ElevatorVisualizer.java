package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/**
 * ElevatorVisualizer class provides a 2D visualization of the elevator subsystem for debugging and
 * telemetry. It uses the LoggedMechanism2d framework to render a ligament representing the
 * elevator.
 */
public class ElevatorVisualizer {
  // A LoggedMechanism2d instance for rendering the elevator visualization
  private final LoggedMechanism2d mechanism;

  // A LoggedMechanismLigament2d representing the elevator's vertical segment
  private final LoggedMechanismLigament2d elevator;

  // A key used for logging and differentiating multiple elevator visualizers if needed
  private final String key;

  /**
   * Constructs an ElevatorVisualizer.
   *
   * @param key A unique string identifier for this visualizer instance.
   * @param color The color used to represent the elevator ligament in the visualization.
   */
  public ElevatorVisualizer(String key, Color color) {
    this.key = key;
    // Creates a LoggedMechanism2d with a specified width (20) and height (50), and a white
    // background color
    mechanism = new LoggedMechanism2d(20, 50, new Color8Bit(Color.kWhite));

    // Creates a root point for the mechanism visualization at (10.0, 0.0)
    LoggedMechanismRoot2d root = mechanism.getRoot("Elevator Root", 10.0, 0.0);

    // Creates a ligament (vertical segment) representing the elevator's height
    // The ligament has a name "Elevator", a maximum length from ElevatorConstants,
    // initial angle of 0.0 degrees, thickness of 10, and the specified color.
    elevator =
        new LoggedMechanismLigament2d(
            "Elevator", ElevatorConstants.kMaxElevatorHeightMeters, 0.0, 10, new Color8Bit(color));

    // Appends the elevator ligament to the root, integrating it into the visualization
    root.append(elevator);
  }

  /**
   * Updates the elevator visualizer with the current elevator height. Adjusts the length of the
   * ligament to reflect the elevator's position in meters.
   *
   * @param height The current elevator height in meters.
   */
  public void update(double height) {
    // Sets the ligament length to the current elevator height
    elevator.setLength(height);

    // Records the updated mechanism2d state to the logger under "Elevator/Mechanism2d/<key>"
    // FIXME: The comment indicates this line may need revisiting for output adjustments
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);
  }
}
