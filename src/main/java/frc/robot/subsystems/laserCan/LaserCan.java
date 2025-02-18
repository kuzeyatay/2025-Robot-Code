package frc.robot.subsystems.laserCan;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class LaserCan extends SubsystemBase {

  public int index;
  // IO interface for controlling and reading from the flywheel hardware
  private final LaserCanIO io;
  // Structure holding inputs from the FlywheelIO, automatically logged for telemetry
  private final LaserCanIOInputsAutoLogged inputs = new LaserCanIOInputsAutoLogged();

  /**
   * Constructs a new laserCAN subsystem.
   *
   * @param io The laserCanIO implementation to interact with the laserCan hardware.
   */
  public LaserCan(LaserCanIO io) {
    this.io = io;
  }

  /**
   * This method is called periodically by the scheduler. Updates the flywheel inputs and logs them.
   */
  @Override
  public void periodic() {
    // Update inputs from the hardware implementation
    io.updateInputs(inputs);
    // Process and log these inputs for debugging and telemetry
    Logger.processInputs(" laserCan", inputs);
  }

  @Getter public Double laserDistance = inputs.m_distance;
}
