package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;

public class AlignCommand extends Command {
  private final Drive m_swerve;
  private final CommandPS5Controller m_controller;
  private final Vision vision;

  // Slew rate limiters to smooth joystick inputs; 3 units per second.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public AlignCommand(Drive swerve, CommandPS5Controller controller, Vision vision) {
    m_swerve = swerve;
    this.vision = vision;
    m_controller = controller;
    // Declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  @Override
  public void execute() {
    // Assume field-relative driving by default.
    boolean fieldRelative = true;

    // Get the x speed. Invert because the controller returns negative when pushed forward.
    double xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * DriveConstants.maxSpeedMetersPerSec;

    // Get the y speed (strafe speed). Invert because we want positive to the left.
    double ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * DriveConstants.maxSpeedMetersPerSec;

    // Get the angular rotation rate. Invert to ensure correct turning direction.
    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * DriveConstants.maxAngularSpeedRadiansPerSec;

    // When the A-button is held, use Limelight values for aiming and ranging.
    if (m_controller.getL2Axis() > 0.05) {
      rot = limelightAimProportional();
      xSpeed = limelightRangeProportional();
      // Disable field-relative driving when using Limelight targeting.
      fieldRelative = false;
    }

    // Create a ChassisSpeeds object. If driving field-relative, convert speeds accordingly.
    ChassisSpeeds speeds;
    if (fieldRelative) {
      // Assume m_swerve.getRotation() returns the current robot heading as a Rotation2d.
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_swerve.getRotation());
    } else {
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    // Run the swerve using your runVelocity method.
    m_swerve.runVelocity(speeds);
  }

  /**
   * Uses a proportional control loop based on Limelight's "tx" value to aim.
   *
   * @return the angular velocity (in radians per second) for turning.
   */
  private double limelightAimProportional() {
    // Proportional constant (hand-tuned).
    double kP = 0.035;
    // Calculate angular velocity proportional to the Limelight horizontal offset.
    double targetingAngularVelocity = vision.getTargetX(0).getRadians() * kP;
    targetingAngularVelocity *= DriveConstants.maxAngularSpeedRadiansPerSec;
    // Invert since tx is positive when the target is to the right.
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }

  /**
   * Uses a proportional control loop based on Limelight's "ty" value for ranging.
   *
   * @return the forward speed for moving toward the target.
   */
  private double limelightRangeProportional() {
    double kP = 0.1;
    double targetingForwardSpeed = vision.getTargetX(0).getRadians() * kP;
    targetingForwardSpeed *= DriveConstants.maxSpeedMetersPerSec;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  @Override
  public boolean isFinished() {
    // This command is intended to run continuously.
    return false;
  }
}
