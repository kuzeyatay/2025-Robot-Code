package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 0.1;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // seconds
  private static final double FF_RAMP_RATE = 0.1; // volts/second
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // rad/sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // rad/sec^2

  private DriveCommands() {} // Prevent instantiation

  //////////////////////////////////////////////////////////////////////////////
  // Command Factories
  //////////////////////////////////////////////////////////////////////////////

  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.runVelocity(speeds);
        },
        drive);
  }

  public static Command aligningDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          omega = Math.copySign(omega * omega, omega);
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omegaSupplier.getAsDouble());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(speeds);
        },
        drive);
  }

  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    return Commands.run(
            () -> {
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.runVelocity(speeds);
            },
            drive)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),
        Commands.run(() -> drive.runCharacterization(0.0), drive).withTimeout(FF_START_DELAY),
        Commands.runOnce(timer::restart),
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> limiter.reset(0.0)),
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),
        Commands.sequence(
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),
            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;
                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  public static Command angleRotate(
      Drive drive, DoubleSupplier ySupplier, DoubleSupplier xSupplier, Vision vision, boolean tv) {
    return Commands.run(
        () -> {
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double tx = vision.getTargetX(0).getRadians();
          double omega = -MathUtil.applyDeadband(tx, DEADBAND);
          linearMagnitude = linearMagnitude * linearMagnitude;
          double xGamepad = xSupplier.getAsDouble();
          double error = (omega + xGamepad * 5) / 30; // normalized error (-1 to 1)
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
          double finalRotation = error * 2.5;
          finalRotation = MathUtil.applyDeadband(finalRotation, 0.02);
          if (tv) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    finalRotation,
                    drive.getRotation()));
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    0,
                    drive.getRotation()));
          }
        },
        drive);
  }

  public static Command alignDrive(
      Drive drive, Vision vision, DoubleSupplier ySupplier, DoubleSupplier xSupplier) {
    final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
    return Commands.run(
        () -> {
          boolean fieldRelative = true;
          double xSpeed =
              -xspeedLimiter.calculate(MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.02))
                  * DriveConstants.maxSpeedMetersPerSec;
          double ySpeed =
              -yspeedLimiter.calculate(MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.02))
                  * DriveConstants.maxSpeedMetersPerSec;
          double rot =
              -rotLimiter.calculate(MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.02))
                  * DriveConstants.maxAngularSpeedRadiansPerSec;
          // Override with vision-based control
          rot = limelightAimProportional(vision);
          xSpeed = limelightRangeProportional(vision);
          fieldRelative = true;
          ChassisSpeeds speeds;
          if (fieldRelative) {
            speeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, drive.getRotation());
          } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
          }
          drive.runVelocity(speeds);
        },
        drive);
  }

  public static Command alignWhileDrivingCommand(
      Supplier<Double> xSpeed,
      Supplier<Double> ySpeed,
      Supplier<Translation2d> target,
      Drive drive) {
    PIDController pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(0.1);
    pid.enableContinuousInput(-180, 180);
    return new DeferredCommand(
        () ->
            new RepeatCommand(
                new FunctionalCommand(
                    () -> {
                      /* Initialization */
                    },
                    () -> {
                      Translation2d currentTranslation = drive.getPose().getTranslation();
                      Translation2d targetVector = currentTranslation.minus(target.get());
                      Rotation2d targetAngle = targetVector.getAngle();
                      double newSpeed;
                      double linearMagnitude =
                          MathUtil.applyDeadband(Math.hypot(xSpeed.get(), ySpeed.get()), DEADBAND);
                      Rotation2d linearDirection = new Rotation2d(xSpeed.get(), ySpeed.get());
                      linearMagnitude = linearMagnitude * linearMagnitude;
                      Translation2d linearVelocity =
                          new Pose2d(new Translation2d(), linearDirection)
                              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                              .getTranslation();
                      if (DriverStation.getAlliance().get() == Alliance.Red) {
                        newSpeed =
                            pid.calculate(
                                drive.getRotation().getDegrees(), targetAngle.getDegrees());
                      } else {
                        newSpeed =
                            pid.calculate(
                                drive.getRotation().getDegrees() + 180, targetAngle.getDegrees());
                      }
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                              linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                              newSpeed * drive.getMaxAngularSpeedRadPerSec(),
                              drive.getRotation()));
                    },
                    interrupted -> pid.close(),
                    () -> pid.atSetpoint(),
                    drive)),
        Set.of(drive));
  }

  public static final double TX_TOLERANCE = 1.0; // degrees tolerance for horizontal error
  public static final double TY_TOLERANCE = 1.0; // degrees tolerance for vertical error

  public static Command driveToTagPose(Drive drive, Pose2d tagPose) {
    return Commands.run(
        () -> {
          // Get the current robot pose (assumes your Drive subsystem has a getPose() method)
          Pose2d currentPose = drive.getPose();

          // Compute the error pose.
          // This returns the tag pose expressed in the robot's coordinate frame.
          Pose2d errorPose = tagPose.relativeTo(currentPose);

          // Extract errors:
          // errorPose.getTranslation().getX() is the forward (or backward) error.
          // errorPose.getTranslation().getY() is the lateral error (sideways).
          // errorPose.getRotation().getRadians() is the heading error.
          double errorX = errorPose.getTranslation().getX();
          double errorY = errorPose.getTranslation().getY();
          double errorTheta = errorPose.getRotation().getRadians();

          // Define proportional gains for translation and rotation.
          double kPLinear = 0.5; // Adjust as needed for your robot's speed and dynamics.
          double kPAngular = 1.0; // Adjust for how quickly you want to rotate.

          // Compute the commanded speeds based on error.
          double xSpeed = errorX * kPLinear; // forward/backward speed.
          double ySpeed = errorY * kPLinear; // lateral (side-to-side) speed.
          double omega = errorTheta * kPAngular; // angular (rotational) speed.

          // Create a chassis speeds object.
          // This command assumes robot-relative speeds.
          ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

          // Command the drivetrain to run at these speeds.
          drive.runVelocity(speeds);
        },
        drive);
  }

  //////////////////////////////////////////////////////////////////////////////
  // Helper Methods and State
  //////////////////////////////////////////////////////////////////////////////

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  private static double limelightAimProportional(Vision vision) {
    double kP = 1;
    double targetingAngularVelocity = vision.getTargetX(0).getRadians() * kP;
    targetingAngularVelocity *= DriveConstants.maxAngularSpeedRadiansPerSec;
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }

  private static double limelightRangeProportional(Vision vision) {
    double kP = 1;
    double targetingForwardSpeed = vision.getTargetX(0).getRadians() * kP;
    targetingForwardSpeed *= DriveConstants.maxSpeedMetersPerSec;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
}
