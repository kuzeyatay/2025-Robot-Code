/* FIXME READ FOR SETUP INSTRUCTIONS:
Manually rotate the turning position each module such that the position in AdvantageScope (/Drive/Module.../TurnPosition) is increasing.
The module should be rotating counter-clockwise as viewed from above the robot. Verify that the units visible in AdvantageScope
(radians) match the physical motion of the module. If necessary, change the value of turnInverted or turnEncoderInverted.

Manually rotate each module to align it directly forwards. Verify using AdvantageScope that the drive position increases when the
wheel rotates such that the robot would be propelled forwards. We recommend pressing a straight object such as aluminum tubing against
the pairs of left and right modules to ensure accurate alignment.

Record the value of /Drive/Module.../TurnPosition for each aligned module. Update the value of ...ZeroRotation for each module to
new Rotation2d(<insert value>). */

/* FIXME Drive/Turn PID Tuning:

The project includes default gains for the drive velocity PID controllers and turn position PID controllers,
which can be found in the "Drive PID configuration" and "Turn PID configuration" sections. These gains should be tuned for each robot.

Use AdvantageScope to plot the measured and setpoint values while tuning. Measured values are published to the
/RealOutputs/SwerveStates/Measured field and setpoint values are published to the /RealOutputs/SwerveStates/SetpointsOptimized field. */

/* FIXME Feedforward Characterization:

The project includes default feedforward gains for velocity control of the drive motors (kS and kV).

The project includes a simple feedforward routine that can be used to quicly measure the drive kS and kV values without requiring SysId:

Tune turning PID gains as described here:

  1.Place the robot in an open space.

  2.Select the "Drive Simple FF Characterization" auto routine.

  3.Enable the robot in autonomous. The robot will slowly accelerate forwards, similar to a SysId quasistic test.

  4.Disable the robot after at least ~5-10 seconds.

  5.Check the console output for the measured kS and kV values, and copy them to the driveKs and driveKv constants  */

/* FIXME Wheel Radius Characterization

The effective wheel radius of a robot tends to change over time as wheels are worn down, swapped, or compress into the carpet.
This can have significant impacts on odometry accuracy. Regularly recharacterize the wheel radius to combat these issues.

the project includes an automated wheel radius characterization routine, which only requires enough space for the robot
to rotate in place.

  1.Place the robot on carpet. Characterizing on a hard floor may produce errors in the measurement, as the robot's effective wheel radius is affected by carpet compression.

  2.Select the "Drive Wheel Radius Characterization" auto routine.

  3.Enable the robot is autonomous. The robot will slowly rotate in place.

  4.Disable the robot after at least one full rotation.

  5.Check the console output for the measured wheel radius, and copy the value to wheelRadiusMeters */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

// Define the DriveConstants class to hold all constants related to the drive subsystem
public class DriveConstants {
  // Maximum linear speed in meters per second
  public static final double maxSpeedMetersPerSec = 4; // max to 4.8

  // Maximum linear acceleration in meters per second squared
  public static final double maxAccelerationMetersPerSecSq = Units.feetToMeters(70.0); // FIXME

  // Maximum angular speed in radians per second
  public static final double maxAngularSpeedRadiansPerSec = 6.5;

  // Maximum angular acceleration in radians per second squared
  public static final double maxAngularAccelerationRadPerSecSq = 4.0;

  // Odometry update frequency in Hz
  public static final double odometryFrequency =
      70; // FIXME max value is 100hz not recommended to cross it.

  // Track width of the robot in meters
  public static final double trackWidth = 0.603; // FIXME Measure it.

  // Wheelbase of the robot in meters
  public static final double wheelBase = 0.603;

  // Drive base radius calculated using track width and wheelbase
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  // Translations for each swerve module relative to the robot center
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Bumper dimensions in the X direction
  public static final Distance bumperWidthX = Meters.of(Units.inchesToMeters(36)); // FIXME

  // Bumper dimensions in the Y direction
  public static final Distance bumperWidthY = Meters.of(Units.inchesToMeters(36)); // FIXME

  // Zeroed rotation values for each swerve module in radians
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // CAN IDs for the Pigeon gyro sensor
  public static final int pigeonCanId = 20;

  // CAN IDs for drive motors
  public static final int frontLeftDriveCanId = 3;
  public static final int backLeftDriveCanId = 5;
  public static final int frontRightDriveCanId = 1;
  public static final int backRightDriveCanId = 7;

  // CAN IDs for turn motors
  public static final int frontLeftTurnCanId = 4;
  public static final int backLeftTurnCanId = 6;
  public static final int frontRightTurnCanId = 2;
  public static final int backRightTurnCanId = 8;

  // Drive motor current limit in Amperes
  public static final int driveMotorCurrentLimit = 45;

  // Wheel radius in meters
  public static final double wheelRadiusMeters =
      Units.inchesToMeters(2); // FIXME see above for instructions. 2 is probably wrong

  // Total gear reduction for the drive motors
  public static final double driveMotorReduction = ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0));

  // Drive gearbox configuration using a NEO motor with specified reduction
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder position factor (rotor rotations to wheel radians)
  public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;

  // Drive encoder velocity factor (rotor RPM to wheel rad/s)
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction;

  // PID constants for drive control
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;

  // Simulation PID constants for drive
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor inversion
  public static final boolean turnInverted = true;

  // Turn motor current limit in Amperes
  public static final int turnMotorCurrentLimit =
      30; // FIXME lower or higher according to wheel slippage

  // Gear reduction for turn motors
  public static final double turnMotorReduction = (150.0 / 7.0); // Same for every mk4 modules

  // Turn gearbox configuration using a NEO motor with specified reduction
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder position factor (rotor rotations to wheel radians)
  public static final double turnEncoderPositionFactor = 2 * Math.PI / turnMotorReduction;

  // Turn encoder velocity factor (rotor RPM to wheel rad/s)
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / turnMotorReduction;

  // PID constants for turn control
  public static final double turnKp = 2.08;
  public static final double turnKd = 0.0;

  // Simulation PID constants for turn
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;

  // PID input range for turn control in radians
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration constants
  public static final double robotMassKg = 74.088; // FIXME
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;

  // Robot configuration for PathPlanner
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // Simulation configuration using IronMaple's DriveTrainSimulationConfig

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  driveGearbox,
                  turnGearbox,
                  driveMotorReduction,
                  turnMotorReduction,
                  Volts.of(0.1),
                  Volts.of(0.1),
                  Meters.of(wheelRadiusMeters),
                  KilogramSquareMeters.of(0.02),
                  wheelCOF));
}
