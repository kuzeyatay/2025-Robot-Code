package frc.robot.subsystems.superstructure.algeManipulator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.ModeSetter;

// Define the ArmConstants class as a final utility class
public final class AlgeManipulatorConstants {

  // Define the minimum angle for the arm in degrees
  public static final double minAngle = 0.0;
  // Define the maximum angle for the arm in degrees
  public static final double maxAngle = 135;

  // Convert the minimum angle to radians for soft limit reverse
  public static final double kSoftLimitReverse = Units.degreesToRadians(minAngle);
  // Convert the maximum angle to radians for soft limit forward
  public static final double kSoftLimitForward = Units.degreesToRadians(maxAngle);

  // Define the origin point of the arm using Translation2d (FIXME indicates this may need
  // attention)
  public static final Translation2d armOrigin = new Translation2d(0, 0); // FIXME ????
  public static final double kArmMass = 0.5; // Kilograms
  // Calculate the gear ratio for the arm based on multiple gear stages
  public static final double kArmGearRatio = (42.0 / 30.0) * (64 / 14);

  // Define the CAN ID for the leader motor controller
  public static final int leaderID = 16;

  // Specify whether the leader motor should be inverted
  public static final boolean leaderInverted = true;

  // Define the cosine offset to apply to the arm's zero position in radians
  // This adjusts the converted arm position to match the real-world arm position
  public static final double kArmZeroCosineOffset =
      0.0; //  FIXME radians to add to converted arm position to get real-world arm position (starts
  // at ~x
  // deg angle if using abs encoder)

  // Define the motion constraints for the arm using trapezoidal profiling (max velocity and
  // acceleration)
  public static final TrapezoidProfile.Constraints kArmMotionConstraint =
      new TrapezoidProfile.Constraints(40.0, 40.0);

  // Define the length of the arm in meters by converting from inches
  public static final double armLength = 0.305;

  // Define the PID and feedforward gains based on the current mode (SIM, REAL, REPLAY)
  public static final Gains gains =
      switch (ModeSetter.currentMode) {
          // In simulation mode, use higher kP and set other gains accordingly
        case SIM -> new Gains(7.5, 0.0, 0.0, 0.0, 0.07, 0.02, 0.2);
          // In real or replay modes, use different gains for actual hardware
        case REAL, REPLAY -> new Gains(25, 0.0, 0.5, 0.25, 0.06, 0.01, 0.3);
      };

  // Define a record to hold the PID and feedforward gains
  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
