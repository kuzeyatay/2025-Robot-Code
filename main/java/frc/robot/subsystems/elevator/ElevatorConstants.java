package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.ModeSetter;

// Declares the ElevatorConstants class which holds all constants related to the Elevator subsystem
public class ElevatorConstants {
  // Defines the elevatorGearbox as a Kraken X60 FOC motor with 2 motors in the gearbox
  public static final DCMotor elevatorGearbox = DCMotor.getKrakenX60Foc(2);

  // Defines the gear ratio for the elevator mechanism
  public static final double elevatorGearRatio = 8.46;

  // Defines the CAN ID for the leader motor of the elevator
  public static final int leaderID = 15;

  // Defines the CAN ID for the follower motor of the elevator
  public static final int followerID = 16;

  // Indicates whether the leader motor is inverted
  public static final boolean leaderInverted = false;

  // Indicates whether the follower motor is inverted
  public static final boolean followerInverted = false;

  // Defines the origin point of the elevator in 2D space; FIXME indicates it needs to be set
  // correctly
  public static final Translation2d elevatorOrigin = new Translation2d(100, 0); // FIXME ????

  // Defines the static feedforward gain for the elevator (in volts)
  public static final double kElevatorkS = 0.0; // volts (V)

  // Defines the gravity compensation feedforward gain for the elevator (in volts)
  public static final double kElevatorkG = 0.762; // volts (V)

  // Defines the velocity feedforward gain for the elevator (in volts per meter per second)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))

  // Defines the gearing factor for the elevator mechanism
  public static final double kElevatorGearing = 10.0;

  // Defines the radius of the drum driving the elevator, converted from inches to meters; FIXME
  // indicates it needs to
  // be set correctly
  public static final double kElevatorDrumRadius =
      Units.inchesToMeters(2.0); // FIXME the radius of the sprocket driving the elevator

  // Defines the mass of the elevator carriage in kilograms
  public static final double kCarriageMass = 4.0; // kg

  // Comment explaining that the encoder is zeroed at the bottom position, setting the minimum
  // height
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = 0.1;

  // Defines the maximum height the elevator can reach in meters
  public static final double kMaxElevatorHeightMeters = 1.25;

  // Defines the motion constraints for the elevator's trapezoidal motion profile with max velocity
  // and acceleration
  public static final TrapezoidProfile.Constraints elevatorMotionConstraint =
      new TrapezoidProfile.Constraints(2.0, 2.0);

  // Defines the gains for the elevator based on the current mode using a switch expression
  public static final Gains gains =
      switch (ModeSetter.currentMode) {
        case SIM -> new Gains(75, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0);
        case REAL, REPLAY -> new Gains(10.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
      };

  // Defines a record to hold the PID and feedforward gains for the elevator
  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
