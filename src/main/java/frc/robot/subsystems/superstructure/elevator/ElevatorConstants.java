package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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

  public static final double kElevatorMaxV = 10.0;
  // Defines the origin point of the elevator in 2D space; FIXME indicates it needs to be set
  // correctly
  public static final Translation2d elevatorOrigin = new Translation2d(100, 0); // FIXME ????

  // Defines the radius of the drum driving the elevator, converted from inches to meters
  public static final double kElevatorDrumRadius = Units.inchesToMeters(1.41 / 2.0);

  // FIXME Defines the mass of the elevator carriage in kilograms
  public static final double kCarriageMass = 25.5; // kg

  // Comment explaining that the encoder is zeroed at the bottom position, setting the minimum
  // height
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  public static final double kMinElevatorHeightMeters = 0.0;

  // FIXME Defines the maximum height the elevator can reach in meters
  public static final double kMaxElevatorHeightMeters = 1.2;

  // FIXME TEST Defines the gains for the elevator based on the current mode using a switch
  // expression
  public static final Gains gains =
      switch (ModeSetter.currentMode) {
        case SIM -> new Gains(1000, 0.0, 2, 2.0, 9.29, 0.03, 0.19);
        case REAL, REPLAY -> new Gains(02, 0.0, 0.0, 0.0, 9.29, 0.03, 0.19);
      };

  // Defines a record to hold the PID and feedforward gains for the elevator
  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
