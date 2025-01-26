package frc.robot.util;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * EqualsUtil class provides utility methods for comparing double precision numbers and WPILib
 * geometry objects with a certain tolerance (epsilon). This helps in checking equality where
 * floating-point arithmetic might result in slight differences that shouldn't be considered
 * unequal.
 */
public class EqualsUtil {
  /**
   * Compares two double values to see if they are approximately equal within a specified epsilon.
   *
   * @param a The first double value.
   * @param b The second double value.
   * @param epsilon The tolerance within which the values are considered equal.
   * @return True if |a - b| <= epsilon, false otherwise.
   */
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Compares two double values to see if they are approximately equal using a default epsilon of
   * 1e-9.
   *
   * @param a The first double value.
   * @param b The second double value.
   * @return True if |a - b| <= 1e-9, false otherwise.
   */
  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  /**
   * A nested static class providing extension methods for comparing WPILib geometry objects like
   * Twist2d.
   */
  public static class GeomExtensions {
    /**
     * Checks if two Twist2d objects are approximately equal in their dx, dy, and dtheta components.
     * Uses the epsilonEquals method to compare each component.
     *
     * @param twist The first Twist2d object.
     * @param other The second Twist2d object.
     * @return True if all corresponding components are approximately equal, false otherwise.
     */
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
      return EqualsUtil.epsilonEquals(twist.dx, other.dx)
          && EqualsUtil.epsilonEquals(twist.dy, other.dy)
          && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
    }
  }
}
