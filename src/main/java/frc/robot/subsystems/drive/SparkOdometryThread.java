package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  // List of Spark motor controllers to monitor for errors
  private final List<SparkBase> sparks = new ArrayList<>();

  // List of DoubleSupplier functions for Spark-specific signals (e.g., encoder positions)
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();

  // List of DoubleSupplier functions for generic signals (non-Spark controllers)
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();

  // Queues to store Spark-specific signal values
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();

  // Queues to store generic signal values
  private final List<Queue<Double>> genericQueues = new ArrayList<>();

  // Queues to store timestamp values for odometry updates
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  // Singleton instance of SparkOdometryThread
  private static SparkOdometryThread instance = null;

  // Notifier to run the odometry update loop at a fixed rate
  private Notifier notifier = new Notifier(this::run);

  /**
   * Retrieves the singleton instance of SparkOdometryThread.
   *
   * @return The singleton instance.
   */
  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  /**
   * Private constructor to enforce singleton pattern. Initializes the notifier with a descriptive
   * name.
   */
  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  /**
   * Starts the odometry thread if there are timestamp queues registered. The thread runs
   * periodically based on the odometry frequency defined in DriveConstants.
   */
  public void start() {
    if (!timestampQueues.isEmpty()) {
      notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
    }
  }

  /**
   * Registers a Spark signal to be read from the odometry thread. This method associates a Spark
   * motor controller with a specific signal.
   *
   * @param spark The Spark motor controller to monitor.
   * @param signal The DoubleSupplier function that retrieves the desired signal from the Spark.
   * @return A queue that will store the signal values for odometry processing.
   */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    // Create a new queue with a capacity of 20 elements
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    // Acquire the odometry lock to ensure thread-safe access
    Drive.odometryLock.lock();
    try {
      // Add the Spark motor controller to the list
      sparks.add(spark);

      // Add the signal supplier to the Spark-specific signals list
      sparkSignals.add(signal);

      // Add the corresponding queue to the Spark-specific queues list
      sparkQueues.add(queue);
    } finally {
      // Release the odometry lock
      Drive.odometryLock.unlock();
    }

    // Return the queue to the caller for further processing
    return queue;
  }

  /**
   * Registers a generic signal to be read from the odometry thread. This method is used for signals
   * that are not associated with Spark motor controllers.
   *
   * @param signal The DoubleSupplier function that retrieves the desired generic signal.
   * @return A queue that will store the signal values for odometry processing.
   */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    // Create a new queue with a capacity of 20 elements
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    // Acquire the odometry lock to ensure thread-safe access
    Drive.odometryLock.lock();
    try {
      // Add the signal supplier to the generic signals list
      genericSignals.add(signal);

      // Add the corresponding queue to the generic queues list
      genericQueues.add(queue);
    } finally {
      // Release the odometry lock
      Drive.odometryLock.unlock();
    }

    // Return the queue to the caller for further processing
    return queue;
  }

  /**
   * Creates and returns a new queue for timestamp values. These timestamps correspond to the times
   * at which odometry samples are taken.
   *
   * @return A queue that will store timestamp values.
   */
  public Queue<Double> makeTimestampQueue() {
    // Create a new queue with a capacity of 20 elements
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    // Acquire the odometry lock to ensure thread-safe access
    Drive.odometryLock.lock();
    try {
      // Add the queue to the list of timestamp queues
      timestampQueues.add(queue);
    } finally {
      // Release the odometry lock
      Drive.odometryLock.unlock();
    }

    // Return the queue to the caller for further processing
    return queue;
  }

  /**
   * The main loop that runs periodically to read and store signal values. This method is invoked by
   * the Notifier at a fixed rate.
   */
  private void run() {
    // Acquire the odometry lock to ensure thread-safe access
    Drive.odometryLock.lock();
    try {
      // Get the current timestamp in seconds by converting FPGA time from microseconds
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Array to store Spark signal values
      double[] sparkValues = new double[sparkSignals.size()];

      // Flag to determine if all Spark signals are valid (no errors)
      boolean isValid = true;

      // Iterate through all Spark signals to retrieve their current values
      for (int i = 0; i < sparkSignals.size(); i++) {
        // Retrieve the signal value using the DoubleSupplier
        sparkValues[i] = sparkSignals.get(i).getAsDouble();

        // Check if the associated Spark motor controller has any errors
        if (sparks.get(i).getLastError() != REVLibError.kOk) {
          // If an error is detected, mark the entire sample as invalid
          isValid = false;
        }
      }

      // If all Spark signals are valid, proceed to store the values in their respective queues
      if (isValid) {
        // Iterate through Spark-specific signals and add them to their queues
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.get(i).offer(sparkValues[i]);
        }

        // Iterate through generic signals and add them to their queues
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }

        // Iterate through timestamp queues and add the current timestamp
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      // Release the odometry lock
      Drive.odometryLock.unlock();
    }
  }
}
