package frc.robot.subsystems.genericFlywheels;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/**
 * FlywheelIOSparkMax is a hardware implementation of the FlywheelIO interface, using two NEO motors
 * controlled by SPARK MAX controllers. The spark motor is directly controlled, while the follower
 * motor mirrors the spark. This class configures the motors, handles sensor feedback (encoder), and
 * provides methods to control the flywheel in both open-loop and closed-loop modes.
 */
public class GenericFlywheelsIOSparkMax implements GenericFlywheelsIO {
  // Defines the gear ratio for the flywheel mechanism (if any)
  private double GEAR_RATIO = 42 / 18;
  private boolean brakeModeEnabled = true;
  private boolean isAbsoluteConnected = false;
  private AbsoluteEncoder absoluteEncoder;

  // Creates a SPARK MAX controller for the spark motor, using a Brushless NEO
  private final SparkMax spark;

  // Retrieves a RelativeEncoder from the spark motor controller
  private final RelativeEncoder encoder;

  // Retrieves a SparkClosedLoopController for closed-loop (PID) control from the spark controller
  private final SparkClosedLoopController conroller;

  // Configuration object for the SPARK MAX controllers
  private final SparkMaxConfig config = new SparkMaxConfig();

  /**
   * Constructor sets up the SPARK MAX controllers and configures parameters such as voltage
   * compensation and current limit. It also applies the configurations to both spark and follower
   * controllers.
   */
  public GenericFlywheelsIOSparkMax(int deviceId, double GEAR_RATIO, boolean isAbsoluteConnected) {
    this.GEAR_RATIO = GEAR_RATIO;
    this.isAbsoluteConnected = isAbsoluteConnected;
    // Configure voltage compensation and current limits on the SPARK MAX controllers

    // Creates a SPARK MAX controller for the spark motor, using a Brushless NEO
    spark = new SparkMax(deviceId, MotorType.kBrushless);

    // Retrieves a RelativeEncoder from the spark motor controller
    encoder = spark.getEncoder();

    // Retrieves a SparkClosedLoopController for closed-loop (PID) control from the spark controller
    conroller = spark.getClosedLoopController();

    if (isAbsoluteConnected) {
      absoluteEncoder = spark.getAbsoluteEncoder();

      config.absoluteEncoder.positionConversionFactor(93.93).inverted(false);
    }

    config.voltageCompensation(12.0).smartCurrentLimit(40);
    config.idleMode(
        brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 0
        .p(0.8)
        .i(0.0)
        .d(0.0)
        // velocityFF is a feedforward for velocity mode; here just an example value
        .velocityFF(1.0 / 5767)
        // Limit output from -1 to 1 (full reverse to full forward)
        .outputRange(-1, 1);

    // Apply the configuration to the spark motor with retries (using SparkUtil's tryUntilOk)
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Updates the FlywheelIOInputs with the latest sensor and status data from the flywheel hardware.
   * This includes position, velocity, applied voltage, and current draws from both motors.
   *
   * @param inputs The FlywheelIOInputs instance to populate with current data.
   */
  @Override
  public void updateInputs(GenericFlywheelsIOInputs inputs) {
    // Convert the encoder's position from rotations to radians, dividing by GEAR_RATIO if necessary
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    // Convert the encoder's velocity from RPM to rad/s, dividing by GEAR_RATIO if necessary
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    // Calculate the applied voltage to the spark motor by multiplying output percentage by bus
    // voltage
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    // Retrieve the output currents from both the spark and follower motors
    inputs.currentAmps = new double[] {spark.getOutputCurrent()};

    if (isAbsoluteConnected) {
      inputs.encoderPositionRad = absoluteEncoder.getPosition() - 35;
    }
  }

  /**
   * Runs the flywheel open-loop at the specified voltage. This directly sets the output voltage
   * without any closed-loop control.
   *
   * @param volts The voltage command to apply to the flywheel motor.
   */
  @Override
  public void setVoltage(double volts) {
    spark.setVoltage(volts);
  }

  /**
   * Runs the flywheel in closed-loop mode at the specified velocity setpoint. Also applies a
   * feedforward voltage to help achieve the target speed.
   *
   * @param velocityRadPerSec The desired angular velocity in radians per second.
   * @param ffVolts The feedforward voltage to apply in addition to the closed-loop output.
   */
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    conroller.setReference(
        // Convert rad/s to RPM and apply the gear ratio
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity, // Using velocity control mode
        ClosedLoopSlot.kSlot2, // PID slot index 0
        ffVolts, // Feedforward in volts
        ArbFFUnits.kVoltage); // Indicates that feedforward is given in volts
  }

  /**
   * Stops the flywheel by halting motor output. This sets the motor output to zero and the flywheel
   * will coast or brake depending on configuration.
   */
  @Override
  public void stop() {
    spark.stopMotor();
  }

  /**
   * Configures the PID constants for velocity control on the SPARK MAX. Sets the feedback sensor as
   * the primary encoder and applies the given PID gains.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    /*
     * Configure the closed loop controller. We want to make sure we set the feedback sensor
     * as the primary encoder. Also, we apply the given PID constants and set output ranges.
     */

    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    new Thread(
            () ->
                tryUntilOk(
                    spark,
                    5,
                    () ->
                        spark.configure(
                            config.idleMode(
                                brakeModeEnabled
                                    ? SparkBaseConfig.IdleMode.kBrake
                                    : SparkBaseConfig.IdleMode.kCoast),
                            SparkBase.ResetMode.kNoResetSafeParameters,
                            SparkBase.PersistMode.kNoPersistParameters)))
        .start();
  }
}
