// Copybottom (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.*;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class FlywheelsIOSparkMax implements FlywheelsIO {
  // Hardware

  // Creates a SPARK MAX controller for the leader Spark, using a Brushless NEO
  private final SparkMax topSpark;
  // Creates a SPARK MAX controller for the follower Spark, also a Brushless NEO
  private final SparkMax bottomSpark;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  // Controllers
  // Retrieves a SparkClosedLoopController for closed-loop (PID) control from the topSpark
  // controller
  private final SparkClosedLoopController topConroller;
  // Retrieves a SparkClosedLoopController for closed-loop (PID) control from the bottomSpark
  // controller
  private final SparkClosedLoopController bottomConroller;

  // Configuration object for the SPARK MAX controllers
  private final SparkMaxConfig topConfig = new SparkMaxConfig();
  private final SparkMaxConfig bottomConfig = new SparkMaxConfig();

  public FlywheelsIOSparkMax() {

    topSpark = new SparkMax(flywheelConfig.topID(), MotorType.kBrushless);
    bottomSpark = new SparkMax(flywheelConfig.bottomID(), MotorType.kBrushless);

    topEncoder = topSpark.getEncoder();
    bottomEncoder = bottomSpark.getEncoder();
    topConroller = topSpark.getClosedLoopController();
    bottomConroller = bottomSpark.getClosedLoopController();

    // Configure voltage compensation and current limits on the SPARK MAX controllers
    topConfig.voltageCompensation(12.0).smartCurrentLimit(80);
    // Configure voltage compensation and current limits on the SPARK MAX controllers
    bottomConfig.voltageCompensation(12.0).smartCurrentLimit(80);

    // Apply the configuration to the leader Spark with retries (using SparkUtil's tryUntilOk)
    tryUntilOk(
        topSpark,
        5,
        () ->
            topSpark.configure(
                topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Apply the same configuration to the follower Spark
    tryUntilOk(
        bottomSpark,
        5,
        () ->
            bottomSpark.configure(
                bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.topPositionRads =
        Units.rotationsToRadians(topEncoder.getPosition()) / flywheelConfig.reduction();
    inputs.topVelocityRpm = topEncoder.getVelocity() / flywheelConfig.reduction();
    inputs.topAppliedVolts = topSpark.getAppliedOutput() * topSpark.getBusVoltage();
    inputs.topSupplyCurrentAmps = topSpark.getOutputCurrent();
    inputs.topTempCelsius = topSpark.getMotorTemperature();

    inputs.bottomPositionRads =
        Units.rotationsToRadians(bottomEncoder.getPosition()) / flywheelConfig.reduction();
    inputs.bottomVelocityRpm = bottomEncoder.getVelocity() / flywheelConfig.reduction();
    inputs.bottomAppliedVolts = bottomSpark.getAppliedOutput() * bottomSpark.getBusVoltage();
    inputs.bottomSupplyCurrentAmps = bottomSpark.getOutputCurrent();
    inputs.bottomTempCelsius = bottomSpark.getMotorTemperature();
  }

  @Override
  public void runVolts(double topVolts, double bottomVolts) {
    topSpark.setVoltage(topVolts);
    bottomSpark.setVoltage(bottomVolts);
  }

  @Override
  public void runVelocity(
      double topRpm, double bottomRpm, double topFeedforward, double bottomFeedforward) {
    topConroller.setReference(
        topRpm * flywheelConfig.reduction(),
        ControlType.kVelocity, // Using velocity control mode
        ClosedLoopSlot.kSlot0,
        topFeedforward,
        ArbFFUnits.kVoltage);

    bottomConroller.setReference(
        bottomRpm * flywheelConfig.reduction(),
        ControlType.kVelocity, // Using velocity control mode
        ClosedLoopSlot.kSlot0,
        bottomFeedforward,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    /*
     * Configure the closed loop controller. We want to make sure we set the feedback sensor
     * as the primary encoder. Also, we apply the given PID constants and set output ranges.
     */
    topConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 0
        .p(kP)
        .i(kI)
        .d(kD)
        // velocityFF is a feedforward for velocity mode; here just an example value
        .velocityFF(1.0 / 5767)
        // Limit output from -1 to 1 (full reverse to full forward)
        .outputRange(-1, 1);

    bottomConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for velocity control in slot 0
        .p(kP)
        .i(kI)
        .d(kD)
        // velocityFF is a feedforward for velocity mode; here just an example value
        .velocityFF(1.0 / 5767)
        // Limit output from -1 to 1 (full reverse to full forward)
        .outputRange(-1, 1);
  }

  @Override
  public void runCharacterizationtop(double input) {
    topSpark.setVoltage(input);
  }

  @Override
  public void runCharacterizationbottom(double input) {
    bottomSpark.setVoltage(input);
  }

  @Override
  public void stop() {
    topSpark.stopMotor();
    ;
    bottomSpark.stopMotor();
  }
}
