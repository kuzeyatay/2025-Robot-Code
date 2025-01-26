package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics simulation implementation of the ModuleIO interface for simulating swerve module
 * functionality.
 */
public class ModuleIOSim implements ModuleIO {
  // Simulation of the swerve module's physical behavior
  private final SwerveModuleSimulation moduleSimulation;

  // Simulated motor controllers for drive and turn motors
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  // Closed-loop control flags for drive and turn motors
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;

  // PID controllers for closed-loop control of drive and turn motors
  private final PIDController driveController = new PIDController(driveSimP, 0, driveSimD);
  private final PIDController turnController = new PIDController(turnSimP, 0, turnSimD);

  // Variables for drive feedforward voltage and applied voltages
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  /**
   * Constructor for ModuleIOSim.
   *
   * @param moduleSimulation Simulation instance for the swerve module.
   */
  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // Configure simulated drive motor with a current limit
    this.driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(driveMotorCurrentLimit));

    // Configure simulated turn motor with a current limit
    this.turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(turnMotorCurrentLimit));

    // Enable PID wrapping for the turn controller to handle angles continuously
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Updates the simulation inputs based on the current state of the module.
   *
   * @param inputs An instance of ModuleIOInputs to be populated with simulated data.
   */
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Calculate drive motor applied voltage using closed-loop control if enabled
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts
              + driveController.calculate(
                  moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }

    // Calculate turn motor applied voltage using closed-loop control if enabled
    if (turnClosedLoop) {
      turnAppliedVolts =
          turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    // Request the calculated voltages from the simulated motor controllers
    driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
    turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive motor inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn motor inputs
    inputs.turnConnected = true;
    inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs with the current timestamp and positions (sampled at 50 Hz)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  /**
   * Sets the drive motor to run in open-loop mode with the specified output.
   *
   * @param output Desired open-loop output for the drive motor.
   */
  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  /**
   * Sets the turn motor to run in open-loop mode with the specified output.
   *
   * @param output Desired open-loop output for the turn motor.
   */
  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  /**
   * Sets the drive motor to run at the specified velocity in closed-loop mode.
   *
   * @param velocityRadPerSec Desired drive velocity in radians per second.
   */
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = driveSimKs * Math.signum(velocityRadPerSec) + driveSimKv * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  /**
   * Sets the turn motor to move to the specified position in closed-loop mode.
   *
   * @param rotation Desired turn position as a Rotation2d object.
   */
  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
