// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(ElevatorConstants.kMotorId, MotorType.kBrushed);

  private final Encoder m_encoder = new Encoder(0, 1, true, EncodingType.k4X);

  private final ProfiledPIDController m_pidController = new ProfiledPIDController(0.8, 0, 0,
      new TrapezoidProfile.Constraints(42, 24)); // 42
  // TODO: Make elevator faster

  private final Timer m_calibrationTimer = new Timer();

  private boolean m_calibrated = false;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true)
        .idleMode(IdleMode.kBrake);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString(ElevatorConstants.kSlash + "Current Command",
        this.getCurrentCommand() == null ? "" : this.getCurrentCommand().getName());
    SmartDashboard.putNumber(ElevatorConstants.kSlash + "HeightInches", Math.round(getHeightInches() * 100.0) / 100.0);
    SmartDashboard.putNumber(ElevatorConstants.kSlash + "Setpoint", m_pidController.getGoal().position);
    SmartDashboard.putNumber(ElevatorConstants.kSlash + "Output Current", m_motor.getOutputCurrent());
    SmartDashboard.putBoolean(ElevatorConstants.kSlash + "Calibrated", m_calibrated);

    // Failsafe in case the setpoint is ever below the minimum height
    if (m_pidController.getGoal().position < ElevatorConstants.kMinimumHeightInches) {
      m_pidController.setGoal(new TrapezoidProfile.State(ElevatorConstants.kMinimumHeightInches, 0));
    }

    // Failsafe in case the setpoint is ever above the maximum height
    if (m_pidController.getGoal().position > ElevatorConstants.kMaximumHeightInches) {
      m_pidController.setGoal(new TrapezoidProfile.State(ElevatorConstants.kMaximumHeightInches, 0));
    }
  }

  public void calibrate() {
    m_calibrationTimer.start();

    if (m_motor.getOutputCurrent() < 10) {
      m_calibrationTimer.reset();
    }

    if (m_calibrationTimer.get() < 0.5) {
      setMotorSpeed(-0.2);
    } else {
      setMotorSpeed(0);
      m_encoder.reset();
      m_pidController.reset(new TrapezoidProfile.State(getHeightInches(), 0));
      m_calibrated = true;
    }
  }

  public void bounceL4() {
    if (Math.abs(ElevatorConstants.kL4HeightInches - getHeightInches()) < 1) {
      setPIDSetpoint(ElevatorConstants.kL3HeightInches);
    } else {
      setPIDSetpoint(ElevatorConstants.kL4HeightInches);
    }
  }

  public void setPIDSetpoint(double setpoint) {
    m_pidController.reset(getHeightInches());
    m_pidController.setGoal(new TrapezoidProfile.State(setpoint, 0));
  }

  public void runPID() {
    if (m_calibrated) {
      double pidOutput = m_pidController.calculate(getHeightInches());
      SmartDashboard.putNumber(ElevatorConstants.kSlash + "PID Output", pidOutput);
      setMotorSpeed(0.12 + pidOutput);
    } else {
      calibrate();
    }
  }

  public void manualControl(double speed) {
    setMotorSpeed(speed);
  }

  public double getHeightInches() {
    return (m_encoder.getDistance() / 256) * 5.222 + 7.125;
  }

  // Methods for setting the motor speed manually **if all else fails which it
  // very well might...**
  public Command setMotorSpeedCommand(DoubleSupplier speed) {
    return run(() -> setMotorSpeed(speed.getAsDouble()));
  }

  public Command setMotorSpeedCommand(DoubleSupplier speed, boolean ignoreSafety) {
    return run(() -> setMotorSpeed(speed.getAsDouble(), ignoreSafety));
  }

  public void setMotorSpeed(double speed) {
    setMotorSpeed(speed, false);
  }

  public void setMotorSpeed(double speed, boolean ignoreSafety) {
    if (m_calibrated) {
      if (speed < 0 && getHeightInches() > ElevatorConstants.kMinimumHeightInches) {
        m_motor.set(speed);
      } else if (speed > 0 && getHeightInches() < ElevatorConstants.kMaximumHeightInches) {
        m_motor.set(speed);
      } else {
        m_motor.set(0);
      }
    } else {
      m_motor.set(speed);
    }
  }

  public void resetCalibration() {
    m_calibrationTimer.stop();
    m_calibrationTimer.reset();
    m_calibrated = false;
  }

  public boolean isCalibrated() {
    return m_calibrated;
  }

  public void resetPIDController() {
    m_pidController.reset(new TrapezoidProfile.State(getHeightInches(), 0));
    m_pidController.setGoal(new TrapezoidProfile.State(getHeightInches(), 0));
  }
}
