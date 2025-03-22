// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final VictorSPX m_motor = new VictorSPX(ElevatorConstants.kMotor1Id);

  private final Encoder m_encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final DigitalInput m_lowerLimitSwitch = new DigitalInput(2);
  private final DigitalInput m_upperLimitSwitch = new DigitalInput(3);

  private final ProfiledPIDController m_pidController = new ProfiledPIDController(0.5, 0, 0,
      new TrapezoidProfile.Constraints(42, 42));

  private boolean m_calibrated = false;
  private double m_setpoint = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_motor.configFactoryDefault();

    // TODO: Invert the motor if necessary
    m_motor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/HeightInches", getHeightInches());

    if (m_lowerLimitSwitch.get()) {
      m_motor.set(VictorSPXControlMode.PercentOutput, 0);
      m_encoder.reset();
      m_calibrated = true;
    }

    // Failsafe in case the setpoint is ever below the minimum height
    if (m_setpoint < ElevatorConstants.kMinimumHeightInches) {
      m_setpoint = ElevatorConstants.kMinimumHeightInches;
    }

    // Failsafe in case the setpoint is ever above the maximum height
    if (m_setpoint > ElevatorConstants.kMaximumHeightInches) {
      m_setpoint = ElevatorConstants.kMaximumHeightInches;
    }

    if (m_calibrated && !m_upperLimitSwitch.get()) {
      // If the elevator is calibrated, go to the setpoint
      double pidOutput = m_pidController.calculate(getHeightInches(), m_setpoint);
      SmartDashboard.putNumber(ElevatorConstants.kSlash + "PID Output", pidOutput);

      // m_motor.set(VictorSPXControlMode.PercentOutput, pidOutput);
    } else if (m_upperLimitSwitch.get()) {
      m_motor.set(VictorSPXControlMode.PercentOutput, 0);
    } else {
      // If the elevator is not calibrated, move down
      m_motor.set(VictorSPXControlMode.PercentOutput, -0.1);
    }
  }

  private double getHeightInches() {
    return m_encoder.getDistance() / 256; // 256 ticks per revolution gives us a distance in revolutions
    // TODO: Convert revolutions to inches from the ground using gearbox info, plus
    // add a constant for the minimum height
  }

  public Command setHeightInchesCommand(double heightInches) {
    return run(() -> setHeightInches(heightInches));
  }

  private void setHeightInches(double heightInches) {
    m_setpoint = heightInches;
  }

  // Methods for setting the motor speed manually **if all else fails which it
  // very well might...**
  public Command setMotorSpeedCommand(DoubleSupplier speed) {
    return run(() -> setMotorSpeed(speed.getAsDouble()));
  }

  public Command setMotorSpeedCommand(double speed) {
    return run(() -> setMotorSpeed(speed));
  }

  private void setMotorSpeed(double speed) {
    m_motor.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
