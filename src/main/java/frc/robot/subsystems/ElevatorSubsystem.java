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
  private final VictorSPX m_motor1 = new VictorSPX(ElevatorConstants.kMotor1Id);
  private final VictorSPX m_motor2 = new VictorSPX(ElevatorConstants.kMotor2Id);

  private final Encoder m_encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final DigitalInput m_lowerLimitSwitch = new DigitalInput(2);
  private final DigitalInput m_upperLimitSwitch = new DigitalInput(3);

  private final ProfiledPIDController m_pidController = new ProfiledPIDController(0.5, 0, 0,
      new TrapezoidProfile.Constraints(42, 42));
  // private final ElevatorFeedforward m_ffController = new ElevatorFeedforward();

  private boolean m_calibrated = false;
  private double m_setpoint = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // TODO: Initialize the encoder with the minimum inches from the ground
    // TODO: Invert motor1 and/or motor2 if needed
    m_motor1.setInverted(false);
    m_motor2.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/HeightInches", getHeightInches());

    if (m_lowerLimitSwitch.get()) {
      m_encoder.reset();
      m_calibrated = true;
    }

    // if (m_calibrated) {
    // // If the elevator is calibrated, go to the setpoint
    // double pidOutput = m_pidController.calculate(getHeightInches(), m_setpoint);
    // m_motor1.set(VictorSPXControlMode.PercentOutput, pidOutput);
    // m_motor2.set(VictorSPXControlMode.PercentOutput, pidOutput);
    // }
  }

  private void setHeightInches(double heightInches) {
    m_setpoint = heightInches;
  }

  private double getHeightInches() {
    return m_encoder.getDistance() / 256; // 256 ticks per revolution gives us a distance in revolutions
    // TODO: Convert revolutions to inches from the ground using gearbox info
  }

  public Command setMotorSpeed(DoubleSupplier speed) {
    return run(() -> {
      m_motor1.set(VictorSPXControlMode.PercentOutput, speed.getAsDouble());
      m_motor2.set(VictorSPXControlMode.PercentOutput, speed.getAsDouble());
    });
  }
}
