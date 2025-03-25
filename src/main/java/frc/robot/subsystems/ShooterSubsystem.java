// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final VictorSPX m_motor = new VictorSPX(ShooterConstants.kMotorId);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_motor.configFactoryDefault();

    // TODO: Invert the motor if neccessary
    m_motor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public Command setSpeedCommand(double speed) {
    return run(() -> setSpeed(speed));
  }

  private void setSpeed(double speed) {
    m_motor.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
