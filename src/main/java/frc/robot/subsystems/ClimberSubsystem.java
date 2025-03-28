// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final VictorSPX m_motor1 = new VictorSPX(ClimberConstants.kMotor1Id);
  private final VictorSPX m_motor2 = new VictorSPX(ClimberConstants.kMotor2Id);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor1.configFactoryDefault();
    m_motor2.configFactoryDefault();

    m_motor1.setInverted(false);
    m_motor2.setInverted(false);

    m_motor1.setNeutralMode(NeutralMode.Brake);
    m_motor2.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
  }

  public Command setSpeedCommand(DoubleSupplier speed) {
    return run(() -> setSpeed(speed.getAsDouble()));
  }

  public Command setSpeedCommand(double speed) {
    return run(() -> setSpeed(speed));
  }

  private void setSpeed(double speed) {
    m_motor1.set(VictorSPXControlMode.PercentOutput, speed);
    m_motor2.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
