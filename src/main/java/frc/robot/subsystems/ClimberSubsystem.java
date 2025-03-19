// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  // private final SparkMax m_climberMotor = new
  // SparkMax(ClimberConstants.kMotorId, MotorType.kBrushed);
  private final VictorSPX m_motor = new VictorSPX(ClimberConstants.kMotorId);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setSpeedCommand(double speed) {
    return run(() -> setSpeed(speed));
  }

  private void setSpeed(double speed) {
    m_motor.set(VictorSPXControlMode.PercentOutput, speed);
  }
}
