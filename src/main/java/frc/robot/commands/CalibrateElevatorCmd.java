// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateElevatorCmd extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;

  /** Creates a new CalibrateElevatorCmd. */
  public CalibrateElevatorCmd(ElevatorSubsystem elevatorSubsystem) {
    m_elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.resetCalibration();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.calibrate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setMotorSpeed(0);
    m_elevatorSubsystem.setPIDSetpoint(m_elevatorSubsystem.getHeightInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.isCalibrated();
  }
}
