// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorManualCmd extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DoubleSupplier m_speedSupplier;

  /** Creates a new ElevatorManualCmd. */
  public ElevatorManualCmd(ElevatorSubsystem elevatorSubsystem, DoubleSupplier speedSupplier) {
    m_elevatorSubsystem = elevatorSubsystem;
    m_speedSupplier = speedSupplier;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setMotorSpeed(m_speedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setPIDSetpoint(m_elevatorSubsystem.getHeightInches());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_speedSupplier.getAsDouble()) < 0.1;
  }
}
