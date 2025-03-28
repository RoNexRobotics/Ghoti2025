// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCoralCmd extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final Timer m_timer = new Timer();
  private final DoubleSupplier m_speedSupplier;
  private final BooleanSupplier m_timedSupplier;

  /** Creates a new ShootCoralCmd. */
  public ShootCoralCmd(ShooterSubsystem shooterSubsystem, DoubleSupplier speedSupplier, BooleanSupplier timedSupplier) {
    m_shooterSubsystem = shooterSubsystem;
    m_speedSupplier = speedSupplier;
    m_timedSupplier = timedSupplier;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_shooterSubsystem.setSpeed(m_speedSupplier.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setSpeed(0);
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timedSupplier.getAsBoolean()) {
      return m_timer.get() >= 3;
    } else {
      return false;
    }
  }
}
