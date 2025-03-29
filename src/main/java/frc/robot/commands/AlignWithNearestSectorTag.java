// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AprilTagUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithNearestSectorTag extends Command {
  private final SwerveSubsystem m_swerveSubsystem;

  private final ProfiledPIDController m_xController = new ProfiledPIDController(2, 0, 0,
      new TrapezoidProfile.Constraints(2, 1));
  private final ProfiledPIDController m_yController = new ProfiledPIDController(2, 0, 0,
      new TrapezoidProfile.Constraints(2, 1));

  private final Transform2d m_offset;

  private Pose2d m_targetPose;

  /** Creates a new AlignToNearestSectorCmd. */
  public AlignWithNearestSectorTag(SwerveSubsystem swerveSubsystem, Transform2d offset) {
    m_swerveSubsystem = swerveSubsystem;
    m_offset = offset;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController.reset(m_swerveSubsystem.getPose().getX());
    m_yController.reset(m_swerveSubsystem.getPose().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int tagId = m_swerveSubsystem.reefSectors.getNearestSectorTag();

    if (tagId == -1)
      return;

    Pose2d tagPose = AprilTagUtils.getAprilTagPose3d(tagId).toPose2d();
    m_targetPose = tagPose.transformBy(m_offset);

    if (!m_swerveSubsystem.isRedAlliance()) {
      m_swerveSubsystem.drive(
          m_xController.calculate(m_swerveSubsystem.getPose().getX(), m_targetPose.getX()),
          m_yController.calculate(m_swerveSubsystem.getPose().getY(), m_targetPose.getY()),
          m_targetPose.getRotation().getSin(),
          m_targetPose.getRotation().getCos());
    } else {
      m_swerveSubsystem.drive(
          -m_xController.calculate(m_swerveSubsystem.getPose().getX(), m_targetPose.getX()),
          -m_yController.calculate(m_swerveSubsystem.getPose().getY(), m_targetPose.getY()),
          m_targetPose.getRotation().getSin(),
          m_targetPose.getRotation().getCos());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(
        0,
        0,
        m_targetPose.getRotation().getSin(),
        m_targetPose.getRotation().getCos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_targetPose == null) {
      return false;
    }

    Pose2d currentPose = m_swerveSubsystem.getPose();
    double xError = Math.abs(currentPose.getX() - m_targetPose.getX());
    double yError = Math.abs(currentPose.getY() - m_targetPose.getY());

    return xError < 0.1 && yError < 0.1; // Thresholds for position
  }
}
