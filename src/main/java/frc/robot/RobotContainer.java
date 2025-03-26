// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithNearestSectorTag;
import frc.robot.commands.ElevatorManualCmd;
import frc.robot.commands.ElevatorPIDCmd;
import frc.robot.commands.ShootCoralCmd;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class RobotContainer {
        // Controllers
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController m_operatorController = new CommandXboxController(
                        OIConstants.kOperatorControllerPort);

        // Subsystems
        private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
        public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
        private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
        private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

        // Commands

        // Other stuff
        private final SendableChooser<Command> m_autoChooser;

        public RobotContainer() {
                // Send a friendly message to the driver
                Elastic.sendNotification(new Notification(NotificationLevel.INFO, "YEYEYE!", "Robot program started."));

                registerNamedCommands();
                configureBindings();

                Command allianceRelativeDirectAngle = m_swerveSubsystem.driveCommand(
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                OIConstants.kDriverControllerTranslationDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                OIConstants.kDriverControllerTranslationDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                OIConstants.kDriverControllerRotationDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRightY(),
                                                OIConstants.kDriverControllerRotationDeadband))
                                .beforeStarting(new InstantCommand(() -> m_swerveSubsystem.setCommandedHeading(),
                                                m_swerveSubsystem));

                m_swerveSubsystem.setDefaultCommand(allianceRelativeDirectAngle);

                m_elevatorSubsystem
                                .setDefaultCommand(new ElevatorPIDCmd(m_elevatorSubsystem));

                // Send the auto chooser to the dashboard
                m_autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", m_autoChooser);

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand("AlignWithNearestSectorTag",
                                new AlignWithNearestSectorTag(m_swerveSubsystem,
                                                new Transform2d(
                                                                Units.inchesToMeters(24),
                                                                Units.inchesToMeters(0),
                                                                Rotation2d.fromDegrees(-90))));

                NamedCommands.registerCommand("Elevator L1", new InstantCommand(
                                () -> m_elevatorSubsystem.setPIDSetpoint(ElevatorConstants.kL1HeightInches)));
                NamedCommands.registerCommand("Elevator L2",
                                new InstantCommand(() -> m_elevatorSubsystem
                                                .setPIDSetpoint(ElevatorConstants.kL2HeightInches)));
                NamedCommands.registerCommand("Elevator L3", new InstantCommand(
                                () -> m_elevatorSubsystem.setPIDSetpoint(ElevatorConstants.kL3HeightInches)));
                NamedCommands.registerCommand("Elevator L4", new InstantCommand(
                                () -> m_elevatorSubsystem.setPIDSetpoint(ElevatorConstants.kL4HeightInches)));

                NamedCommands.registerCommand("Shoot Coral", new ShootCoralCmd(m_shooterSubsystem));
        }

        private void configureBindings() {

                m_driverController.x()
                                .whileTrue(m_swerveSubsystem.alignWithAprilTag(18,
                                                new Transform2d(Units.inchesToMeters(48), Units.inchesToMeters(0),
                                                                Rotation2d.fromDegrees(0))));

                m_driverController.y().whileTrue(new AlignWithNearestSectorTag(m_swerveSubsystem, new Transform2d(
                                Units.inchesToMeters(24),
                                Units.inchesToMeters(0),
                                Rotation2d.fromDegrees(-90))));

                m_driverController.povDown().onTrue(m_climberSubsystem.setSpeedCommand(-1))
                                .onFalse(m_climberSubsystem.setSpeedCommand(0));
                m_driverController.povUp().onTrue(m_climberSubsystem.setSpeedCommand(1))
                                .onFalse(m_climberSubsystem.setSpeedCommand(0));

                // DRIVERS CONTROLS ^^^^^^^^^^^^^^^^^^
                // OPERATORS CONTROLS BELOW CAUSE THERE'S NO DOWN ARROW ON THIS KEYBOARD

                // m_operatorController.button(5).whileTrue(new
                // CalibrateElevatorCmd(m_elevatorSubsystem));

                m_operatorController.povUp().onTrue(new ElevatorManualCmd(m_elevatorSubsystem, () -> 0.4))
                                .onFalse(new ElevatorManualCmd(m_elevatorSubsystem, () -> 0));
                m_operatorController.povDown().onTrue(new ElevatorManualCmd(m_elevatorSubsystem, () -> -0.4))
                                .onFalse(new ElevatorManualCmd(m_elevatorSubsystem, () -> 0));

                m_operatorController.button(1).onTrue(new InstantCommand(
                                () -> m_elevatorSubsystem.setPIDSetpoint(ElevatorConstants.kL1HeightInches)));
                m_operatorController.button(4).onTrue(new InstantCommand(() -> m_elevatorSubsystem
                                .setPIDSetpoint(ElevatorConstants.kL2HeightInches)));
                m_operatorController.button(2).onTrue(new InstantCommand(() -> m_elevatorSubsystem
                                .setPIDSetpoint(ElevatorConstants.kL3HeightInches)));
                m_operatorController.button(3).onTrue(new InstantCommand(
                                () -> m_elevatorSubsystem.setPIDSetpoint(ElevatorConstants.kL4HeightInches)));

                m_operatorController.button(8).onTrue(new ShootCoralCmd(m_shooterSubsystem));
        }

        public Command getAutonomousCommand() {
                return m_autoChooser.getSelected();
        }
}
