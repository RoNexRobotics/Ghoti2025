// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.AprilTagUtils;
import frc.robot.util.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;
  StructArrayPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SmartDashboard/Field/AprilTag Poses", Pose3d.struct).publish();
  public final ReefSectors reefSectors;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    // Set swerve telemetry verbosity
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kSwerveConfigurationDirectory)
          .createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    reefSectors = new ReefSectors(m_swerve::getPose, m_swerve.field);

    // Configure swerve drive
    m_swerve.setHeadingCorrection(true);
    m_swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    m_swerve.setAngularVelocityCompensation(true, true, 0.1);
    m_swerve.setModuleEncoderAutoSynchronize(true, 1);
    m_swerve.setAutoCenteringModules(false);
    m_swerve.setMotorIdleMode(false);

    if (SwerveDriveTelemetry.isSimulation) {
      m_swerve.resetOdometry(new Pose2d(7.588, 4.037, Rotation2d.fromDegrees(0)));
    }

    setupPathPlanner();
    // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
  }

  @Override
  public void periodic() {

    ChassisSpeeds robotVelocity = m_swerve.getRobotVelocity();
    double velocity = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) * 2.23694;
    SmartDashboard.putNumber(SwerveConstants.kSlash + "Speedometer (MPH)", Math.round(velocity * 1000.0) / 1000.0);

    if (SwerveConstants.kMegaTag2Enabled) {
      // Limelight 3G
      // LimelightHelpers.SetRobotOrientation("limelight-better",
      // m_swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
      // 0, 0, 0, 0, 0);
      // LimelightHelpers.PoseEstimate ll3Gmt2 =
      // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-better");
      // if (ll3Gmt2 != null &&
      // Math.abs(m_swerve.getGyro().getYawAngularVelocity().magnitude()) <= 720 &&
      // ll3Gmt2.tagCount != 0) {
      // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
      // m_swerve.addVisionMeasurement(ll3Gmt2.pose, ll3Gmt2.timestampSeconds);
      // }

      // Limelight 1
      // LimelightHelpers.SetRobotOrientation("limelight",
      // m_swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
      // 0, 0, 0, 0, 0);
      // LimelightHelpers.PoseEstimate ll1mt2 =
      // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-better");
      // if (ll1mt2 != null &&
      // Math.abs(m_swerve.getGyro().getYawAngularVelocity().magnitude()) <= 720 &&
      // ll1mt2.tagCount != 0) {
      // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(5, 5, 20));
      // m_swerve.addVisionMeasurement(ll1mt2.pose, ll1mt2.timestampSeconds);
      // }

      // if (mt2 != null) {
      // Pose3d[] poses = new Pose3d[mt2.rawFiducials.length];
      // for (int i = 0; i < mt2.rawFiducials.length; i++) {
      // Pose3d pose = AprilTagUtils.getAprilTagPose3d(mt2.rawFiducials[i].id);
      // poses[i] = pose;
      // }
      // publisher.set(poses);
      // } else {
      // publisher.set(null);
      // }

      LimelightHelpers.PoseEstimate ll3Gmt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-better");

      if (ll3Gmt1 != null && !((ll3Gmt1.tagCount == 1 && ll3Gmt1.rawFiducials.length == 1 &&
          (ll3Gmt1.rawFiducials[0].ambiguity > 0.7 || ll3Gmt1.rawFiducials[0].distToCamera > 3)) ||
          (ll3Gmt1.tagCount == 0))) {
        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, .5));
        m_swerve.addVisionMeasurement(ll3Gmt1.pose, ll3Gmt1.timestampSeconds);
      }

      LimelightHelpers.PoseEstimate ll1mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (ll1mt1 != null && !((ll1mt1.tagCount == 1 && ll1mt1.rawFiducials.length == 1 &&
          (ll1mt1.rawFiducials[0].ambiguity > 0.7 || ll1mt1.rawFiducials[0].distToCamera > 3)) ||
          (ll1mt1.tagCount == 0))) {
        m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, .5));
        m_swerve.addVisionMeasurement(ll1mt1.pose, ll1mt1.timestampSeconds);
      }

    }
  }

  private void setupPathPlanner() {
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          m_swerve::getPose,
          m_swerve::resetOdometry,
          m_swerve::getRobotVelocity,
          (speeds, feedforwards) -> m_swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(10.0, 0.0, 0.0),
              // new PIDConstants(18.0, 0.0, 0.0)),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      if (poses.isEmpty())
        return;

      List<Trajectory.State> states = new ArrayList<>();
      for (Pose2d pose : poses) {
        Trajectory.State state = new Trajectory.State();
        state.poseMeters = pose;
        states.add(state);
      }

      m_swerve.postTrajectory(new Trajectory(states));
    });
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        m_swerve.getMaximumChassisVelocity(), 1.0,
        m_swerve.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  public Command alignWithAprilTag(int id) {
    return alignWithAprilTag(id, Transform2d.kZero);
  }

  public Command alignWithAprilTag(int id, Transform2d offset) {
    Pose3d tagPose = AprilTagUtils.getAprilTagPose3d(id);
    if (tagPose == null) {
      return Commands.none();
    }

    Pose2d targetPose = tagPose.toPose2d().transformBy(offset);
    targetPose = new Pose2d(targetPose.getTranslation(),
        targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    return driveToPose(targetPose);
  }

  public void drive(double translationX, double translationY, double headingX, double headingY) {
    drive(translationX, translationY, headingX, headingY, ElevatorConstants.kMinimumHeightInches);
  }

  public void drive(double translationX, double translationY, double headingX, double headingY, double elevatorHeight) {

    double elevatorMultiplier = 0.5 + ((ElevatorConstants.kMaximumHeightInches - elevatorHeight)
        / (ElevatorConstants.kMaximumHeightInches - ElevatorConstants.kMinimumHeightInches)) * 0.5;

    translationX *= elevatorMultiplier;
    translationY *= elevatorMultiplier;

    Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX,
        translationY), 0.8);

    if (isRedAlliance()) {
      scaledInputs = new Translation2d(-scaledInputs.getX(), -scaledInputs.getY());
    }

    m_swerve.driveFieldOriented(m_swerve.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX,
        headingY,
        m_swerve.getOdometryHeading().getRadians(),
        m_swerve.getMaximumChassisVelocity()));
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerve.driveFieldOriented(velocity.get());
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier headingX, DoubleSupplier headingY, DoubleSupplier elevatorHeightSupplier) {
    return run(() -> {
      drive(translationX.getAsDouble(), translationY.getAsDouble(), headingX.getAsDouble(), headingY.getAsDouble(),
          elevatorHeightSupplier.getAsDouble());
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      drive(translationX.getAsDouble(), translationY.getAsDouble(), headingX.getAsDouble(), headingY.getAsDouble());
    });
  }

  public void setCommandedHeading() {
    drive(0, 0, m_swerve.getOdometryHeading().getSin(), m_swerve.getOdometryHeading().getCos());
  }

  public void zeroGyro() {
    m_swerve.zeroGyro();
  }

  public Pose2d getPose() {
    return m_swerve.getPose();
  }

  public void resetOdometry() {
    m_swerve.resetOdometry(Pose2d.kZero);
  }

  public void resetOdometry(Pose2d pose) {
    m_swerve.resetOdometry(pose);
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerve;
  }

  public boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }
}
