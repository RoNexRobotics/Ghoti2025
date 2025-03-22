package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static final class SwerveConstants {
        public static final String kSlash = "Swerve/";
        public static final File kSwerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        // Theoretical no-load free speed: 21.97 ft/s
        // More realistic max speed: 16.81 ft/s
        public static final double kMaxSpeed = Units.feetToMeters(14);
        public static final boolean kMegaTag2Enabled = true;
    }

    public static final class ElevatorConstants {
        public static final String kSlash = "Elevator/";
        public static final double kMinimumHeightInches = 30;
        public static final double kMaximumHeightInches = 72;
        public static final int kMotor1Id = 13;
        public static final int kMotor2Id = 14;
    }

    public static final class ShooterConstants {
        public static final String kSlash = "Shooter/";
        public static final int kMotorId = 15;
    }

    public static final class ClimberConstants {
        public static final String kSlash = "Climber/";
        public static final int kMotorId = 16;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriverControllerTranslationDeadband = 0.1;
        public static final double kDriverControllerRotationDeadband = 0.1;
        public static final int kOperatorControllerPort = 1;
    }
}
