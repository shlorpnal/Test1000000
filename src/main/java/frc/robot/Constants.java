package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static class ClimbConstants {
    public static final int climbID = 25;
    public static final double kS = 0.18;
    public static final double kV = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class SerializerConstants {
    public static final int serializerID = 22;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IndexerConstants {
    public static final int indexerID = 18;
  }

  public static class SlapdownConstants {
    public static final int slapdownID = 20; // Placeholder
    public static final int kP = 1; // Placeholder
    public static final int kI = 0; // Placeholder
    public static final int kD = 0; // Placeholder
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final int intakeID = 21;
  }

  public static class ShooterConstants {

    public static final int shooterLID = 15; // Right Motor
    public static final double LkP = 0.0000145; // 0.0000145// 0.0071881;
    public static final double LkI = 0;
    public static final double LkD = 0; // 0.0001; // 0.005;
    public static final double LkS = 0.46606;
    public static final double LkV = 0.1025;//0.116;
    public static final double LkA = 0; // 0.021737;

    public static final int shooterFID = 16; // Left Motor
    public static final double FkP = 0;
    public static final double FkI = 0;
    public static final double FkD = 0;
    public static final double FkS = 0;
    public static final double FkV = 0;
    public static final double FkA = 0;

    public static final double RPM_TOLERANCE = 50;

    // get more accurate gear ratio
    public static final double gearRatio = 1.2;
    public static final double DEGREES_PER_ROT = 360 / gearRatio;

    public static final double max_RPM = 6811;
    public static final double targetRPM = 0;
    public static final double shotslope = 0;
  }

  public static class HoodConstants {

    public static final int hoodID = 17;
    public static final double kP = 0.05;//0.00082;//0.09725;
    public static final double kI = 0.0;
    public static final double kD = 0.0000; //0
    public static final double kS = 0.55;
    public static final double kV = 0.055;
    public static final double kA = 0.0;
    public static final double kG = 0.2; // 0.25;
    public static final double pos_tolerance = 1; // degrees

    // Encoder -> degrees
    public static final double gearRatio = 14.0 / 42.0;
    public static final double hoodRatio = (10.0 / 100.0);
    public static final double DEGREES_PER_ROT = 360 / gearRatio;
    // theoretical must change
    public static final double DEGREES_PER_ROT_HOOD = DEGREES_PER_ROT * hoodRatio;

    // from horizontal
    public static final double minAngle = 0;
    public static final double maxAngle = 30;
    public static final double hoodslope = 0;
  }

  public static class LimeLightConstants {
    // Limelight
    // All in inches
    public static final double LIMELIGHT_HEIGHT = 14.5; // 14.5
    public static final double LIMELIGHT_ANGLE = 31;

    // Hub
    public static final double TARGET_HEIGHT = 72;
    public static final double HUB_APRILTAG_HEIGHT = 44.25; // 44.5
  }

  public static final double stickDeadband = 0.1;

  public static final class Swerve {
    public static final int pigeonID = 14;

    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK5n.KrakenX60_MK5n(
            COTSTalonFXSwerveConstants.SDS.MK5n.driveRatios.L3);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(27.5);
    public static final double wheelBase = Units.inchesToMeters(27.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorAInvert = chosenModule.driveMotorAInvert;
    public static final InvertedValue driveMotorBInvert = chosenModule.driveMotorBInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKS = 0.1;
    public static final double angleKV = 2.49;
    public static final double angleKA = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    // public static final double driveKF = 0.0;

    public static final double driveKS = 0.0;
    public static final double driveKV = 0.124;
    public static final double driveKA = 0.0;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;
    /** Radians per Second */
    public static final double maxAngularVelocity = 4.45; // tune

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // FrontLeft
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.152344);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // FrontRight
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.148682);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // BackLeft
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.119873);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // BackRight
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.226074);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants { // Tune
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PoseEstimator {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  }
}
