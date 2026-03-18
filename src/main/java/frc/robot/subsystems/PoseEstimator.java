package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends SubsystemBase {

  private final SwerveDrivePoseEstimator sEstimator;
  public static final Pose2d HubPose = new Pose2d(4.633, 4.040, Rotation2d.fromDegrees(0));
  public static double targetYaw;
  private final Swerve s_Swerve;

  public PoseEstimator(Swerve swerve) {
    this.s_Swerve = swerve;
    sEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            s_Swerve.getGyroYaw(),
            s_Swerve.getModulePositions(),
            new Pose2d(),
            Constants.PoseEstimator.stateStdDevs,
            Constants.PoseEstimator.visionStdDevs);
  }

  /** Update odometry with gyro and module positions (call every loop) */
  public void updateSwerve() {
    sEstimator.update(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());
  }

  /**
   * Add a vision measurement to the estimator
   *
   * @param visionPose Pose from Vision subsystem
   * @param timestamp Seconds timestamp
   * @param visionStdDevs 3x1 matrix [x std, y std, heading std]
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestamp, Matrix<N3, N1> visionStdDevs) {
    sEstimator.addVisionMeasurement(visionPose, timestamp, visionStdDevs);
  }

  /** Get the current estimated pose */
  public Pose2d getEstimatedPosition() {
    return sEstimator.getEstimatedPosition();
  }

  /** Fully reset pose and gyro heading */
  public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    sEstimator.resetPosition(gyroAngle, modulePositions, pose);
  }

  /** Reset just the robot heading (rotation) */
  public void resetHeading(Rotation2d heading, SwerveModulePosition[] modulePositions) {
    Pose2d current = sEstimator.getEstimatedPosition();
    sEstimator.resetPosition(
        new Rotation2d(heading.getRadians()),
        modulePositions,
        new Pose2d(current.getTranslation(), heading));
  }

  /** Zero the robot heading (sets yaw to 0) */
  public void zeroHeading(SwerveModulePosition[] modulePositions) {
    Pose2d current = sEstimator.getEstimatedPosition();
    sEstimator.resetPosition(
        new Rotation2d(), modulePositions, new Pose2d(current.getTranslation(), new Rotation2d()));
  }

  public double getAngleToHub(Pose2d robotPose) {

    double dx = HubPose.getX() - robotPose.getX();
    double dy = HubPose.getY() - robotPose.getY();

    return Math.toDegrees(Math.atan2(dy, dx));
}

  @Override
  public void periodic() {
    Pose2d pose = getEstimatedPosition();
    updateSwerve();

    SmartDashboard.putNumber("robotX", pose.getX());
    SmartDashboard.putNumber("robotY", pose.getY());
    SmartDashboard.putNumber("robotHeading", pose.getRotation().getRadians());

    Logger.recordOutput("PoseEstimator/EstimatedPose", pose);
  }
}
