package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionConsumer consumer;
  private final NetworkTable table;
  private final Alert disconnectedAlert = new Alert("Limelight disconnected", AlertType.kWarning);

  public Vision(VisionConsumer consumer, String limelightName) {
    this.consumer = consumer;
    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  @Override
  public void periodic() {
    double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    double latency = table.getEntry("tl").getDouble(0.0) / 1000.0; // seconds

    double angle = Constants.LimeLightConstants.LIMELIGHT_ANGLE + LimelightHelpers.getTY("limelight");
    System.out.println(
        (Constants.LimeLightConstants.TARGET_HEIGHT - Constants.LimeLightConstants.LIMELIGHT_HEIGHT)
            / Math.tan(Math.toRadians(angle)));
                                                //Needs to be tuned to a multiplied constant
    SmartDashboard.putBoolean("InVision?", hasTarget());
  

    // Reject if no data
    if (botpose.length < 6) {
      disconnectedAlert.set(true);
      return;
    }
    disconnectedAlert.set(false);

    Pose3d rawPose3d =
        new Pose3d(
            new Translation3d(botpose[0], botpose[1], botpose[2]), // x, y, z
            new Rotation3d(botpose[3], botpose[4], botpose[5]) // roll, pitch, yaw (radians)
            );

    boolean rejectPose =
        rawPose3d.getZ() > VisionConstants.maxZError
            || rawPose3d.getZ() < -VisionConstants.maxZError
            || rawPose3d.getX() < 0.0
            || rawPose3d.getX() > VisionConstants.aprilTagLayout.getFieldLength()
            || rawPose3d.getY() < 0.0
            || rawPose3d.getY() > VisionConstants.aprilTagLayout.getFieldWidth();

    if (rejectPose) {
      Logger.recordOutput(
          "Vision/RejectedPose",
          new Pose2d(rawPose3d.getX(), rawPose3d.getY(), rawPose3d.getRotation().toRotation2d()));
      return;
    }

    // Conversion to 2d
    Pose2d visionPose =
        new Pose2d(rawPose3d.getX(), rawPose3d.getY(), rawPose3d.getRotation().toRotation2d());

    // std devs
    double distance =
        Math.sqrt(visionPose.getX() * visionPose.getX() + visionPose.getY() * visionPose.getY());

     double distancefromHub =
        Math.hypot(
            PoseEstimator.HubPose.getX() - visionPose.getX(),
            PoseEstimator.HubPose.getY() - visionPose.getY());

    double linearStdDev = VisionConstants.linearStdDevBaseline * Math.pow(distance, 0.5);//Math.sqrt(distancefromHub);//Math.pow(distance, 0.5);
    double angularStdDev = VisionConstants.angularStdDevBaseline * Math.pow(distance, 0.5);//Math.sqrt(distancefromHub);//Math.pow(distance, 0.5);

    Matrix<N3, N1> stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

    consumer.accept(visionPose, Timer.getFPGATimestamp() - latency, stdDevs);

    Logger.recordOutput("Vision/RobotPose2d", visionPose);
    Logger.recordOutput("Vision/StdDevs", stdDevs);
  }

  public double[] getBotpose() {
    return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
}

   public double getDistanceInches() {
    double ty = LimelightHelpers.getTY("limelight");

    double angle = Constants.LimeLightConstants.LIMELIGHT_ANGLE + ty;

    return ((Constants.LimeLightConstants.TARGET_HEIGHT
            - Constants.LimeLightConstants.LIMELIGHT_HEIGHT))
        / Math.tan(Math.toDegrees(angle));
                                             //Needs to be tuned to a multiplied constant
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV("limelight");
  }

  public double getTX() {
    return LimelightHelpers.getTX("limelight");
  }

  public double getTY() {
    return LimelightHelpers.getTY("limelight");
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
