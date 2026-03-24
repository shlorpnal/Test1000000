package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.SwerveModule;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  public final SwerveModule[] mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

  public final Pigeon2 gyro;
  private final SwerveDriveOdometry swerveOdometry;
  private final Field2d field = new Field2d();
  public static final Pose2d HubPose = new Pose2d(4.633, 4.040, Rotation2d.fromDegrees(0));
  static double targetYaw;

  public SwerveDrivePoseEstimator m_poseEstimator;
  public LimelightHelpers.PoseEstimate mt2;
  public LimelightHelpers.PoseEstimate leftPose;
  public LimelightHelpers.PoseEstimate rightPose;
  public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];

  public static Pose2d botPose2d = new Pose2d();
  public Pose3d botPose3d = new Pose3d();
  public PoseEstimate best = new PoseEstimate();
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  
    MutVoltage voltage = Volts.mutable(0);
    MutDistance distance = Meters.mutable(0);
    MutLinearVelocity velocity = MetersPerSecond.mutable(0);

    final SysIdRoutine m_driveRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, Volts.of(2), null), 
      new SysIdRoutine.Mechanism( (Voltage) -> {
        mSwerveMods[0].returnDriveMotor().set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
        mSwerveMods[1].returnDriveMotor().set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
        mSwerveMods[2].returnDriveMotor().set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
        mSwerveMods[3].returnDriveMotor().set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
      },

      log -> {
      log.motor("Front Left")
      .voltage(voltage.mut_replace(
        mSwerveMods[0].returnDriveMotor().getMotorVoltage().getValueAsDouble() * mSwerveMods[0].returnDriveMotor().getSupplyVoltage().getValueAsDouble(),
        Volts
      ))
      .linearPosition(distance.mut_replace(
        mSwerveMods[0].getDrivePosition(),
        Meters
      ))
      .linearVelocity(velocity.mut_replace(
        mSwerveMods[0].getDriveVelocity(),
        MetersPerSecond
      ));

      log.motor("Front Right")
      .voltage(voltage.mut_replace(
        mSwerveMods[1].returnDriveMotor().getMotorVoltage().getValueAsDouble() * mSwerveMods[1].returnDriveMotor().getSupplyVoltage().getValueAsDouble(),
        Volts
      ))
      .linearPosition(distance.mut_replace(
        mSwerveMods[1].getDrivePosition(),
        Meters
      ))
      .linearVelocity(velocity.mut_replace(
        mSwerveMods[1].getDriveVelocity(),
        MetersPerSecond
      ));

      log.motor("Back Left")
      .voltage(voltage.mut_replace(
        mSwerveMods[2].returnDriveMotor().getMotorVoltage().getValueAsDouble() * mSwerveMods[2].returnDriveMotor().getSupplyVoltage().getValueAsDouble(),
        Volts
      ))
      .linearPosition(distance.mut_replace(
        mSwerveMods[2].getDrivePosition(),
        Meters
      ))
      .linearVelocity(velocity.mut_replace(
        mSwerveMods[2].getDriveVelocity(),
        MetersPerSecond
      ));

      log.motor("Back Right")
      .voltage(voltage.mut_replace(
        mSwerveMods[3].returnDriveMotor().getMotorVoltage().getValueAsDouble() * mSwerveMods[3].returnDriveMotor().getSupplyVoltage().getValueAsDouble(),
        Volts
      ))
      .linearPosition(distance.mut_replace(
        mSwerveMods[3].getDrivePosition(),
        Meters
      ))
      .linearVelocity(velocity.mut_replace(
        mSwerveMods[3].getDriveVelocity(),
        MetersPerSecond
      ));
      },
      this));


  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID);
    gyro.setYaw(0);

     m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(),
                    getModulePositions(), getPose(), VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(1.0)));
    

    /*mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };*/

    swerveOdometry =
        new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

    SmartDashboard.putData("Field", field);


    try{
      RobotConfig config = RobotConfig.fromGUISettings();

    //Configure AutoBuilder
      AutoBuilder.configure(
        () -> getPose(),
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        (speed) -> pathPlannerRobotDrive(speed), 
        new PPHolonomicDriveController(
          new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD),
          new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD)
        ), 
        config, 
        () -> {
          //If we're blue alliance, DriverStation.Alliance.Red
          //If we're red alliance, DriverStation.Alliance.Blue

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this);
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and Configure AutoBuilder", e.getStackTrace());
    }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_driveRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_driveRoutine.dynamic(direction);
    }

  /** Drive the robot */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
    }
  }

   public void updateCameraPose() {
            boolean doRejectUpdate = false;
            int bestCamera;
            double leftAmbiguity = 0;
            double rightAmbiguity = 0;
    
          cameraPoses[0] = grabPose("limelight-front");
    
    
            if (cameraPoses[0] == null && cameraPoses[1] == null) {
                bestCamera = -1;
            } else if (cameraPoses[0] == null) {
                bestCamera = 1;
            } else if (cameraPoses[1] == null) {
                bestCamera = 0;
            } else {
                if (cameraPoses[0].tagCount > 0) {
                    leftAmbiguity = cameraPoses[0].rawFiducials[0].ambiguity;
                }
                if (cameraPoses[1].tagCount > 0) {
                    rightAmbiguity = cameraPoses[1].rawFiducials[0].ambiguity;
                }
                if (leftAmbiguity < rightAmbiguity) {
                    bestCamera = 0;
                } else {
                    bestCamera = 1;
                }
            }
            if (bestCamera == -1) {
                doRejectUpdate = true;
            }else {
                if(cameraPoses[bestCamera].tagCount<1){
                    doRejectUpdate=true;
                }
            }
            
            ;
            if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our angular velocity is greater
            {
                doRejectUpdate = true;
            }
            SmartDashboard.putBoolean("RejectUpdate", doRejectUpdate);
            if (!doRejectUpdate) {
                SmartDashboard.putNumber("bestcamera",bestCamera);
                SmartDashboard.putNumberArray("CameraPose", new double[] { cameraPoses[bestCamera].pose.getTranslation().getX(), cameraPoses[bestCamera].pose.getTranslation().getY(),
                    cameraPoses[bestCamera].pose.getRotation().getRadians() });
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999));
                m_poseEstimator.addVisionMeasurement(
                        cameraPoses[bestCamera].pose,
                        cameraPoses[bestCamera].timestampSeconds);
                
                        
                }
        }

         
    public double getRotationToHub() {
      targetYaw = Math.atan2(
          HubPose.getY() - getPose().getY(),
          HubPose.getX() - getPose().getX()
        );
        return Math.toDegrees(targetYaw);  
    }

      public double getDistanceToHub() {
        return Math.hypot(
          HubPose.getX() - getPose().getX(),
          HubPose.getY() - getPose().getY()
        );
    }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /*public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }*/

  public void resetPose(Pose2d pose) {
     m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

    public static Pose2d getPose() {
            return botPose2d;
    }

    public Rotation2d getGyroRotation2D() {
        return Rotation2d.fromDegrees(getCompassHeading());
    }

    public Pose3d getPose3d() {
        return new Pose3d(getPose());
    }

    public void setHeading(Rotation2d heading) {

        getCompassHeading();
        m_poseEstimator.resetPosition(getGyroRotation2D(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }


  public double getCompassHeading() {
        SmartDashboard.putNumber("CompassHeading", Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360.0);
    }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

   public Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(getCompassHeading());
  }

  public void zeroHeading() {
        gyro.setYaw(0);
        Pose2d current = getPose();
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(current.getTranslation(), new Rotation2d()));
  }
  
    public void resetGyroToAlliance() {
       gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 0 : 180);
    }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds state = Constants.Swerve.swerveKinematics.toChassisSpeeds(
      getModuleStates());
    return state;
  }

  public void pathPlannerRobotDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

    public PoseEstimate grabPose(String camera) {
        LimelightHelpers.SetRobotOrientation( "limelight-front", gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0,  0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        return mt2;
    }


  @Override
  public void periodic() {

    m_poseEstimator.update(getGyroYaw(), getModulePositions());
                        botPose2d = m_poseEstimator.getEstimatedPosition();
                        //SmartDashboard.putNumber("Rotation2D",getGyroRotation2D().getDegrees());
                        updateCameraPose();
    
    SmartDashboard.putNumberArray("bot Pose", new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()});
    SmartDashboard.putNumber("yaw", gyro.getRotation2d().getDegrees());
    


    /*swerveOdometry.update(getGyroYaw(), getModulePositions());

    field.setRobotPose(getPose());

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/States", getModuleStates());*/

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
  }
}
  
