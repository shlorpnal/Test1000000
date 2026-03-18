package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
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
        this::getPose, 
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

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void zeroHeading() {
    /*swerveOdometry.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));*/

        gyro.setYaw(0);
        Pose2d current = getPose();
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(current.getTranslation(), new Rotation2d()));
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

  public boolean isClimbing(double roll) {
    double setpoint = 0;
    if (roll >= setpoint){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {

     /*try {
      swerveOdometry.update(getGyroYaw(), getModulePositions());
    } catch (Exception e) {
      // Log it and continue
      System.out.println("Warning: Failed to update odometry: " + e.getMessage());
    }*/

    SmartDashboard.getBoolean("isClimbing?", isClimbing(gyro.getRoll().getValueAsDouble()));

    swerveOdometry.update(getGyroYaw(), getModulePositions());

    field.setRobotPose(getPose());

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/States", getModuleStates());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
