package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter.HoodSub;
import frc.robot.subsystems.Shooter.ShooterSub;
import frc.robot.subsystems.hopper.SerializerSub;
// import frc.robot.util.ShotCalc;

public class ShooterStructure extends SubsystemBase {

  private ShooterSub shooter;
  private HoodSub hood;
  private Swerve swerve;
  private SerializerSub serializer;

  private double targetRPM = 0;
  private double targetAngle = 0;

  public enum ShooterState {
    IDLE,
    SPINNINGUP,
    READY,
    SHOOTING
  }

  private ShooterState state = ShooterState.IDLE;

  public ShooterStructure(
      ShooterSub shooter, HoodSub hood, Swerve swerve, SerializerSub serializer) {
    this.shooter = shooter;
    this.hood = hood;
    this.swerve = swerve;
    this.serializer = serializer;
  }

  public void requestIDLE() {
    state = ShooterState.IDLE;
  }

  public void requestVisionShot() {
    state = ShooterState.SPINNINGUP;
  }

  public boolean isReady() {
    return state == ShooterState.READY;
  }

  public boolean atRPM(double distance) {
    targetRPM = shooter.getInterpolatedRPM(distance);

    if (shooter.getRPM() - targetRPM < Constants.ShooterConstants.RPM_TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    switch (state) {
      case IDLE -> handleIdle();
      case SPINNINGUP -> handleSpinUp();
      case READY -> handleReady();
      case SHOOTING -> handleShoot();
    }

    SmartDashboard.putBoolean("Shooter Ready", isReady());
    SmartDashboard.putBoolean("AtRPM?", atRPM(swerve.getDistanceToHub()));
    SmartDashboard.putNumber(
        "flywheelSetpoint", shooter.getInterpolatedRPM(swerve.getDistanceToHub()));
    SmartDashboard.putNumber("hoodSetpoint", hood.getInterpolatedAngle(swerve.getDistanceToHub()));
  }

  private void handleIdle() {
    //hood.setAngle(3);
    hood.setAngleFromDistance(35.0);
    shooter.stop();
    serializer.stop();
    SmartDashboard.putNumber("hood25", 25);
  }

  private void handleSpinUp() {
    double distance = swerve.getDistanceToHub();
    if (!LimelightHelpers.getTV("limelight")) {
      shooter.setRPMFromDistance(80.0);
      hood.setAngleFromDistance(40.0);
    } else {

    shooter.setRPMFromDistance(distance);
    hood.setAngleFromDistance(distance);
    }

    if (atRPM(distance)) {
      state = ShooterState.READY;
    }
  }

  private void handleReady() {
    double distance = swerve.getDistanceToHub();
    if (!LimelightHelpers.getTV("limelight")) {
      shooter.setRPMFromDistance(80.0);
      hood.setAngleFromDistance(40.0);
    } else {

    shooter.setRPMFromDistance(distance);
    hood.setAngle(distance);

    serializer.stop();
    }
  }

  private void handleShoot() {
    double distance = swerve.getDistanceToHub();

    if (!LimelightHelpers.getTV("limelight")) {
      shooter.setRPMFromDistance(80.0);
      hood.setAngleFromDistance(40.0);
    } else {

    shooter.setRPMFromDistance(distance);
    hood.setAngle(distance);
    }
    
      if (atRPM(80.0)){
        serializer.autofeed();
      } else {
      serializer.stop();
      }
  }

  public void shoot() {
    if (state == ShooterState.READY) {
      state = ShooterState.SHOOTING;
    }
  }

  public void stopShooting() {
    if (state == ShooterState.SHOOTING) {
      state = ShooterState.IDLE;
    }
  }
}
