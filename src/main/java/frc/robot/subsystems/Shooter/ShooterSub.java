package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
  // L -> Leader / F -> Follower
  private SparkMax shooterLMotor;
  private SparkMax shooterFMotor;

  private RelativeEncoder shooterLEncoder;
  private RelativeEncoder shooterFEncoder;

  private SparkClosedLoopController Lpid;
  private SparkClosedLoopController Fpid;

  private final SimpleMotorFeedforward LsFF =
      new SimpleMotorFeedforward(ShooterConstants.LkS, ShooterConstants.LkV, ShooterConstants.LkA);

  private final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();

  private final MutVoltage flywheelVoltage = Volts.mutable(80);

  private final MutAngle position = Radians.mutable(0);

  private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

  private final SysIdRoutine flywheelRoutine;

  public ShooterSub() {
    // Leader
    shooterLMotor = new SparkMax(Constants.ShooterConstants.shooterLID, MotorType.kBrushless);
    SparkMaxConfig shooterLConfig = new SparkMaxConfig();
    Lpid = shooterLMotor.getClosedLoopController();

    shooterLConfig.inverted(true).idleMode(IdleMode.kCoast);
    shooterLConfig.smartCurrentLimit(60).voltageCompensation(12);
    shooterLConfig
        .closedLoop
        .p(Constants.ShooterConstants.LkP)
        .i(Constants.ShooterConstants.LkI)
        .d(Constants.ShooterConstants.LkD);
    shooterLConfig.encoder.velocityConversionFactor(Constants.ShooterConstants.gearRatio);

    shooterLMotor.configure(
        shooterLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Follower
    shooterFMotor = new SparkMax(Constants.ShooterConstants.shooterFID, MotorType.kBrushless);
    SparkMaxConfig shooterFConfig = new SparkMaxConfig();

    shooterFConfig.idleMode(IdleMode.kCoast).follow(shooterLMotor, true);
    shooterFConfig.smartCurrentLimit(60).voltageCompensation(12);
    shooterFConfig
        .closedLoop
        .p(Constants.ShooterConstants.LkP)
        .i(Constants.ShooterConstants.LkI)
        .d(Constants.ShooterConstants.LkD);
    Fpid = shooterFMotor.getClosedLoopController();
    shooterFConfig.encoder.velocityConversionFactor(Constants.ShooterConstants.gearRatio);
    shooterFMotor.configure(
        shooterFConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Treemap
    shooterRPMMap.put(31.375, 2500.0);
    shooterRPMMap.put(81.4, 3000.0);
    shooterRPMMap.put(140.0, 2900.0);
    shooterRPMMap.put(178.7, 2930.0);
    shooterRPMMap.put(191.7, 2960.0);
    shooterRPMMap.put(211.9072, 3000.0);
    //shooterRPMMap.put(220.9072, 6811.0);

    flywheelRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Voltage) -> {
                  shooterLMotor.set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
                  shooterFMotor.set(Voltage.in(Volts) / RobotController.getBatteryVoltage());
                },
                log -> {
                  log.motor("flywheelL")
                      .voltage(
                          flywheelVoltage.mut_replace(
                              shooterLMotor.getAppliedOutput() * shooterLMotor.getBusVoltage(),
                              Volts))
                      .angularPosition(
                          position.mut_replace(shooterLEncoder.getPosition(), Rotations))
                      .angularVelocity(
                          velocity.mut_replace(
                              shooterLEncoder.getVelocity() / 60, RotationsPerSecond));
                  log.motor("flywheelF")
                      .voltage(
                          flywheelVoltage.mut_replace(
                              shooterFMotor.getAppliedOutput() * shooterFMotor.getBusVoltage(),
                              Volts))
                      .angularPosition(
                          position.mut_replace(shooterFEncoder.getPosition(), Rotations))
                      .angularVelocity(
                          velocity.mut_replace(
                              shooterFEncoder.getVelocity() / 60, RotationsPerSecond));
                },
                this));
    // Encoders
    configEncoders();
  }

  private void configEncoders() {
    shooterLEncoder = shooterLMotor.getEncoder();
    shooterLEncoder.setPosition(0);

    shooterFEncoder = shooterFMotor.getEncoder();
    shooterFEncoder.setPosition(0);
  }

  public void setspeed(double speed) {
    shooterLMotor.set(-speed);
  }

  public void setRPM(double RPM) {
    double RPS = RPM / 60;

    double ffVolts = LsFF.calculate(RPS);
    // double arbFF = ffVolts / 12.0;

    Lpid.setSetpoint(
        RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
  }

  public double getInterpolatedRPM(double distance) {
    return shooterRPMMap.get(distance);
  }

  public void setRPMFromDistance(double distance) {
    double rpm = shooterRPMMap.get(distance);
    setRPM(rpm);
  }

  public double getRPM() {
    return shooterLEncoder.getVelocity();
  }

  public void stop() {
    shooterLMotor.stopMotor();
    shooterFMotor.stopMotor();
  }

  public void voltageControl(double voltage) {
    shooterLMotor.setVoltage(voltage);
  }

  public Command sysIdQuasi(SysIdRoutine.Direction direction) {
    return flywheelRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return flywheelRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("flywheelL velocity", shooterLEncoder.getVelocity());
    SmartDashboard.putNumber("flywheelF velcotiy", shooterFEncoder.getVelocity());
    SmartDashboard.putNumber("setpoint", 1500);

    SmartDashboard.putNumber("output", shooterLMotor.getAppliedOutput());

    // SmartDashboard.putBoolean("AtRPM", atRPM(getRPM()));

    // double RPS = 1500 / 60;
    // double radPerSec = RPS * 2 * Math.PI;
    // SmartDashboard.putNumber("Lff", LsFF.calculate(radPerSec));
    // SmartDashboard.putNumber("Fff", FsFF.calculate(radPerSec));
  }
}
