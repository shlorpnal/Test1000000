package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSub extends SubsystemBase {

  private TalonFX hoodMotor;

  private PIDController pidController =
      new PIDController(
          Constants.HoodConstants.kP, Constants.HoodConstants.kI, Constants.HoodConstants.kD);

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  //// gear ratio
  public HoodSub() {
    hoodMotor = new TalonFX(Constants.HoodConstants.hoodID);
    TalonFXConfigurator hoodConfig = hoodMotor.getConfigurator();
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs magicConfigs = new MotionMagicConfigs();

    outputConfig.NeutralMode = NeutralModeValue.Brake;
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;

    softLimitConfig.ForwardSoftLimitThreshold = 2.49;
    softLimitConfig.ForwardSoftLimitEnable = true;
    softLimitConfig.ReverseSoftLimitThreshold = 0.2;
    softLimitConfig.ReverseSoftLimitEnable = true;

    currentConfig.StatorCurrentLimit = 40;
    currentConfig.StatorCurrentLimitEnable = true;

    pidConfig.kP = Constants.HoodConstants.kP;
    pidConfig.kI = Constants.HoodConstants.kI;
    pidConfig.kD = Constants.HoodConstants.kD;
    pidConfig.kG = Constants.HoodConstants.kG;
    pidConfig.kS = Constants.HoodConstants.kS;
    pidConfig.kV = Constants.HoodConstants.kV;

    magicConfigs.MotionMagicCruiseVelocity = 50; // max target velocity in rps
    magicConfigs.MotionMagicAcceleration = 100; // target acceleration to velocity (0.5 secs)
    magicConfigs.MotionMagicJerk = 1000; // target jerk (0.1 secs)

    hoodConfig.apply(outputConfig);
    hoodConfig.apply(softLimitConfig);
    hoodConfig.apply(currentConfig);
    hoodConfig.apply(pidConfig);
    hoodConfig.apply(magicConfigs);

    hoodAngleMap.put(31.375, 0.0);
    hoodAngleMap.put(81.4, 8.0);
    hoodAngleMap.put(140.0, 15.0);
    hoodAngleMap.put(178.7, 18.5);
    hoodAngleMap.put(191.7, 22.0);
    hoodAngleMap.put(211.9072, 28.0);

    // Encoders
    configEncoders();
  }

  private void configEncoders() {
    hoodMotor.setPosition(0);
  }

  public void setAngle(double deg) {

    double setPoint = (deg / Constants.HoodConstants.DEGREES_PER_ROT_HOOD) * 9.0;

    /*MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    hoodMotor.setControl(request.withPosition(setPoint));*/

    double pidOutput = pidController.calculate(getPosition(), setPoint);

    double output = pidOutput;

    SmartDashboard.putNumber("hoodRotSetpoint", setPoint);

    if (Math.abs(pidOutput) > 0.01) {
      output += Math.copySign(Constants.HoodConstants.kG, pidOutput);
    }

    output = MathUtil.clamp(output, -0.08, 0.08);

    SmartDashboard.putNumber("hoodOutput", output);
    hoodMotor.set(output);
  }

  public boolean isAtSetpoint() {

    return pidController.atSetpoint();
  }

  public double getInterpolatedAngle(double distance) {
    return hoodAngleMap.get(distance);
  }

  public void setAngleFromDistance(double distance) {
    double angle = hoodAngleMap.get(distance);
    setAngle(angle);
  }

  public void manualHood(double volts) {
    hoodMotor.setVoltage(volts);
  }

  public double getPosition() {
    double hoodPos = hoodMotor.getPosition().getValueAsDouble();
    return hoodPos;
  }

  public double getAngleHood() {
    double hoodPos = hoodMotor.getPosition().getValueAsDouble();
    return ((hoodPos * Constants.HoodConstants.DEGREES_PER_ROT_HOOD) / 9.0);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hoodPositionDeg", getAngleHood());
    SmartDashboard.putNumber("hoodPosition", getPosition());
    SmartDashboard.putNumber("hoodSetpoint", 25);
  }
}
