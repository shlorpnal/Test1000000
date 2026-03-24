// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  public TalonFX climbMotor;
  public double climbPos;
  boolean isAtSetPoint;

  public enum climbStates {
    OUT,
    IN
  }

  public climbStates climbState;

  /** Creates a new CLimb. */
  public Climb() {
    climbMotor = new TalonFX(Constants.ClimbConstants.climbID);
    climbPos = climbMotor.getPosition().getValueAsDouble();
    TalonFXConfigurator config = climbMotor.getConfigurator();
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    SoftwareLimitSwitchConfigs softLimitConfig = new SoftwareLimitSwitchConfigs();
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();

    outputConfig.NeutralMode = NeutralModeValue.Brake;

    softLimitConfig.ForwardSoftLimitThreshold = 511.096;
    softLimitConfig.ForwardSoftLimitEnable = false;
    softLimitConfig.ReverseSoftLimitThreshold = -1;
    softLimitConfig.ReverseSoftLimitEnable = false;

    currentConfig.StatorCurrentLimit = 20;
    currentConfig.StatorCurrentLimitEnable = true;

    pidConfig.kS = Constants.ClimbConstants.kS;
    pidConfig.kV = Constants.ClimbConstants.kV;
    pidConfig.kP = Constants.ClimbConstants.kP;
    pidConfig.kI = Constants.ClimbConstants.kI;
    pidConfig.kD = Constants.ClimbConstants.kD;

    motionMagicConfig.MotionMagicCruiseVelocity = 120; // target max velocity of 30 rps
    motionMagicConfig.MotionMagicAcceleration =
        240; // target acceleration to max velocity of 0.5 secs
    motionMagicConfig.MotionMagicJerk = 2400; // target jerk of 0.1 secs

    config.apply(outputConfig);
    config.apply(softLimitConfig);
    config.apply(currentConfig);
    config.apply(pidConfig);
    config.apply(motionMagicConfig);

    configEncoders();
  }

  private void configEncoders() {
    climbMotor.setPosition(0);
  }

  public void move(double speed) {
    climbMotor.set(speed);
  }

  public void stop() {
    climbMotor.stopMotor();
  }

  public void setPosition(double setpoint) {
    MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    climbMotor.setControl(request.withPosition(setpoint));
  }

  public void resetClimbEncoder() {
    climbMotor.setPosition(0);
  }

  public climbStates getState() {
    return climbState;
  }

  public void setState(climbStates state) {
    climbState = state;
  }

  public double getPosition() {
    double climbPos = climbMotor.getPosition().getValueAsDouble();
    return climbPos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climbPosition", getPosition());
    SmartDashboard.putNumber("climbSetpoint", 200);

    /*switch (climbState) {
      case OUT:
        setPosition(200); // Placeholder
        break;

      case IN:
        setPosition(0); // Placeholder
        break;
    }*/
  }
}
