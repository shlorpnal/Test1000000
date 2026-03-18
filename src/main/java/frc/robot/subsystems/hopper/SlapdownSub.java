// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SlapdownSub extends SubsystemBase {

  public SparkMax slapdownMotor;
  public SparkMax intakeMotor;
  public RelativeEncoder slapdownEncoder;
  public RelativeEncoder intakeEncoder;
  public PIDController slapdownPID;
  boolean isAtSetPoint;

  public static double slapdownPosition = 0;

  public enum slapdownStates {
    UP,
    DOWN
  }

  public slapdownStates slapdownState;

  public SlapdownSub() {
    // slapdownState = slapdownStates.UP;
    slapdownMotor = new SparkMax(Constants.SlapdownConstants.slapdownID, MotorType.kBrushless);
    slapdownEncoder = slapdownMotor.getEncoder();
    slapdownPID =
        new PIDController(
            Constants.SlapdownConstants.kP,
            Constants.SlapdownConstants.kI,
            Constants.SlapdownConstants.kD);
    SparkMaxConfig slapdownConfig = new SparkMaxConfig();

    slapdownPID.setTolerance(0.01);
    slapdownConfig.inverted(false).idleMode(IdleMode.kBrake);

    slapdownConfig
        .softLimit
        .forwardSoftLimit(40) // Placeholder
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-0.1) // Placeholder
        .reverseSoftLimitEnabled(false);
    slapdownConfig
        .smartCurrentLimit(20) // Placholder
        .voltageCompensation(8); // Placeholder

    slapdownMotor.configure(
        slapdownConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    slapdownEncoder.setPosition(0);
  }

  public void slapdownMove(double speed) {
    slapdownMotor.set(speed);
  }

  public void stop() {
    slapdownMotor.stopMotor();
  }

  public void slapdownPosition(double setPoint) {
    double speedOutput =
        MathUtil.clamp(slapdownPID.calculate(slapdownEncoder.getPosition(), setPoint), -0.8, 0.8);
    slapdownMove(-speedOutput);
  }

  public void resetSlapdownEncoder() {
    slapdownEncoder.setPosition(0);
  }

  public slapdownStates getState() {
    return slapdownState;
  }

  public void setState(slapdownStates state) {
    slapdownState = state;
  }
  
  public void requestUP() {
    if (slapdownState == slapdownStates.DOWN) {
      setState(slapdownStates.UP);
    }
  }

  public void requestDown() {
    if (slapdownState == slapdownStates.UP) {
      setState(slapdownStates.DOWN);
    }
  }

  public boolean slapdownUP() {
    return slapdownState == slapdownStates.UP;
  }

  public boolean slapdownDOWN() {
    return slapdownState == slapdownStates.DOWN;
  }

  public boolean isAtSetPoint() {
    return slapdownPID.atSetpoint();
  }

  public void slapdownManual(double speed) {
    slapdownMotor.set(speed);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Slapdown position", slapdownEncoder.getPosition());

    /*switch (slapdownState) {
      case UP:
        slapdownPosition(0); // Placeholder
        break;

      case DOWN:
        slapdownPosition(1.142857432365417); // Placeholder
        break;
    }*/
  }
}
