// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  // CANID is a placeholder, needs to be set
  // creates an empty sparkmax object
  private final SparkMax motor;

  public IntakeSub() {
    // sets the value of the sparkmax object; canid and configures motor type
    motor = new SparkMax(Constants.SlapdownConstants.intakeID, MotorType.kBrushless);
    // creates a config object
    SparkMaxConfig configuration = new SparkMaxConfig();

    configuration.inverted(false).idleMode(IdleMode.kCoast);
    configuration.smartCurrentLimit(20).voltageCompensation(8);

    motor.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void set(double speed) {
    motor.set(speed);
  }

  public void autofeed() {
    motor.set(-.9);
  }

  public void stop() {
    motor.set(0);
  }
}
