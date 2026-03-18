package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveAFXConfig = new TalonFXConfiguration();
   public TalonFXConfiguration swerveDriveBFXConfig = new TalonFXConfiguration();
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  public CTREConfigs() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
    swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.angleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.angleCurrentThreshold;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime =
        Constants.Swerve.angleCurrentThresholdTime;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
    swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
    swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
    swerveAngleFXConfig.Slot0.kS = Constants.Swerve.angleKS;
    swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKV;
    swerveAngleFXConfig.Slot0.kA = Constants.Swerve.angleKA;

    /** Swerve Drive Motor Configuration */
    
    //A
    /* Motor Inverts and Neutral Mode */
    swerveDriveAFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorAInvert;
    swerveDriveAFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveAFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

    /* Current Limiting */
    swerveDriveAFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.driveEnableCurrentLimit;
    swerveDriveAFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
    swerveDriveAFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentThreshold;
    swerveDriveAFXConfig.CurrentLimits.SupplyCurrentLowerTime =
        Constants.Swerve.driveCurrentThresholdTime;

    /* PID Config */
    swerveDriveAFXConfig.Slot0.kP = Constants.Swerve.driveKP;
    swerveDriveAFXConfig.Slot0.kI = Constants.Swerve.driveKI;
    swerveDriveAFXConfig.Slot0.kD = Constants.Swerve.driveKD;
    swerveDriveAFXConfig.Slot0.kS = Constants.Swerve.driveKS;
    swerveDriveAFXConfig.Slot0.kV = Constants.Swerve.driveKV;
    swerveDriveAFXConfig.Slot0.kA = Constants.Swerve.driveKA;

    /* Open and Closed Loop Ramping */
    swerveDriveAFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
    swerveDriveAFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

    swerveDriveAFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;
    swerveDriveAFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;

    //B--------------------------------------------------------------------------------------
    /* Motor Inverts and Neutral Mode */
    swerveDriveBFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorBInvert;
    swerveDriveBFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveBFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

    /* Current Limiting */
    swerveDriveBFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.driveEnableCurrentLimit;
    swerveDriveBFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
    swerveDriveBFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentThreshold;
    swerveDriveBFXConfig.CurrentLimits.SupplyCurrentLowerTime =
        Constants.Swerve.driveCurrentThresholdTime;

    /* PID Config */
    swerveDriveBFXConfig.Slot0.kP = Constants.Swerve.driveKP;
    swerveDriveBFXConfig.Slot0.kI = Constants.Swerve.driveKI;
    swerveDriveBFXConfig.Slot0.kD = Constants.Swerve.driveKD;
    swerveDriveBFXConfig.Slot0.kS = Constants.Swerve.driveKS;
    swerveDriveBFXConfig.Slot0.kV = Constants.Swerve.driveKV;
    swerveDriveBFXConfig.Slot0.kA = Constants.Swerve.driveKA;

    /* Open and Closed Loop Ramping */
    swerveDriveBFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
    swerveDriveBFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

    swerveDriveBFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;
    swerveDriveBFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.closedLoopRamp;
  }
}
