package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSTalonFXSwerveConstants {
  public final double wheelDiameter;
  public final double wheelCircumference;
  public final double angleGearRatio;
  public final double driveGearRatio;
  public final double angleKP;
  public final double angleKI;
  public final double angleKD;
  public final InvertedValue driveMotorAInvert;
  public final InvertedValue driveMotorBInvert;
  public final InvertedValue angleMotorInvert;
  public final SensorDirectionValue cancoderInvert;

  public COTSTalonFXSwerveConstants(
      double wheelDiameter,
      double angleGearRatio,
      double driveGearRatio,
      double angleKP,
      double angleKI,
      double angleKD,
      InvertedValue driveMotorAInvert,
      InvertedValue driveMotorBInvert,
      InvertedValue angleMotorInvert,
      SensorDirectionValue cancoderInvert) {
    this.wheelDiameter = wheelDiameter;
    this.wheelCircumference = wheelDiameter * Math.PI;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.angleKP = angleKP;
    this.angleKI = angleKI;
    this.angleKD = angleKD;
    this.driveMotorAInvert = driveMotorAInvert;
    this.driveMotorBInvert = driveMotorBInvert;
    this.angleMotorInvert = angleMotorInvert;
    this.cancoderInvert = cancoderInvert;
  }

  /** Swerve Drive Specialities */
  public static final class SDS {
    /** Swerve Drive Specialties - MK3 Module */
  
    public static final class MK5n {
      // Mk5n (Kraken x60) custom coded
      public static final COTSTalonFXSwerveConstants KrakenX60_MK5n(double driveGearRatio) {
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((287.0 / 11.0) / 1.0);

        double angleKP = 25.0;
        double angleKI = 0.0;
        double angleKD = 0.5;

        InvertedValue driveMotorAInvert = InvertedValue.CounterClockwise_Positive;
        InvertedValue driveMotorBInvert = InvertedValue.Clockwise_Positive;
        InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSTalonFXSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            driveMotorAInvert,
            driveMotorBInvert,
            angleMotorInvert,
            cancoderInvert);
      }

      public static final class driveRatios {
        /** SDS MK4i - (8.14 : 1) */
        public static final double L1 = (8.10 / 1.0);
        /** SDS MK4i - (6.75 : 1) */
        public static final double L2 = (6.75 / 1.0);
        /** SDS MK4i - (6.12 : 1) */
        public static final double L3 = (5.2734375 / 1.0);
      }
    }
  }
}
  

