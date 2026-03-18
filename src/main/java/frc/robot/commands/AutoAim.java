package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AutoAim extends Command {

    private final Swerve s_Swerve;
    private final PIDController rotationPID;
    private final Vision m_vision;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public AutoAim(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, Vision m_vision) {
        this.s_Swerve = s_Swerve;
        this.m_vision = m_vision;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        
        addRequirements(s_Swerve);

        rotationPID = new PIDController(0.021, 0, 0.000);
        rotationPID.setSetpoint(0);
        rotationPID.setTolerance(1.0);
        rotationPID.enableContinuousInput(-180, 180);
    }

      @Override
        public void initialize() {  
         rotationPID.reset();
        }

    @Override
    public void execute() {

        double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double[] targetPose = LimelightHelpers.getCameraPose_TargetSpace("limelight");
        /*if (targetPose.length < 6) {
            swerve.drive(new Translation2d(0,0), 0, true, true);
            return;
        }*/ 

        double kHypercubed = 0;

        if(LimelightHelpers.getTV("limelight") == true
        
        && LimelightHelpers.getFiducialID("limelight") == 5
        || LimelightHelpers.getFiducialID("limelight") == 10
        || LimelightHelpers.getFiducialID("limelight") == 2
        || LimelightHelpers.getFiducialID("limelight") == 8
        || LimelightHelpers.getFiducialID("limelight") == 9
        || LimelightHelpers.getFiducialID("limelight") == 11
        ){

        if(LimelightHelpers.getFiducialID("limelight") == 8 && LimelightHelpers.getFiducialID("limelight") == 5){
            kHypercubed = 1.0;
        }

         if(LimelightHelpers.getFiducialID("limelight") == 11 && LimelightHelpers.getFiducialID("limelight") == 2){ 
            kHypercubed = -1.0;
        }

        if(LimelightHelpers.getFiducialID("limelight") == 10 && m_vision.getDistanceInches() > 50.0){
            kHypercubed = -7.0;
        } else {
            kHypercubed = 0.0;
        }

        double tx = LimelightHelpers.getTX("limelight"); // degrees
        rotationVal = MathUtil.clamp(
            rotationPID.calculate(tx + kHypercubed),
            -0.3, 0.3
            );
    }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0,0), 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }
}