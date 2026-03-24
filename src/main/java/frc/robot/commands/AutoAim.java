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

public class AutoAim extends Command {

    private final Swerve s_Swerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private final PIDController rotPID = new PIDController(0.02, 0.0, 0.001);

    private static final double ANGLE_TOLERANCE = 1.5;

    public AutoAim(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        
        addRequirements(s_Swerve);

        rotPID.enableContinuousInput(-180, 180);
        rotPID.setTolerance(ANGLE_TOLERANCE);
    }

    @Override
    public void initialize() {
        rotPID.reset();
    }

    @Override
    public void execute() {

        double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        double rotationVal = rotPID.calculate(s_Swerve.gyro.getYaw().getValueAsDouble(), s_Swerve.getRotationToHub());
        rotationVal = MathUtil.clamp(rotationVal, -0.5, 0.5);

        if (rotPID.atSetpoint()) {
            rotationVal = 0;
        }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal,
        !robotCentricSup.getAsBoolean(),
        true
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0,0), 0, true, true);
    }
}