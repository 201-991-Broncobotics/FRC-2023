package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.SwerveConstants.*;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier targetSup;

    private static double target_heading = 0;
    private double last_time;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier targetSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetSup = targetSup;

        last_time = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = getTurning(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerve.maxSpeed), 
            rotationVal * Constants.BaseFalconSwerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180) { angle -= 360; }
        while (angle <= -180) { angle += 360; }

        return angle;
    }

    public double getTurning(double turning) {
        double current_angle = s_Swerve.getYaw().getDegrees();
        if (Math.abs(turning) > joystick_deadzone) {
            target_heading = current_angle;
            last_time = System.currentTimeMillis();
            return turning;
        }
        if (System.currentTimeMillis() - last_time < calibration_time * 1000) {
            target_heading = current_angle;
            return 0;
        }
        if (targetSup.getAsInt() % 90 == 0) {
            target_heading = normalizeAngle(targetSup.getAsInt() - current_angle) + current_angle;
        }
        double error_in_percent = Math.max(Math.min((target_heading - current_angle) / maximum_error, 1), -1);
        if (error_in_percent == 0) {
            return 0;
        }
        int multiplier = 1;
        if (error_in_percent < 0) {
            error_in_percent = 0 - error_in_percent;
            multiplier = -1;
        }
        return Math.pow(error_in_percent, exponent) * maximum_power * multiplier;
    }

    public static void resetGyro() {
        target_heading = 0; // should be 0 but just to be safe :)
    }
}