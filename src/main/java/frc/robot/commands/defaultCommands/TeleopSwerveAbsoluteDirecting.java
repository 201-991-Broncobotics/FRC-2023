package frc.robot.commands.defaultCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.Buttons.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerveAbsoluteDirecting extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier directionXSup;
    private DoubleSupplier directionYSup;
    private DoubleSupplier turnSup;
    private IntSupplier targetSup;
    private BooleanSupplier slowSup;

    public TeleopSwerveAbsoluteDirecting(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier directionXSup, DoubleSupplier directionYSup, IntSupplier targetSup, DoubleSupplier turnSup, BooleanSupplier slowSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.directionXSup = directionXSup;
        this.directionYSup = directionYSup;
        this.targetSup = targetSup;
        this.turnSup = turnSup;
        this.slowSup = slowSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = signedPower(translationSup.getAsDouble());
        double strafeVal = signedPower(strafeSup.getAsDouble());
        double turnVal = signedPower(turnSup.getAsDouble()) * turn_sensitivity;

        double slowVal = 1;

        if (slowSup.getAsBoolean()) slowVal = slow;
        
        double x_dir = directionXSup.getAsDouble();
        double y_dir = directionYSup.getAsDouble();

        if (x_dir * x_dir + y_dir * y_dir < joystick_deadzone * joystick_deadzone) {
            x_dir = 0;
            y_dir = 0;
        }

        if (turnVal == 0) {
            if (x_dir != 0 || y_dir != 0) {
                double target_heading = s_Swerve.getYaw().getDegrees();

                if (x_dir == 0) {
                    if (y_dir > 0) {
                        target_heading = 0;
                    } else {
                        target_heading = 180;
                    }
                } else if (x_dir < 0) {
                    target_heading = Math.atan(y_dir / x_dir) * 180.0 / Math.PI + 90;
                } else {
                    target_heading = Math.atan(y_dir / x_dir) * 180.0 / Math.PI - 90;
                }

                double error_in_percent = Math.max(Math.min(Swerve.normalizeAngle(target_heading - s_Swerve.getYaw().getDegrees()) / maximum_error, 1), -1);
                int multiplier = 1;
                if (error_in_percent < 0) {
                    error_in_percent = 0 - error_in_percent;
                    multiplier = -1;
                }
                turnVal = Math.pow(error_in_percent, exponent) * maximum_power * multiplier;

            } else if (targetSup.getAsInt() % 90 == 0) { // setTargetHeading on purpose
                s_Swerve.setTargetHeading(targetSup.getAsInt());
            }
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerve.maxSpeed).times(slowVal), 
            turnVal * Constants.BaseFalconSwerve.maxAngularVelocity, 
            true, 
            true
        );
    }
}