package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
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
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.joystick_deadzone);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.joystick_deadzone);
        double turnVal = MathUtil.applyDeadband(turnSup.getAsDouble(), Constants.joystick_deadzone);

        double slowVal = 1;

        if (slowSup.getAsBoolean()) slowVal = slow;
        
        double x_dir = directionXSup.getAsDouble();
        double y_dir = directionYSup.getAsDouble();

        if (x_dir * x_dir + y_dir * y_dir < Constants.joystick_deadzone * Constants.joystick_deadzone) {
            x_dir = 0;
            y_dir = 0;
        }

        if (turnVal == 0) {
            if (x_dir != 0 || y_dir != 0) {
                if (x_dir == 0) {
                    if (y_dir > 0) {
                        s_Swerve.setTargetHeading(0);
                    } else {
                        s_Swerve.setTargetHeading(180);
                    }
                } else if (x_dir < 0) {
                    s_Swerve.setTargetHeading(Math.atan(y_dir / x_dir) * 180.0 / Math.PI + 90);
                } else {
                    s_Swerve.setTargetHeading(Math.atan(y_dir / x_dir) * 180.0 / Math.PI - 90);
                }
            } else if (targetSup.getAsInt() % 90 == 0) {
                s_Swerve.setTargetHeading(targetSup.getAsInt());
            }
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerve.maxSpeed).times(slowVal), 
            turnVal * Constants.BaseFalconSwerve.maxAngularVelocity * turn_sensitivity, 
            true, 
            true
        );
    }
}