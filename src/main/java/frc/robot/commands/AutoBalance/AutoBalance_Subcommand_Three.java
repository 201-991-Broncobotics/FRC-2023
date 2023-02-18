package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoBalanceConstants.*;

public class AutoBalance_Subcommand_Three extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Swerve swerve;

    private boolean isFirstAction = true;

    private double power = -drive_speed;
    private double target_power = -drive_speed;

    private double prev_time = 0;

    private double prev_pitch = 0;

    private double start_time = 0;

    public AutoBalance_Subcommand_Three(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve); // means that other functions are not allowed to access it
        
        isFirstAction = true;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            prev_time = System.currentTimeMillis() / 1000.0;
            start_time = prev_time;
            prev_pitch = swerve.getPitch();
            isFirstAction = false;
        }

        double delta_time = System.currentTimeMillis() / 1000.0 - prev_time;
        prev_time = System.currentTimeMillis() / 1000.0;
        
        // Positive Pitch means angled down --> must drive backward

        boolean changeDir = false;

        double delta_pitch = (swerve.getPitch() - prev_pitch) / delta_time;

        if ((prev_time - start_time > min_time) && (Math.abs(delta_pitch) > min_deriv) && (Math.abs(swerve.getPitch()) < pitch_tolerance) && (delta_pitch * target_power > 0) && (delta_pitch * swerve.getPitch() < 0)) changeDir = true;
        SmartDashboard.putNumber("deriv", delta_pitch);
            // we change dir if delta pitch is decreasing and we have negative power

        if (changeDir) {
            target_power /= -ratio;
        }

        prev_pitch = swerve.getPitch();

        if (power > target_power) {
            power = Math.max(power - delta_time * max_linear_acceleration, target_power);
        } else if (power < target_power) {
            power = Math.min(power + delta_time * max_linear_acceleration, target_power);
        }
            
        swerve.drive(new Translation2d(power, 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
    }
    
    @Override
    public boolean isFinished() {
        return false; /*
        if (Math.abs(swerve.getPitch()) > pitch_tolerance) {
            return false;
        } else {
            swerve.changeHeading(0);
            swerve.drive(new Translation2d(), 0, true, false); // brake
            return true;
        } */ //lol
    }
}
