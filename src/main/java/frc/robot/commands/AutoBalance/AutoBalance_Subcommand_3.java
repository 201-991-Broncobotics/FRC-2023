package frc.robot.commands.autoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoBalanceConstants.*;

public class AutoBalance_Subcommand_3 extends CommandBase {
    
    private Swerve swerve;

    private double power = -drive_speed;
    private double target_power = -drive_speed;

    private double prev_time = 0;
    private double prev_pitch = 0;
    private double start_time = 0;
    private double delta_pitch;

    private double triple_ratio = ratio * ratio * ratio;
    
    public AutoBalance_Subcommand_3(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        prev_time = System.currentTimeMillis() / 1000.0;
        start_time = prev_time;
        prev_pitch = swerve.getPitch();
    }

    @Override
    public void execute() {
        double delta_time = System.currentTimeMillis() / 1000.0 - prev_time;
        prev_time = System.currentTimeMillis() / 1000.0;
        
        // Positive Pitch means angled down --> must drive backward

        boolean changeDir = false;

        delta_pitch = (swerve.getPitch() - prev_pitch) / delta_time;

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
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return (Math.abs(target_power) < drive_speed / triple_ratio && Math.abs(target_power) < drive_speed / triple_ratio && Math.abs(prev_pitch) < pitch_tolerance * 0.15 && Math.abs(delta_pitch) < 0.1 * min_deriv);
    }
}
