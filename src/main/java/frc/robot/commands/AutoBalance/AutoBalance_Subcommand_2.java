package frc.robot.commands.autoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoBalanceConstants.*;

public class AutoBalance_Subcommand_2 extends CommandBase {
    
    private Swerve swerve;
    private double stop_time = 0;

    public AutoBalance_Subcommand_2(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        stop_time = Timer.getFPGATimestamp() + mt2;
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(-drive_speed_get_on, 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        if (Math.abs(swerve.getPitch()) < (pitch_tolerance * 1.2)) stop_time = Timer.getFPGATimestamp() + mt2;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > stop_time;
    }
}
