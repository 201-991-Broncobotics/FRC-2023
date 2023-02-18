package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutoBalanceConstants.*;

public class AutoBalance_Subcommand_Three extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Swerve swerve;

    private boolean isFirstAction = true;

    public AutoBalance_Subcommand_Three(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve); // means that other functions are not allowed to access it
        
        isFirstAction = true;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            // we don't need to do anything on our first action
            isFirstAction = false;
        }
        
        // Positive Pitch means angled down --> must drive backward

        if (swerve.getPitch() > 0) {
            swerve.drive(new Translation2d(-drive_speed, 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        } else if (swerve.getPitch() < 0) {
            swerve.drive(new Translation2d(drive_speed, 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
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
