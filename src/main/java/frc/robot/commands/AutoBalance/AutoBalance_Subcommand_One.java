package frc.robot.commands.AutoBalance;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance_Subcommand_One extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Swerve swerve;

    private boolean isFirstAction = true;

    BooleanSupplier exitSup;

    public AutoBalance_Subcommand_One(Swerve swerve, BooleanSupplier exitSup) {
        this.swerve = swerve;
        addRequirements(swerve); // means that other functions are not allowed to access it
        
        isFirstAction = true;
        this.exitSup = exitSup;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            swerve.setTargetHeading(180);
            isFirstAction = false;
        }
        swerve.drive(new Translation2d(), 0, false, true);
    }
    
    @Override
    public boolean isFinished() {
        swerve.setTargetHeading(180);
        if ((Math.abs(swerve.getError()) > 2) && !(exitSup.getAsBoolean())) {
            return false;
        } else {
            swerve.changeHeading(0);
            swerve.drive(new Translation2d(), 0, true, false); // brake
            return true;
        }
    }
}
