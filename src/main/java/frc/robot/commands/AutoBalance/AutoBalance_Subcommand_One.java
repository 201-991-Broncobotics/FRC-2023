package frc.robot.commands.AutoBalance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance_Subcommand_One extends CommandBase {
    // Plan: move to intake position, then intake for some 
                // amount of time, then finish
    
    private Swerve swerve;

    public AutoBalance_Subcommand_One(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve); // means that other functions are not allowed to access it
    }

    @Override
    public void initialize() {
        swerve.setTargetHeading(((Math.abs(swerve.getYaw().getDegrees()) + 90) % 360 < 180) ? 0 : 180);
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getError()) < 2;
    }
}
