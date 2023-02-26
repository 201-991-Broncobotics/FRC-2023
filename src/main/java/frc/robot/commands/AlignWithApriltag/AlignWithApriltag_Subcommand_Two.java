package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Two extends CommandBase {
    
    // Turn until we are aligned with the april tag as determined by Subcommand One

    private Swerve swerve;

    public AlignWithApriltag_Subcommand_Two(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
