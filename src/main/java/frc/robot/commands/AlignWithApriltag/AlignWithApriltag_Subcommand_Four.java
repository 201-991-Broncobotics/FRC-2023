package frc.robot.commands.AlignWithApriltag;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Four extends CommandBase {
    
    // Calculate the x of the robot in reference to the april tag

    private Swerve swerve;

    public AlignWithApriltag_Subcommand_Four(Swerve swerve) {
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
