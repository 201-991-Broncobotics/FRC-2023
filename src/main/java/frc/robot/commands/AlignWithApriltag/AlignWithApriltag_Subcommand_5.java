package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_5 extends CommandBase {
    
    // Strafe until we are in line with april tag, then finally brake 

    private Swerve swerve;

    public AlignWithApriltag_Subcommand_5(Swerve swerve) {
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
