package frc.robot.commands.AlignWithApriltag;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_One extends CommandBase {

    // Calculate heading of april tag, using current heading
    // also reset variables

    private Swerve swerve;
    private double count;

    public AlignWithApriltag_Subcommand_One(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.brake();
        frc.robot.Variables.previous_angles = new double[angle_trials];
        frc.robot.Variables.previous_x = new double[distance_trials];
        count = 0;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        frc.robot.Variables.continueWithAWA = true;
        if (interrupted || count < angle_trials) {
            frc.robot.Variables.continueWithAWA = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
