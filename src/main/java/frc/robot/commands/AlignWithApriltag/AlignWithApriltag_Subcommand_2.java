package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.SwerveConstants.*;

public class AlignWithApriltag_Subcommand_2 extends CommandBase {
    
    // Turn left or right until we are aligned with the april tag

    private Swerve swerve;

    public AlignWithApriltag_Subcommand_2(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.changeHeading(frc.robot.Variables.angular_offset);
            frc.robot.Variables.target_swerve_heading = swerve.getYaw().getDegrees() + frc.robot.Variables.angular_offset;
        }
    }

    @Override
    public void execute() {
        if (frc.robot.Variables.continueWithAWA) swerve.drive(new Translation2d(), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
        }
    }

    @Override
    public boolean isFinished() {
        return ((!frc.robot.Variables.continueWithAWA) || (Math.abs(swerve.getError()) < yaw_tolerance));
    }
}
