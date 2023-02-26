package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_5 extends CommandBase {
    
    // Strafe until we are in line with april tag, then finally brake 

    private Swerve swerve;
    private double strafe_speed;
    private double starting_time;

    private double total_offset;

    private Pose2d startingPose;

    public AlignWithApriltag_Subcommand_5(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
            starting_time = Timer.getFPGATimestamp();

            double tag_offset = 0;
            if (Limelight.getData()[0] == 1) {
                tag_offset = 1;
            } else {
                tag_offset = 3; // TODO: Don't forget to change this
            }

            if ((offset + tag_offset) * 0.0254 < frc.robot.Variables.prev_x_average) {
                strafe_speed = sideways_speed;
            } else {
                strafe_speed = -sideways_speed;
            }

            total_offset = Math.abs(frc.robot.Variables.prev_x_average - (offset + tag_offset) * 0.0254); // how far we need to strafe in meters
            startingPose = swerve.getPose();
        }
    }

    @Override
    public void execute() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.drive(new Translation2d(0, strafe_speed * frc.robot.Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
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
        return (!frc.robot.Variables.continueWithAWA || Timer.getFPGATimestamp() - starting_time > max_alignment_time || swerve.getPose().relativeTo(startingPose).getTranslation().getNorm() > total_offset);
    }

}
