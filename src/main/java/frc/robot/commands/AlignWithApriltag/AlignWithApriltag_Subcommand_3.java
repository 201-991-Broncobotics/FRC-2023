package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_3 extends CommandBase {
    
    // Strafe left or right until we can see the april tag

    private Swerve swerve;
    private double strafe_speed;
    private double time;
    private double start_time;

    public AlignWithApriltag_Subcommand_3(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
            if (Limelight.getData()[0] == -12) {
                if (frc.robot.Variables.angular_offset < 0) {
                    strafe_speed = sideways_speed;
                } else {
                    strafe_speed = 0 - sideways_speed;
                }
                time = 0;
                start_time = System.currentTimeMillis() / 1000.0;
            } else {
                strafe_speed = 0.0;
            }
        }
    }

    @Override
    public void execute() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.drive(new Translation2d(0, strafe_speed * frc.robot.Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
        if (Limelight.getData()[0] != -12 && time == 0) {
            time = System.currentTimeMillis() / 1000.0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
        }
        if (Limelight.getData()[0] == -12) frc.robot.Variables.continueWithAWA = false;
    }

    @Override
    public boolean isFinished() {
        return ((!frc.robot.Variables.continueWithAWA) || (System.currentTimeMillis() - time > extra_time_to_be_in_frame && time > 0) || (System.currentTimeMillis() - start_time > max_time_to_get_in_frame));
    }
}
