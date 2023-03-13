package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_3 extends CommandBase {
    
    // Strafe left or right until we can see the april tag

    private Swerve swerve;
    private double strafe_speed;
    private double time;
    private double starting_time;

    public AlignWithApriltag_Subcommand_3(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);

            if (frc.robot.Variables.angular_offset < 0) {
                strafe_speed = sideways_speed;
            } else {
                strafe_speed = 0 - sideways_speed;
            }
            time = 0;
            starting_time = Timer.getFPGATimestamp();

            if (Limelight.getData()[0] != -12) {
                strafe_speed *= 0.5;
            }
        }
    }

    @Override
    public void execute() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.drive(new Translation2d(0, strafe_speed * frc.robot.Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
        }
        if (Limelight.getData()[0] != -12 && time == 0) {
            time = Timer.getFPGATimestamp();
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
        return ((!frc.robot.Variables.continueWithAWA) || (Timer.getFPGATimestamp() - time > extra_time_to_be_in_frame && time > 0) || (Timer.getFPGATimestamp() - starting_time > max_time_to_get_in_frame));
    }
}
