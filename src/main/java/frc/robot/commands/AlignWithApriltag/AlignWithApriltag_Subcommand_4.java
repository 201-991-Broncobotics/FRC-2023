package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_4 extends CommandBase {
    
    // Calculate the x of the robot in reference to the april tag

    private Swerve swerve;
    private int count;
    private double starting_time;

    public AlignWithApriltag_Subcommand_4(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
            frc.robot.Variables.previous_x = new double[distance_trials];
            frc.robot.Variables.prev_x_average = 0;
            starting_time = Timer.getFPGATimestamp();
            count = 0;
        }
    }

    @Override
    public void execute() {
        if (frc.robot.Variables.continueWithAWA) { // runs every 0.02 seconds I think?
            if (Limelight.getData()[1] == -12) {
                count = 9999;
                return;
            }
            if (Math.abs(Limelight.getData()[1]) < max_angular_tolerance) {
                
                frc.robot.Variables.previous_x[count] += Limelight.getData()[2] / (1.0 * distance_trials);
                frc.robot.Variables.prev_x_average += frc.robot.Variables.previous_x[count];

                count += 1; // we want to run this 10 times, so final count should be 10
            }
            Timer.delay(0.01); // Limelight networktables update every 100hz
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (frc.robot.Variables.continueWithAWA) {
            swerve.brake();
            swerve.setTargetHeading(frc.robot.Variables.target_swerve_heading);
        }
        frc.robot.Variables.continueWithAWA = (frc.robot.Variables.continueWithAWA) && (count == distance_trials);
    }

    @Override
    public boolean isFinished() {
        return (!frc.robot.Variables.continueWithAWA || count >= distance_trials || Timer.getFPGATimestamp() - starting_time > max_calculation_time);
    }

}
