package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AprilTagAlignmentConstants.*;

public class AlignWithApriltag_Subcommand_1 extends CommandBase {

    // Calculate heading of april tag, using current heading
    // also reset variables

    private Swerve swerve;
    private int count;
    private double starting_time;

    public AlignWithApriltag_Subcommand_1(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.brake();

        frc.robot.Variables.angular_offset = 0;
        frc.robot.Variables.previous_x = new double[distance_trials];
        
        starting_time = System.currentTimeMillis() / 1000.0;
        count = 0;
    }

    @Override
    public void execute() { // runs every 0.02 seconds I think?
        if (Limelight.getData()[1] == -12) {
            count = 9999;
            return;
        }
        if ((Math.abs(swerve.getYaw().getDegrees() + Limelight.getData()[1]) + max_angular_tolerance) % 90 < 2 * max_angular_tolerance) {
            // if we're close enough
            frc.robot.Variables.angular_offset += Limelight.getData()[1] / (1.0 * angle_trials);
            count += 1; // we want to run this 25 times, so final count should be 25
        }
        Timer.delay(0.01); // Limelight networktables update every 100hz
    }

    @Override
    public void end(boolean interrupted) {
        frc.robot.Variables.continueWithAWA = (count == angle_trials);
    }

    @Override
    public boolean isFinished() {
        return count >= angle_trials || System.currentTimeMillis() / 1000.0 - starting_time > max_calculation_time;
    }
    
}
