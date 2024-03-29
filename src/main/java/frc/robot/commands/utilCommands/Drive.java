package frc.robot.commands.utilCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class Drive extends CommandBase {
        
    private Swerve swerve;
    private final double power, distance;
    private Pose2d starting_pose;
    private double end_time;

    public Drive(Swerve swerve, double distance, double power) {
        this.swerve = swerve;
        this.power = power;
        this.distance = distance;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.changeHeading(0);
        starting_pose = swerve.getPose();
        end_time = Timer.getFPGATimestamp() + 1;
    }

    @Override
    public void execute() {
        // we might do a PID thing...
        double conv_power = Math.min(Math.abs(power), (distance - swerve.getPose().relativeTo(starting_pose).getTranslation().getNorm()) * pT);
        if (power < 0) conv_power = -conv_power;
        conv_power = power; // we could test without this line soon
        swerve.drive(new Translation2d(conv_power, 0).times(Constants.BaseFalconSwerve.maxSpeed), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.brake();
    }
    
    @Override
    public boolean isFinished() {
        return ((swerve.getPose().relativeTo(starting_pose).getTranslation().getNorm() > distance - dT) || (Timer.getFPGATimestamp() > end_time));
    }
    
}