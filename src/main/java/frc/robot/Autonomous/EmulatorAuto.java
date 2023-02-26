package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;

public class EmulatorAuto extends CommandBase { // the main idea is that, repeating the same actions will repeat the robot's actions
    
    private Swerve swerve;
    private double[][] list;

    private double starting_time;

    private int index = 0;

    public EmulatorAuto(Swerve swerve, double[][] list) {
        // list: list of {strafe vector x, strafe vector y, turn factor, time}
        this.swerve = swerve;
        this.list = list;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        starting_time = Timer.getFPGATimestamp() - list[0][3];
        index = 0;
    }

    @Override
    public void execute() {
        double modified_time = Timer.getFPGATimestamp() - starting_time;
        if (modified_time > list[list.length - 1][3] - 0.1) return; // if we are greater than the last time then return
        
        while (true) {
            if (modified_time < list[index][3]) {
                // the one we are at
                swerve.drive(
                    new Translation2d(list[index][0], list[index][1]), 
                    list[index][2], 
                    false, 
                    true
                );
                return;
            } else {
                index += 1;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false, false);
    }
    
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - starting_time > list[list.length - 1][3] - 0.1;
    }
}
