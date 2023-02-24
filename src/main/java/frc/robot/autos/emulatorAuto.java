package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Swerve;

public class emulatorAuto extends CommandBase { // the main idea is that, repeating the same actions will repeat the robot's actions
    
    private Swerve swerve;
    private double[][] list;

    private boolean isFirstAction = true;

    private double starting_time;

    private int index = 0;

    public emulatorAuto(Swerve swerve, double[][] list) {
        // list: list of {strafe vector x, strafe vector y, turn factor, time}
        this.swerve = swerve;
        this.list = list;
        addRequirements(swerve);
        
        isFirstAction = true;
        starting_time = Timer.getFPGATimestamp() - list[0][3];
        // time we are at is System.currentTimeMillis() / 1000.0 - starting_time
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            starting_time = Timer.getFPGATimestamp() - list[0][3];
            index = 0;
            isFirstAction = false;
        }

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
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - starting_time > list[list.length - 1][3] - 0.1) {
            swerve.drive(new Translation2d(), 0, false, false);
            isFirstAction = true;
            return true;
        }
        return false;
    }
}
