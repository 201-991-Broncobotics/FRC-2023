package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);

        Timer.delay(1.0); // just to get the limelight set up

        int auto_number = (int) SmartDashboard.getNumber("April Tag ID", -2);

        switch (auto_number) {
            case 1: // Red Right/Bottom
                addCommands(
                    new RunTrajectory(swerve, "RedBottomPath")
                );
                break;

            case 2: // Red Center/Middle
                addCommands(
                    new RunTrajectory(swerve, "RedMiddlePath")
                );
                break;
            
            case 3: // Red Left/Top
                addCommands(
                    new RunTrajectory(swerve, "RedTopPath")
                );
                break;
            
            case 6: // Blue Right/Top
                addCommands(
                    new RunTrajectory(swerve, "BlueTopPath")
                );
                break;
            
            case 7: // Blue Center/Middle
                addCommands(
                    new RunTrajectory(swerve, "BlueMiddlePath")
                );
                break;
            
            case 8: // Blue Left/Bottom
                addCommands(
                    new RunTrajectory(swerve, "BlueBottomPath")
                );
                break;
            
            default: // idk, we'll have to figure out a plan for this
                addCommands(
                    new RunTrajectory(swerve, "TestPath")
                );
                break;
        }
    }
}
