package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.AutonomousNames.*;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve, String selectedAuto) {
        addRequirements(doubleArm, claw, swerve);

        Timer.delay(1.0); // just to get the limelight set up

        int auto_number = -2;

        if (selectedAuto.equals(autos[0][1])) {
            auto_number = (int) SmartDashboard.getNumber("April Tag ID", -2);
        } else if (selectedAuto.equals(autos[1][1])) {
            auto_number = 1;
        } else if (selectedAuto.equals(autos[2][1])) {
            auto_number = 2;
        } else if (selectedAuto.equals(autos[3][1])) {
            auto_number = 3;
        } else if (selectedAuto.equals(autos[4][1])) {
            auto_number = 6;
        } else if (selectedAuto.equals(autos[5][1])) {
            auto_number = 7;
        } else if (selectedAuto.equals(autos[6][1])) {
            auto_number = 8;
        } else if (selectedAuto.equals(autos[7][1])) {
            auto_number = -2; // default path
        } else if (selectedAuto.equals(autos[8][1])) {
            auto_number = 9; // Drive Forward One Meter
        } else if (selectedAuto.equals(autos[9][1])) {
            auto_number = 10; // Strafe Right One Meter
        } else if (selectedAuto.equals(autos[10][1])) {
            auto_number = 11; // Turn Clockwise 90 degrees
        } else if (selectedAuto.equals(autos[11][1])) {
            auto_number = 12; // Test Path
        }

        SmartDashboard.putString("Selected Autonomous", selectedAuto);

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
            
            case 9:
                addCommands(
                    new RunTrajectory(swerve, "TestForwardPath")
                );
                break;
            
            case 10:
                addCommands(
                    new RunTrajectory(swerve, "TestStrafePath")
                );
                break;
            
            case 11:
                addCommands(
                    new RunTrajectory(swerve, "TestClockwisePath")
                );
                break;
            
            case 12:
                addCommands(
                    new RunTrajectory(swerve, "TestPath")
                );
                break;
            
            default: // just place a cube then do nothing
                addCommands(
                    new RunTrajectory(swerve, "DefaultPath")
                );
                break;
        }
    }
}
