package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class Autonomous extends SequentialCommandGroup {
    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve, String selectedAuto) {
        addRequirements(doubleArm, claw, swerve);

        Timer.delay(1.0); // just to get the limelight set up

        SmartDashboard.putString("Selected Autonomous", selectedAuto);

        addCommands(
            new RunTrajectoryAdvanced(swerve, selectedAuto)
        );

        String alliance_side = "null", location = "null";
        int number_of_elements = -1;
        boolean autobalance = false;
        double[] temp = SmartDashboard.getNumberArray("Auto Data", new double[] {-1});
        if (temp.length != -1) {
            switch ((int) temp[0]) {
                case 0:
                    alliance_side = "blue";
                    break;
                case 1:
                    alliance_side = "red";
                    break;
            }
            switch ((int) temp[1]) {
                case 0:
                    location = "short";
                    break;
                case 1:
                    location = "medium";
                    break;
                case 2:
                    location = "long";
                    break;
            }
            switch ((int) temp[2]) {
                case 0:
                    number_of_elements = 1;
                    break;
                case 1:
                    number_of_elements = 2;
                    break;
            }
            switch ((int) temp[3]) {
                case 0:
                    autobalance = true;
                    break;
                case 1:
                    autobalance = false;
                    break;
            }
        }

        System.out.println("Auto Selector gave : " + alliance_side + " " + location + " " + number_of_elements + " " + autobalance);
        /*
        switch (auto_number) {
            case 1: // Red Right/Bottom
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "RedBottomPath")
                );
                break;

            case 2: // Red Center/Middle
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "RedMiddlePath")
                );
                break;
            
            case 3: // Red Left/Top
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "RedTopPath")
                );
                break;
            
            case 6: // Blue Right/Top
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "BlueTopPath")
                );
                break;
            
            case 7: // Blue Center/Middle
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "BlueMiddlePath")
                );
                break;
            
            case 8: // Blue Left/Bottom
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "BlueBottomPath")
                );
                break;
            
            case 9:
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "TestForwardPath")
                );
                break;
            
            case 10:
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "TestStrafePath")
                );
                break;
            
            case 11:
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "TestClockwisePath")
                );
                break;
            
            case 12:
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "TestPath")
                );
                break;
            
            default: // just place a cube then do nothing
                addCommands(
                    new RunTrajectoryAdvanced(swerve, "DefaultPath")
                );
                break;
        } */
    }
}
