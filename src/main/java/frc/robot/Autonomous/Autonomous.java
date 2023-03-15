package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
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

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue; // default

        String location = "null";
        int number_of_elements = -1;
        boolean autobalance = false;
        
        String alliance_str = "blue";

        double[] temp = SmartDashboard.getNumberArray("Auto Data", new double[] {-1});
        if (temp.length != -1) {
            switch ((int) temp[0]) {
                case 0:
                    alliance = DriverStation.Alliance.Blue;
                    alliance_str = "blue";
                    break;
                case 1:
                    alliance = DriverStation.Alliance.Red;
                    alliance_str = "red";
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

        System.out.println("Auto Selector gave : " + alliance_str + " " + location + " " + number_of_elements + " " + autobalance);

        String selectedAuto = "NewEngland";

        addCommands(
            new RunTrajectoryAdvanced(swerve, selectedAuto, alliance)
        );
    }
}
