package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoBalance.AutoBalance;
import frc.robot.commands.utilCommands.*;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.*;

public class Autonomous extends SequentialCommandGroup {

    public static HashMap<String, Command> eventMap = new HashMap<>(); 

    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);

        Timer.delay(1.0); // just to get the limelight set up

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue; // default

        String location = "Short", numElements = "", autoBalance = "Balance", allianceString = "Blue";

        double[] temp = SmartDashboard.getNumberArray("Auto Data", new double[] {-1});
        if (temp.length != 1) {
            switch ((int) temp[0]) {
                case 0:
                    allianceString = "Blue";
                    alliance = DriverStation.Alliance.Blue;
                    break;
                case 1:
                    allianceString = "Red";
                    alliance = DriverStation.Alliance.Red;
                    break;
            }
            switch ((int) temp[1]) {
                case 0:
                    location = "Short";
                    break;
                case 1:
                    location = "Medium";
                    break;
                case 2:
                    location = "Long";
                    break;
            }
            switch ((int) temp[2]) {
                case 0:
                    numElements = "";
                    break;
                case 1:
                    numElements = "Double";
                    break;
            }
            switch ((int) temp[3]) {
                case 0:
                    autoBalance = "Balance";
                    break;
                case 1:
                    autoBalance = "";
                    break;
            }
        }

        String selectedAuto = location + numElements + autoBalance;
            // form of commands: [Short/Medium/Long] [/Double] [/Balance]

        System.out.println("Auto Selector gave : " + allianceString + " side and " + selectedAuto);

        double waitTime = temp[4];

        Command[] drivecommands = getTrajectoryCommands(swerve, selectedAuto, alliance);

        if (numElements.equals("Double")) { // Go forward, drop element, go to intake, pick up element, go back, drop element, go to finish position; 3 stop points
            if (autoBalance.equals("Balance")) {
                addCommands(
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1], 
                    new AutonomousIntake(swerve, doubleArm, claw), 
                    drivecommands[2], 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[3], 
                    new AutoBalance(swerve, doubleArm)
                );
            } else {
                addCommands(
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1], 
                    new AutonomousIntake(swerve, doubleArm, claw), 
                    drivecommands[2], 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[3]
                ); // Literally the same but without an autobalance
            }
        } else { // Go forward, drop element, go to finish position; 1 stop point
            if (autoBalance.equals("Balance")) {
                addCommands(
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1], 
                    new AutoBalance(swerve, doubleArm)
                );
            } else {
                addCommands(
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1]
                );
            }
        }
    }

    private static Command[] getTrajectoryCommands(Swerve swerve, String fileName, DriverStation.Alliance alliance) {
        PIDController thetaController = new PIDController(kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        List<PathPlannerTrajectory> temporaryPathGroup = PathPlanner.loadPathGroup(fileName, new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared));

        Command[] paths = new Command[temporaryPathGroup.size()];

        for (int i = 0; i < paths.length; i++) {
            if (i == 0) {
                PathPlannerTrajectory convertedTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(
                    temporaryPathGroup.get(0), alliance
                );
                paths[0] = new SequentialCommandGroup(
                    new InstantCommand(
                        () -> swerve.resetOdometry(convertedTrajectory.getInitialHolonomicPose())
                    ), 
                    new PPSwerveControllerCommand(
                        convertedTrajectory, 
                        swerve::getPose, 
                        Constants.BaseFalconSwerve.swerveKinematics, 
                        new PIDController(kPXController, 0, 0),
                        new PIDController(kPYController, 0, 0),
                        thetaController, 
                        swerve::setModuleStates, // Module states consumer
                        swerve
                    )
                );
            } else {
                paths[i] = new PPSwerveControllerCommand(
                    PathPlannerTrajectory.transformTrajectoryForAlliance(temporaryPathGroup.get(i), alliance), 
                    swerve::getPose, 
                    Constants.BaseFalconSwerve.swerveKinematics, 
                    new PIDController(kPXController, 0, 0),
                    new PIDController(kPYController, 0, 0),
                    thetaController, 
                    swerve::setModuleStates, // Module states consumer
                    swerve // Requires this drive subsystem
                );
            }
        }
        return paths;
    }
}