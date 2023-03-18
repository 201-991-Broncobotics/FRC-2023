package frc.robot.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoBalance.AutoBalance;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.setArmPosition.SetProximalConstantDistal;
import frc.robot.commands.utilCommands.*;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

import static frc.robot.Constants.TuningConstants.*;
import static frc.robot.Constants.AutoConstants.*;

public class Autonomous extends SequentialCommandGroup {

    public static HashMap<String, Command[]> autonomousCommands = new HashMap<String, Command[]>(); 

    public static boolean getCacheEmpty() {
        return autonomousCommands.size() == 0;
    }

    public static void cacheCommandGroups(Swerve swerve) {
        double t = System.currentTimeMillis();
        System.out.println("Started");
        for (String i : new String[] {
            "Short", "ShortDouble", "ShortBalance", "ShortDoubleBalance", 
            "Medium", "MediumDouble", "MediumBalance", "MediumDoubleBalance", 
            "Long", "LongDouble", "LongBalance", "LongDoubleBalance"}
        ) {
            autonomousCommands.putIfAbsent("Blue" + i, getTrajectoryCommands(swerve, i, DriverStation.Alliance.Blue));
            autonomousCommands.putIfAbsent("Red" + i, getTrajectoryCommands(swerve, i, DriverStation.Alliance.Red));
        }
        System.out.println("total time " + (System.currentTimeMillis() - t) / 1000.0);
    }

    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(claw, doubleArm, swerve);

        // DriverStation.Alliance alliance = DriverStation.Alliance.Blue; // default

        String allianceString = "Blue", location = "Short", numElements = "", autoBalance = "Balance";

        double[] temp = SmartDashboard.getNumberArray("Auto Data", new double[] {-1});
        if (temp.length != 1) {
            System.out.println((int) temp[0]);
            switch ((int) temp[0]) {
                case 0:
                    allianceString = "Blue";
                    Limelight.setSide("blue");
                    // alliance = DriverStation.Alliance.Blue;
                    break;
                case 1:
                    allianceString = "Red";
                    Limelight.setSide("red");
                    // alliance = DriverStation.Alliance.Red;
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

        String selectedAuto = allianceString + location + numElements + autoBalance;
            // form of commands: [Short/Medium/Long] [/Double] [/Balance]

        System.out.println("Auto Selector gave : " + allianceString + " side and " + selectedAuto);

        double waitTime;
        try {
            waitTime = temp[4];
        } catch (ArrayIndexOutOfBoundsException e) {
            System.out.println("Auto selector not connected");
            return;
        }
        
        Command[] drivecommands = autonomousCommands.get(selectedAuto); // getTrajectoryCommands(swerve, selectedAuto, alliance);

        if (numElements.equals("Double")) { // Go forward, drop element, go to intake, pick up element, go back, drop element, go to finish position; 3 stop points
            if (autoBalance.equals("Balance")) {
                addCommands(
                    new SetProximalConstantDistal(doubleArm, -123), 
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1], 
                    new SetArmPosition(doubleArm, intakeLowerAngles), // I don't think it works if we put it inside autonomousintake
                    new AutonomousIntake(swerve, claw), 
                    drivecommands[2], 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[3], 
                    new AutoBalance(swerve, doubleArm)
                );
            } else {
                addCommands(
                    new SetProximalConstantDistal(doubleArm, -123), 
                    new Wait(waitTime), 
                    drivecommands[0],
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[1], 
                    new SetArmPosition(doubleArm, intakeLowerAngles),
                    new AutonomousIntake(swerve, claw), 
                    drivecommands[2], 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    drivecommands[3]
                ); // Literally the same but without an autobalance
            }
        } else { // Go forward, drop element, go to finish position; 1 stop point
            if (autoBalance.equals("Balance")) {
                addCommands(
                    new SetProximalConstantDistal(doubleArm, -123), 
                    new Wait(waitTime), 
                    new SetArmPosition(doubleArm, topPositionAngles), 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    new ParallelCommandGroup(
                        new SetArmPosition(doubleArm, idlePositionAngles), 
                        drivecommands[0]
                    ),
                    new AutoBalance(swerve, doubleArm)
                );
            } else {
                addCommands(
                    new SetProximalConstantDistal(doubleArm, -123), 
                    new Wait(waitTime), 
                    new SetArmPosition(doubleArm, topPositionAngles), 
                    new AutonomousOuttake(swerve, doubleArm, claw),
                    new ParallelCommandGroup(
                        new SetArmPosition(doubleArm, idlePositionAngles), 
                        drivecommands[0]
                    )
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