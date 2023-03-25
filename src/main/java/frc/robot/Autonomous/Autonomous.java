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

public class Autonomous {

    public static HashMap<String, Command> autonomousCommands = new HashMap<String, Command>(); 

    public static boolean getCacheEmpty() {
        return autonomousCommands.size() == 0;
    }

    public static void cacheCommandGroups(Swerve swerve, DoubleArm doubleArm, Claw claw) {
        double t = System.currentTimeMillis();
        System.out.println("Started");
        for (String i : new String[] {"Short", "Medium", "Long"}) {
            for (String j : new String[] {"", "Double", "Balance"}) {
                Command[] blueCommands = getTrajectoryCommands(swerve, i + j, DriverStation.Alliance.Blue);
                Command[] redCommands = getTrajectoryCommands(swerve, i + j, DriverStation.Alliance.Red);
                if (j.equals("Balance")) {
                    autonomousCommands.putIfAbsent("Blue" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            blueCommands[0], 
                            new SetArmPosition(doubleArm, idlePositionAngles)
                        ), 
                        new AutoBalance(swerve, doubleArm)
                    ));
                    autonomousCommands.putIfAbsent("Red" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            redCommands[0], 
                            new SetArmPosition(doubleArm, idlePositionAngles)
                        ), 
                        new AutoBalance(swerve, doubleArm)
                    ));
                } else if (j.equals("Double")) {
                    autonomousCommands.putIfAbsent("Blue" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            blueCommands[0], 
                            new SetArmPosition(doubleArm, intakeLowerAngles), 
                            new AutonomousIntake(swerve, claw)
                        ),
                        new ParallelCommandGroup(
                            blueCommands[1], 
                            new SetArmPosition(doubleArm, topPositionAngles)
                        ), 
                        new AutonomousOuttake(swerve, doubleArm, claw), // make it not drive forward???
                        new Drive(swerve, 0.32, -0.4), 
                        new SetArmPosition(doubleArm, idlePositionAngles)
                    ));
                    autonomousCommands.putIfAbsent("Red" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            redCommands[0], 
                            new SetArmPosition(doubleArm, intakeLowerAngles), 
                            new AutonomousIntake(swerve, claw)
                        ),
                        new ParallelCommandGroup(
                            redCommands[1], 
                            new SetArmPosition(doubleArm, topPositionAngles)
                        ), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new Drive(swerve, 0.32, -0.4), 
                        new SetArmPosition(doubleArm, idlePositionAngles)
                    ));
                } else {
                    autonomousCommands.putIfAbsent("Blue" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            blueCommands[0], 
                            new SetArmPosition(doubleArm, idlePositionAngles)
                        )
                    ));
                    autonomousCommands.putIfAbsent("Red" + i + j, new AutonomousSequentialCommandGroup(
                        new SetProximalConstantDistal(doubleArm, -123), 
                        new SetArmPosition(doubleArm, topPositionAngles), 
                        new Drive(swerve, 0.32, 0.4), 
                        new AutonomousOuttake(swerve, doubleArm, claw),
                        new ParallelCommandGroup(
                            redCommands[0], 
                            new SetArmPosition(doubleArm, idlePositionAngles)
                        )
                    ));
                }
            }
        }
        System.out.println("total time " + (System.currentTimeMillis() - t) / 1000.0);
    }

    public static Command getAutonomousCommand(Claw claw, DoubleArm doubleArm, Swerve swerve) {

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
            // form of commands: [Blue/Red] [Short/Medium/Long] [/Double/Balance]

        SmartDashboard.putString("Autonomous", selectedAuto);

        return autonomousCommands.get(selectedAuto);
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
                    new InstantCommand(() -> swerve.resetOdometry(convertedTrajectory.getInitialHolonomicPose())), 
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

class AutonomousSequentialCommandGroup extends SequentialCommandGroup {
    public AutonomousSequentialCommandGroup(Command ... commands) {
        super(commands);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}