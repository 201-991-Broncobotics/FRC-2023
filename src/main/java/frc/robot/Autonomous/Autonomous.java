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
import frc.robot.commands.outtake.Outtake;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.utilCommands.Brake;
import frc.robot.commands.utilCommands.Wait;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.TuningConstants.*;
import frc.robot.Constants;

public class Autonomous extends SequentialCommandGroup {

    public static HashMap<String, Command> eventMap = new HashMap<>(); 

    public Autonomous(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);
        eventMap.putIfAbsent("autoBalance", new AutoBalance(swerve, doubleArm)); 
        eventMap.putIfAbsent("event", new SetArmPosition(doubleArm, new double[] {0, 0}));

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

        String selectedAuto = "LongCone";

        Command[] drivecommands = getTrajectoryCommands(swerve, selectedAuto, alliance);
        addCommands(
            drivecommands[0], // we have n + 1 drivecommands, where n is the number of stop points not counting start/ends
            new SetArmPosition(doubleArm, topPositionAngles), 
            new Brake(swerve), 
            new Wait(0.2), 
            new Outtake(claw, doubleArm),
            drivecommands[1], 
            new AutoBalance(swerve, doubleArm)
        );
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
