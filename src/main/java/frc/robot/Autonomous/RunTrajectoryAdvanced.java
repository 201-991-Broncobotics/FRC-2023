package frc.robot.autonomous;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import static frc.robot.Constants.AutoConstants.*;

public class RunTrajectoryAdvanced extends SequentialCommandGroup {
    
    public RunTrajectoryAdvanced(Swerve swerve, String fileName) {

        PathPlannerTrajectory trajectoryPath = PathPlanner.loadPath(
            fileName, 
            new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        );

        PIDController thetaController = new PIDController(kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            trajectoryPath, 
            swerve::getPose, // Pose supplier
            Constants.BaseFalconSwerve.swerveKinematics, 
            new PIDController(kPXController, 0, 0),
            new PIDController(kPYController, 0, 0),
            thetaController, // It might automatically enable continuous input which means we can make a new thing directly here but oh well
            swerve::setModuleStates, // Module states consumer
            swerve // Requires this drive subsystem
        );

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(trajectoryPath.getInitialPose())),
            swerveControllerCommand
        );
    }
}
