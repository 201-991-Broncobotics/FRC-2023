package frc.robot.commands.alignWithApriltag;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPositionV2;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class AlignWithApriltag extends SequentialCommandGroup {
    public AlignWithApriltag(Swerve swerve, DoubleArm doubleArm) {
        addRequirements(swerve, doubleArm);
        addCommands(
            new SetArmPositionV2(doubleArm, idlePositionAngles), 
            new AlignWithApriltag_Subcommand_1(swerve), 
            new AlignWithApriltag_Subcommand_2(swerve), 
            new AlignWithApriltag_Subcommand_3(swerve), 
            new AlignWithApriltag_Subcommand_4(swerve), 
            new AlignWithApriltag_Subcommand_5(swerve)
        );
    }
}
