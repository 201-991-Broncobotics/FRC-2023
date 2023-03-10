package frc.robot.commands.autoBalance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setArmPosition.SetArmPosition;
import frc.robot.commands.utilCommands.Brake;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

import static frc.robot.Constants.TuningConstants.*;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(Swerve swerve, DoubleArm doubleArm) {
        addRequirements(swerve, doubleArm);
        addCommands(
            new SetArmPosition(doubleArm, idlePositionAngles), 
            new AutoBalance_Subcommand_1(swerve), // sets target angle of swerve
            new AutoBalance_Subcommand_2(swerve), // drives until we are on the balance thing
            new AutoBalance_Subcommand_3(swerve), // autobalances forever
            new Brake(swerve) // lol
        );
    }
}
