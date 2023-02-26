package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class TerminateCommands extends CommandBase {

    public TerminateCommands(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);
    }

    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
