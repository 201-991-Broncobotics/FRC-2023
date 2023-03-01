package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;
import frc.robot.subsystems.Swerve;

public class TerminateCommands extends CommandBase {

    DoubleArm doubleArm;
    public TerminateCommands(Claw claw, DoubleArm doubleArm, Swerve swerve) {
        addRequirements(doubleArm, claw, swerve);
        this.doubleArm = doubleArm;
    }

    @Override
    public void execute() {
        doubleArm.resetPID();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
