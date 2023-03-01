package frc.robot.commands.utilCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

public class TerminateArmCommands extends CommandBase {

    DoubleArm doubleArm;
    public TerminateArmCommands(DoubleArm doubleArm) {
        addRequirements(doubleArm);
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
