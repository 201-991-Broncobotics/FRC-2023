package frc.robot.commands.defaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class TeleOpClaw extends CommandBase {

    private Claw claw;

    public TeleOpClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.passive();
    }
}