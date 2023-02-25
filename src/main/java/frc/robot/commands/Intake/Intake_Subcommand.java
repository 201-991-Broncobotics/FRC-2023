package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake_Subcommand extends CommandBase {
    
    private Claw claw;
    private BooleanSupplier stopSup;

    private double starting_time;

    public Intake_Subcommand(Claw claw, BooleanSupplier stopSup) {
        this.claw = claw;
        addRequirements(claw);

        this.stopSup = stopSup;
    }

    @Override
    public void initialize() {
        claw.intake();
        starting_time = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        // nothing
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
    
    @Override
    public boolean isFinished() {
        return ((stopSup.getAsBoolean()) || ((claw.getCurrent() > claw_current_limit) && (System.currentTimeMillis() / 1000.0 - starting_time > 0.8)));
    }
}
