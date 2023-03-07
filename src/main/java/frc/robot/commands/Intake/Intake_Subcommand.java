package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.Buttons.*;

public class Intake_Subcommand extends CommandBase {
    
    private Claw claw;
    private DoubleArm doubleArm;

    private DoubleSupplier motorOneSup;
    private DoubleSupplier motorTwoSup;

    private double starting_time;

    public Intake_Subcommand(Claw claw, DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup) {
        this.claw = claw;
        this.doubleArm = doubleArm;
        addRequirements(claw, doubleArm);

        this.motorOneSup = motorOneSup;
        this.motorTwoSup = motorTwoSup;
    }

    @Override
    public void initialize() {
        claw.intake();
        starting_time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorOneVal = signedPower(motorOneSup.getAsDouble());
        double motorTwoVal = signedPower(motorTwoSup.getAsDouble());

        // Move Arm
        doubleArm.powerArm(
            motorOneVal * first_motor_sensitivity, 
            motorTwoVal * second_motor_sensitivity
        );
    }

    @Override
    public void end(boolean interrupted) {
        claw.stop();
    }
    
    @Override
    public boolean isFinished() {
        return (claw.getCurrent() > claw_current_limit) && (Timer.getFPGATimestamp() - starting_time > 0.5);
    }
}
