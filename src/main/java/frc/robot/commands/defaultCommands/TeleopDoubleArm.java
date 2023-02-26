package frc.robot.commands.defaultCommands;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.Buttons.*;

public class TeleopDoubleArm extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier motorOneSup;
    private DoubleSupplier motorTwoSup;

    private BooleanSupplier stopSup;

    public TeleopDoubleArm(DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup, BooleanSupplier stopSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.motorOneSup = motorOneSup;
        this.motorTwoSup = motorTwoSup;
        this.stopSup = stopSup;
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorOneVal = signedPower(motorOneSup.getAsDouble());
        double motorTwoVal = signedPower(motorTwoSup.getAsDouble());

        if (stopSup.getAsBoolean()) doubleArm.resetPID();

        // Move Arm
        doubleArm.powerArm(
            motorOneVal * first_motor_sensitivity, 
            motorTwoVal * second_motor_sensitivity
        );
    }
}