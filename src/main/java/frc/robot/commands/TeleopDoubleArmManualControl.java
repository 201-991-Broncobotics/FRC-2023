package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;
import static frc.robot.Constants.Buttons.*;

public class TeleopDoubleArmManualControl extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier motorOneSup;
    private DoubleSupplier motorTwoSup;

    public TeleopDoubleArmManualControl(DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.motorOneSup = motorOneSup;
        this.motorTwoSup = motorTwoSup; // sets up the double suppliers
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorOneVal = Math.pow(MathUtil.applyDeadband(motorOneSup.getAsDouble(), joystick_deadzone), axis_exponent);
        double motorTwoVal = Math.pow(MathUtil.applyDeadband(motorTwoSup.getAsDouble(), joystick_deadzone), axis_exponent);

        // Move Arm
        doubleArm.rawPowerArm(
            motorOneVal * raw_arm_sensitivity, // power????? but it's weird
            motorTwoVal * raw_arm_sensitivity_two
        );
    }
}