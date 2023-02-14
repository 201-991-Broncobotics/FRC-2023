package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

public class TestingDoubleArm extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier motorOneSup;
    private DoubleSupplier motorTwoSup;

    public TestingDoubleArm(DoubleArm doubleArm, DoubleSupplier motorOneSup, DoubleSupplier motorTwoSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.motorOneSup = motorOneSup;
        this.motorTwoSup = motorTwoSup; // sets up the double suppliers
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double motorOneVal = MathUtil.applyDeadband(motorOneSup.getAsDouble(), joystick_deadzone);
        double motorTwoVal = MathUtil.applyDeadband(motorTwoSup.getAsDouble(), joystick_deadzone);

        // Move Arm
        doubleArm.rawPowerArm(
            motorOneVal * arm_sensitivity, // power????? but it's weird
            motorTwoVal * arm_sensitivity
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}