package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DoubleArm;
import static frc.robot.Constants.DoubleArmConstants.*;

public class TeleopDoubleArm extends CommandBase {

    private DoubleArm doubleArm;

    private DoubleSupplier horizSup;
    private DoubleSupplier vertSup;

    public TeleopDoubleArm(DoubleArm doubleArm, DoubleSupplier horizSup, DoubleSupplier vertSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm); // means that other functions are not allowed to access it

        this.horizSup = horizSup;
        this.vertSup = vertSup; // sets up the double suppliers
    }

    @Override
    public void execute() {
        // Account for Stick Drift
        double horizVal = MathUtil.applyDeadband(horizSup.getAsDouble(), joystick_deadzone);
        double vertVal = MathUtil.applyDeadband(vertSup.getAsDouble(), joystick_deadzone);

        // Move Arm
        doubleArm.moveArm(
            horizVal * arm_sensitivity, 
            vertVal * arm_sensitivity
        );
    }

}