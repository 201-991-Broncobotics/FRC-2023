package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

import java.util.function.BooleanSupplier;

public class SetProximalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double proximalPosition;
    private BooleanSupplier stopSup;

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        stopSup = () -> false;
        this.proximalPosition = proximalPosition;
    }

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition, BooleanSupplier stopSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.stopSup = stopSup;
        this.proximalPosition = proximalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.resetWhipControl();
        doubleArm.setTargetAngles(new double[] {proximalPosition, doubleArm.getTargetArmAngles()[1]});
    }

    @Override
    public void execute() {
        doubleArm.powerArm(0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // code for if it ended by interruption
        } else {
            // code for if it ended by reaching the target position
            // if we manually move it it will say it's finished
            doubleArm.brake();
        }
    }
    
    @Override
    public boolean isFinished() {
        if (stopSup.getAsBoolean()) {
            doubleArm.resetPID();
            return true;
        }
        return doubleArm.getTotalError() < tolerance;
    }
}
