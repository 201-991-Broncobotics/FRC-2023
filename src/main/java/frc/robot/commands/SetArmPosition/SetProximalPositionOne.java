package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetProximalPositionOne extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double proximalPosition;

    public SetProximalPositionOne(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;

        this.proximalPosition = proximalPosition;
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        if (proximalPosition > doubleArm.getCurrentArmAngles()[0]) {
            doubleArm.resetWhipControl();
            doubleArm.setTargetPositions(DoubleArm.getPositionFromAngles(proximalPosition, doubleArm.getTargetArmAngles()[1]));
        } else {
            doubleArm.setTargetPositions(doubleArm.getCurrentXY());
        }
    }

    @Override
    public void execute() {
        doubleArm.rawPowerArm(0, 0);
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
        return doubleArm.getTotalError() < tolerance;
    }
}
