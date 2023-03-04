package frc.robot.commands.setArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetProximalPosition extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double proximalPosition;
    private boolean greater;

    public SetProximalPosition(DoubleArm doubleArm, double proximalPosition) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.proximalPosition = proximalPosition;
    }

    @Override
    public void initialize() {
        doubleArm.setTargetAngles(new double[] {proximalPosition, doubleArm.getTargetArmAngles()[1]});
        greater = doubleArm.getCurrentArmAngles()[0] < proximalPosition;
    }

    @Override
    public void execute() {
        doubleArm.bangbang(true);
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
        return (doubleArm.getTotalError() < tolerance) || ((doubleArm.getCurrentArmAngles()[0] > proximalPosition) == greater);
    }
}
