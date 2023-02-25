package frc.robot.commands.SetArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DoubleArmConstants.*;

public class SetDistalPositionOne extends CommandBase { // big arm

    private DoubleArm doubleArm;
    private double distalPosition;

    public SetDistalPositionOne(DoubleArm doubleArm, double distalPosition) {
        this.doubleArm = doubleArm;

        this.distalPosition = distalPosition;
    }

    @Override
    public void initialize() { // we only want to run if our target proximal is above the current
        if (distalPosition > doubleArm.getCurrentArmAngles()[1]) {
            SmartDashboard.putNumber("HERE", 1);
            doubleArm.resetWhipControl();
            double temp[] = (DoubleArm.getPositionFromAngles(doubleArm.getTargetArmAngles()[0], distalPosition));
            double[] temp_position = DoubleArm.getAnglesFromTarget(temp[0], temp[1]);
            if (Math.abs(temp_position[0] - doubleArm.getTargetArmAngles()[0]) + Math.abs(temp_position[1] - doubleArm.getTargetArmAngles()[1]) < 8) {
                doubleArm.setTargetPositions(DoubleArm.getPositionFromAngles(temp_position[0], temp_position[1]));
            } else {
                doubleArm.setTargetPositions(doubleArm.getCurrentXY());
            }
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
