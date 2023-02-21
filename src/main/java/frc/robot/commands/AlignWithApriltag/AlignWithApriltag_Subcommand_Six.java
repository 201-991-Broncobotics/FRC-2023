package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DoubleArm;

public class AlignWithApriltag_Subcommand_Six extends CommandBase {
    
    // put the arm to where we want it to be

    private DoubleArm doubleArm;
    private BooleanSupplier exitSup;
    
    private boolean isFirstAction = true;

    public AlignWithApriltag_Subcommand_Six(DoubleArm doubleArm, BooleanSupplier exitSup) {
        this.doubleArm = doubleArm;
        addRequirements(doubleArm);

        this.exitSup = exitSup;
        isFirstAction = true;
    }

    @Override
    public void execute() {
        if (isFirstAction) {
            isFirstAction = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
