package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Four extends CommandBase {
    
    // Calculate the x of the robot in reference to the april tag

    private Swerve swerve;
    private BooleanSupplier exitSup;

    public AlignWithApriltag_Subcommand_Four(Swerve swerve, BooleanSupplier exitSup) {

        this.exitSup = exitSup;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
