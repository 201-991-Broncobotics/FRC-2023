package frc.robot.commands.AlignWithApriltag;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignWithApriltag_Subcommand_Two extends CommandBase {
    
    // Turn until we are aligned with the april tag as determined by Subcommand One

    private Swerve swerve;
    private BooleanSupplier exitSup;
    
    private boolean isFirstAction = true;

    public AlignWithApriltag_Subcommand_Two(Swerve swerve, BooleanSupplier exitSup) {
        this.swerve = swerve;
        addRequirements(swerve);

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
