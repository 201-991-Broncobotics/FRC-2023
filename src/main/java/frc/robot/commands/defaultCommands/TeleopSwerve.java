package frc.robot.commands.defaultCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier targetSup;
    private DoubleSupplier slowSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier targetSup, DoubleSupplier slowSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetSup = targetSup;
        this.slowSup = slowSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = signedPower(translationSup.getAsDouble());
        double strafeVal = signedPower(strafeSup.getAsDouble());
        double rotationVal = signedPower(rotationSup.getAsDouble()) * swerve_turn_sensitivity;

        if ((rotationVal) == 0 && (targetSup.getAsInt() % 90 == 0)) s_Swerve.setTargetHeading(targetSup.getAsInt());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.BaseFalconSwerve.maxSpeed).times(slowSup.getAsDouble()), 
            rotationVal * Constants.BaseFalconSwerve.maxAngularVelocity * Math.min(1, slowSup.getAsDouble() * tffsmbtcfigfdbnft), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}