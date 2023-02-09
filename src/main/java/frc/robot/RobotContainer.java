package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final IntSupplier pov = () -> 0 - driver.getPOV();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    // codding
    private double target_heading = 0; // s_Swerve resets the imu
    private final double cappppping = 0.7;
    private final double max_error = 60; // anything greater than this will go to capping power

    private final double calibration_time = 0.5; // in seconds
    private double last_time = System.currentTimeMillis();

    public double errorToDouble(double error_in_percent) { // ex. if we are 3* off and 30* is max error, we get 0.1
        // error_in_percent will always be positive (not zero or negative)
        return Math.pow(error_in_percent, 0.9);
        // make sure this is always between 0 and 1
        // basic p control --> return error_in_percent;
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> getTurning(() -> -driver.getRawAxis(rotationAxis)), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    public static double normalizeAngle(double angle) {
        while (angle > 180) { angle -= 360; }
        while (angle <= -180) { angle += 360; }

        return angle;
    }

    public double getTurning(DoubleSupplier con_turn) {
        double turning = con_turn.getAsDouble();
        double current_angle = s_Swerve.getYaw().getDegrees();
        if (Math.abs(turning) > 0.1) {
            target_heading = current_angle;
            last_time = System.currentTimeMillis();
            return turning;
        }
        if (System.currentTimeMillis() - last_time < calibration_time * 1000) {
            target_heading = current_angle;
            return 0;
        }
        if (pov.getAsInt() % 90 == 0) {
            target_heading = normalizeAngle(pov.getAsInt() - current_angle) + current_angle;
        }
        double error_in_percent = Math.max(Math.min((target_heading - current_angle) / max_error, 1), -1);
        if (error_in_percent == 0) {
            return 0;
        }
        int multiplier = 1;
        if (error_in_percent < 0) {
            error_in_percent = 0 - error_in_percent;
            multiplier = -1;
        }
        return errorToDouble(error_in_percent) * cappppping * multiplier;
    }

    public void resetGyro(){
        s_Swerve.zeroGyro();
        target_heading = 0; // should be 0 but just to be safe :)
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> resetGyro()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
