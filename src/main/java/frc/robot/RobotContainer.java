package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.*;
import frc.robot.commands.alignWithApriltag.AlignWithApriltag;
import frc.robot.commands.alignWithApriltag.AlignWithApriltagOld;
import frc.robot.commands.autoBalance.AutoBalance;
import frc.robot.commands.defaultCommands.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.outtake.Outtake;
import frc.robot.commands.setArmPosition.*;
import frc.robot.commands.utilCommands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.AutonomousNames.*;
import static frc.robot.Constants.Buttons.*;
import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.TuningConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(driver_usb_port);
    private final XboxController operator = new XboxController(operator_usb_port);

    /* Driver Buttons */
    private final Trigger zeroGyro = new JoystickButton(driver, zeroGyroButton);
    private final Trigger robotCentric = new JoystickButton(driver, robotCentricButton);
    private final Trigger tagAligner = new JoystickButton(driver, tagAlignerButton);
    private final Trigger tagAlignerNew = new JoystickButton(driver, tagAlignerNewButton);
    private final Trigger autoBalance = new JoystickButton(driver, autoBalanceButton);
    private final Trigger terminateCommandsDriver = new JoystickButton(driver, terminateCommandsDriverButton);

    /* Operator Buttons */
    private final Trigger topGoal = new JoystickButton(operator, topGoalButton);
    private final Trigger midGoal = new JoystickButton(operator, midGoalButton);
    private final Trigger lowGoal = new JoystickButton(operator, lowGoalButton);

    private final Trigger idle = new JoystickButton(operator, idleButton);

    private final Trigger intake = new JoystickButton(operator, intakeButton);
    private final Trigger outtake = new JoystickButton(operator, outtakeButton);
    
    private final Trigger terminateCommandsOperator = new JoystickButton(operator, terminateCommandsOperatorButton);

    /* Custom Buttons - this is how you make Triggers based on conditions
    private final Trigger customTrigger = new Trigger(BooleanSupplier condition);

    for example, if you wanted to make a trigger for when the dpad is pressed up, you would do this:
    private final Trigger dpad_up = new Trigger(() -> operator.getPOV() == 0);
    .getPOV() method: increases by 45 degrees clockwise, with up being 0, right being 90, down being 180 and left being 270

    if you wanted to make a trigger for when an axis reaches past a certain value, you would do this:
    private final Trigger right_trigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > joystick_deadzone);

    you could also make a trigger for something more complicated, like if the double arm's x value is greater than 20 */

    private final Trigger stopArmCommands = new Trigger(() -> (
        (operator.getRawAxis(stopArmFromMovingButtonOne) > joystick_deadzone) || 
        (operator.getRawAxis(stopArmFromMovingButtonTwo) > joystick_deadzone)
    ));

    private final Trigger intakeUpper = new Trigger(() -> (operator.getPOV() == intakeUpperValue));
    private final Trigger intakeLower = new Trigger(() -> (operator.getPOV() == intakeLowerValue));

    private final Trigger doneWithIntake = new Trigger(() -> (frc.robot.Variables.go_to_startposition));

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final DoubleArm doubleArm = new DoubleArm();
    private final Claw claw = new Claw(); // testing purposes, if there's an error then comment this back out

    /* Auto Chooser */
    private final SendableChooser<String> autonomousChooser = new SendableChooser<String>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        if (fancy_drive) {
            s_Swerve.setDefaultCommand(
                new TeleopSwerveAbsoluteDirecting(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(directionXAxis), 
                    () -> -driver.getRawAxis(directionYAxis), 
                    () -> -driver.getPOV(), 
                    () -> driver.getRawAxis(turnLeftAxis) - driver.getRawAxis(turnRightAxis), 
                    () -> (driver.getRawButton(slowButtonOne) || driver.getRawButton(slowButtonTwo)))
            );
        } else {
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> robotCentric.getAsBoolean(), 
                    () -> -driver.getPOV(), 
                    () -> 1 - 0.75 * driver.getRawAxis(slowAxis) // what we multiply translation speed by; rotation speed is NOT affected
                )
            );
        }

        doubleArm.setDefaultCommand(
            new TeleopDoubleArm(
                doubleArm, 
                () -> -operator.getRawAxis(motorOneAxis),
                () -> -operator.getRawAxis(motorTwoAxis)
            )
        );

        // No default command for Claw

        // Configure the button bindings
        configureButtonBindings();

        // Add autonomous choices
        addAutonomousChoices();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        tagAligner.toggleOnTrue(new AlignWithApriltagOld(s_Swerve, () -> driver.getRawButton(tagAlignerExitButton)));

        tagAlignerNew.toggleOnTrue(new AlignWithApriltag(s_Swerve, doubleArm));

        autoBalance.toggleOnTrue(new AutoBalance(s_Swerve, doubleArm));

        terminateCommandsDriver.toggleOnTrue(new TerminateCommands(claw, doubleArm, s_Swerve));
        
        /* Operator Buttons */
        topGoal.toggleOnTrue(new SetArmPosition(doubleArm, topPositionAngles));
        midGoal.toggleOnTrue(new SetArmPosition(doubleArm, midPositionAngles));
        lowGoal.toggleOnTrue(new SetArmPosition(doubleArm, lowPositionAngles));
        idle.toggleOnTrue(new SetArmPosition(doubleArm, idlePositionAngles));

        intakeUpper.toggleOnTrue(new SetArmPosition(doubleArm, intakeUpperAngles));
        intakeLower.toggleOnTrue(new SetArmPosition(doubleArm, intakeLowerAngles));

        doneWithIntake.toggleOnTrue(new SetArmPositionAfterIntake(claw, doubleArm));

        stopArmCommands.onTrue(new TerminateArmCommands(doubleArm));

        intake.toggleOnTrue(new Intake(claw));
        outtake.toggleOnTrue(new Outtake(claw, doubleArm));
        
        terminateCommandsOperator.toggleOnTrue(new TerminateCommands(claw, doubleArm, s_Swerve));
    }

    private void addAutonomousChoices() {
        autonomousChooser.setDefaultOption(autos[0][0], autos[0][1]);
        for (int i = 1; i < autos.length; i++) {
            autonomousChooser.addOption(autos[i][0], autos[i][1]);
        }
        SmartDashboard.putData("Auto Selector", autonomousChooser);
    }

    public void teleopInit() {
        doubleArm.resetPID();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new Autonomous(claw, doubleArm, s_Swerve, autonomousChooser.getSelected());
    }
}
