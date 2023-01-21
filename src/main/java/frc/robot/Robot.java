package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Odometry;

import frc.robot.subsystems.SwerveDriveTrain;

import static frc.robot.Constants.*;

public class Robot extends TimedRobot {

  private SwerveDriveTrain swerve;
  private XboxController driver;
  private Odometry odom;

  @Override
  public void robotInit() {
    swerve = new SwerveDriveTrain();
    odom = new Odometry();
  }

  @Override
  public void robotPeriodic() {
    odom.update();
    if (show_data) {
      double[][] temp = swerve.getWheelData();
      SmartDashboard.putNumber("angle", swerve.angle());
      SmartDashboard.putNumberArray("right_front", temp[0]);
        //motion vector x, motion vector y, turning factor, overall vector x, overall vector y
      SmartDashboard.putNumberArray("left_front", temp[1]);
      SmartDashboard.putNumberArray("right_back", temp[2]);
      SmartDashboard.putNumberArray("left_back", temp[3]);

      SmartDashboard.putNumberArray("x_pos_y_pos_angle", odom.getPosition());
      SmartDashboard.updateValues();
    }
  }

  @Override
  public void autonomousInit() {
    // note these will only work if we have imu
    swerve.turnClockwise(45);
    pause(0.5);
    swerve.turnToDegree(-90);
    pause(0.5);
    swerve.turnToDegree(180);
    pause(0.5);
    swerve.strafe(new double[] {1, 0}, 0.3, 5);
    pause(0.5);
    swerve.strafe(new double[] {-1, 1}, 0.3, 5);
    pause(0.5);
  }

  @Override
  public void teleopInit() {
    driver = new XboxController(0);
  }

  @Override
  public void teleopPeriodic() { // driving code
    swerve.drive(driver.getLeftX(), 0 - driver.getLeftY(), driver.getRightX(), 0 - driver.getRightY(), 1 - 0.5 * driver.getRightTriggerAxis(), driver.getAButton());
  }
}