package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class SwerveDriveTrain {

    private SwerveModule[] wheels; // RF, LF, RB, LB
    private ADIS16448_IMU gyro;
    private Pigeon2 imu;

    private final double starting_yaw;
    private double target_angle;

    public SwerveDriveTrain() {
        
        wheels = new SwerveModule[] {
            new SwerveModule(rf_direction, rf_driving, new double[] {1, 1}), 
            new SwerveModule(lf_direction, lf_driving, new double[] {-1, 1}), 
            new SwerveModule(rb_direction, rb_driving, new double[] {1, -1}), 
            new SwerveModule(lb_direction, lb_driving, new double[] {-1, -1})
        };

        if (use_pigeon2) {
            imu = new Pigeon2(imu_port);
            imu.addYaw(0 - imu.getYaw());
            starting_yaw = imu.getYaw() * Math.PI / 180.0; // .getRoll(), .getPitch() or .getYaw()
        } else if (use_ADIS16448) {
            gyro = new ADIS16448_IMU();
            gyro.calibrate();
            gyro.reset();
            starting_yaw = gyro.getAngle() * Math.PI / 180.0;
                                // .getAngle(), .getGyroAngleX(), .getGyroAngleY(), or .getGyroAngleZ()
        } else {
            starting_yaw = 0;
        }
        target_angle = angle();
    }

    public double angle() {
        if (use_pigeon2) { // clockwise increases angle() and yaw
            return normalizeAngle(imu.getYaw() * Math.PI / 180.0 - starting_yaw);
        } else if (use_ADIS16448) {
            return normalizeAngle(gyro.getAngle() * Math.PI / 180.0 - starting_yaw);
        } else {
            return 0; // robot acts as if it's always pointing straight ahead
        }
    }

    public double[][] getWheelData() {
        return new double[][] {
            wheels[0].getTargets(), wheels[1].getTargets(), wheels[2].getTargets(), wheels[3].getTargets()
        };
    }

    //TeleOp

    public void drive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y, double speed_factor, boolean makeX) {
        
        if (makeX) {
            makeX();
            return;
        }

        double left_stick_magnitude = Math.sqrt(left_stick_x * left_stick_x + left_stick_y * left_stick_y);
        double right_stick_magnitude = Math.sqrt(right_stick_x * right_stick_x + right_stick_y * right_stick_y);

        if (left_stick_magnitude < joystick_deadzone) {
            left_stick_magnitude = 0;
        }
        if (right_stick_magnitude < joystick_deadzone) {
            right_stick_magnitude = 0;
            right_stick_x = 0;
        }

        if (left_stick_magnitude == 0 && right_stick_magnitude == 0) { // both sticks are in dead zones, so we don't do anything
            double error = normalizeAngle(target_angle - angle()); 

            if (Math.abs(error) < 0.02) { // if we are close enough to our target then brake
                brake();
                return;
            }

            double[] strafevector = {0, 0}; // otherwise we try to turn toward the target
            double turningfactor = Math.max(Math.min(error * 4.0, 1), -1);
            drive(strafevector, turningfactor, 1);
            return;
        }
        
        double[] strafevector = angleToVector(normalizeAngle(vectorToAngle(new double[] {left_stick_x, left_stick_y}) - angle()));
        strafevector[0] *= left_stick_magnitude;
        strafevector[1] *= left_stick_magnitude; // this handles it if left joystick is within the dead zone

        double turning_factor;

        if (absolute_directing) {
            turning_factor = Math.max(Math.min(normalizeAngle(vectorToAngle(new double[] {right_stick_x, right_stick_y}) - angle()) * directing_sensitivity, 1), -1) * right_stick_magnitude;
                // if the right button isn't pressed we don't want to turn
        } else {
            turning_factor = right_stick_x;
        }

        if (turning_factor == 0) {
            turning_factor = Math.max(Math.min(normalizeAngle(target_angle - angle()) * directing_sensitivity, 1), -1);
        } else {
            target_angle = angle();
        }

        double multiplier = Math.min(speed_factor, 1) / Math.sqrt(Math.max(Math.max(Math.max(wheels[0].calculateSquaredPower(strafevector, turning_factor), wheels[1].calculateSquaredPower(strafevector, turning_factor)), Math.max(wheels[2].calculateSquaredPower(strafevector, turning_factor), wheels[3].calculateSquaredPower(strafevector, turning_factor))), 1.0));
        drive(strafevector, turning_factor, multiplier);
    }

    public void makeX() {
        for (int i = 0; i < 4; i++) {
            wheels[i].makeX();
        }
    }

    public void brake() {
        for (int i = 0; i < 4; i++) {
            wheels[i].brake();
        }
    }

    private void drive(double[] strafevector, double turningfactor, double multiplier) {
        for (int i = 0; i < 4; i++) {
            wheels[i].drive(strafevector, turningfactor, multiplier);
        }
    }

    //Autonomous

    public void turnClockwise(double degrees) { // we can make this negative to turn clockwise
        turnToDegree(angle() * 180 / Math.PI + degrees);
    }

    public void turnToDegree(double degrees) { // we can make this negative as well
            // note that this doesn't do optimized direction, but it will always turn less than 180 degrees
        double target_angle = normalizeAngle(degrees * Math.PI / 180.0);

        double error = normalizeAngle(target_angle - angle());
        while (error > 0.05) { // 2.8 degrees
            error = normalizeAngle(target_angle - angle());
            drive(new double[] {0, 0}, Math.max(Math.min(error * directing_sensitivity, 1), -1), 1);
        }
        
        brake();
    }

    public void strafe(double[] vector, double power, double time) {
        strafe(vector, power);
        
        pause(time);

        brake();
    }

    public void strafe(double[] vector, double power) {
        double start_time = System.nanoTime() / 1000000000.0;

        while (System.nanoTime() / 1000000000.0 < start_time + 0.1) {
            drive(angleToVector(normalizeAngle(vectorToAngle(vector) - angle())), 0, 0.01); // turn the wheels to our target position first
        }
        brake();

        pause(0.1);

        drive(angleToVector(normalizeAngle(vectorToAngle(vector) - angle())), 0, power);
    } // what we can do is have the drive function stop once we cross a specific limit

    // Game-Specific Commands

    public void autoBalance() {
        turnToDegree(0); // point straight ahead

        boolean run = true;
        while (run) {
            double error;
            if (use_pigeon2) {
                error = imu.getRoll();
            } else {
                error = gyro.getGyroAngleX(); // I think this is roll but I don't know for sure
            }
            if (Math.abs(error) > 1.5) { // if we are more than 1.5 degrees off
                drive(new double[] {0, 1.0}, 0, 0.1 * error / Math.abs(error)); // go forward or backward depending on direction of error
            } else {
                run = false;
            }
        }

        double start_time = System.nanoTime() / 1000000000.0;
        while (System.nanoTime() / 1000000000.0 < start_time + 0.5) {
            makeX();
        }
        brake();
    }

}

class SwerveModule { // each module requires 2 Talon FX motors

    private TalonFX direction_motor;
    private TalonFX driving_motor;

    private final double x_;
    private final double y_;
    private final double tolerance = max_error * Math.PI / 180.0;

    private double[] target_motion_vector = {0, 0};
    private double target_turning_factor = 0;
    private double[] target_overall_vector = {0, 0};

    public SwerveModule(int direction_motor_port, int driving_motor_port, double[] position) {
                                    // position: relative position from the center of the circumcircle
                                    // units don't matter, they just have to be consistent
        direction_motor = new TalonFX(direction_motor_port);
        direction_motor.setSelectedSensorPosition(0, 0, 0);

        driving_motor = new TalonFX(driving_motor_port);

        direction_motor.setNeutralMode(NeutralMode.Brake);
        driving_motor.setNeutralMode(NeutralMode.Brake);

        direction_motor.setInverted(false);
        driving_motor.setInverted(false);

        double radius = Math.sqrt(position[0] * position[0] + position[1] * position[1]);

        x_ = position[1] / radius; //ex, (1, 2) --> (2, -1); clockwise rotation
        y_ = 0 - position[0] / radius;
    }

    public double calculateSquaredPower(double[] strafevector, double turnfactor) {
        double x = strafevector[0] + turnfactor * x_; // positive turnfactor -> rotate clockwise
        double y = strafevector[1] + turnfactor * y_;
        return x * x + y * y;
    }

    public void drive(double[] strafevector, double turnfactor, double multiplier) {
        double[] vector = {strafevector[0] + turnfactor * x_, strafevector[1] + turnfactor * y_};
        vector[0] *= multiplier;
        vector[1] *= multiplier;

        target_motion_vector[0] = strafevector[0] * multiplier;
        target_motion_vector[1] = strafevector[1] * multiplier;
        target_turning_factor = turnfactor * multiplier;
        target_overall_vector[0] = vector[0];
        target_overall_vector[1] = vector[1];

        setModule(vectorToAngle(vector), Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1]));
    }

    public void brake() {
        direction_motor.set(ControlMode.PercentOutput, 0);
        driving_motor.set(ControlMode.PercentOutput, 0);

        target_motion_vector[0] = 0;
        target_motion_vector[1] = 0;
        target_turning_factor = 0;
        target_overall_vector[0] = 0;
        target_overall_vector[1] = 0;
    }

    public void makeX() {
        double error = normalizeAngle(vectorToAngle(new double[] {-y_, x_}) - getAngle());

        if (Math.abs(error) > Math.PI / 2.0) error += Math.PI * (error > 0 ? -1 : 1);
        if (Math.abs(error) < 0.02) error = 0;

        direction_motor.set(ControlMode.PercentOutput, Math.max(Math.min(error * directing_motor_sensitivity, 1), -1));
        driving_motor.set(ControlMode.PercentOutput, 0);

        target_motion_vector[0] = 0;
        target_motion_vector[1] = 0;
        target_turning_factor = 0;
        target_overall_vector[0] = -y_;
        target_overall_vector[1] = x_;
    }

    public double[] getTargets() {
        return new double[] {target_motion_vector[0], target_motion_vector[1], target_turning_factor, target_overall_vector[0], target_overall_vector[1]};
    }

    private double getAngle() {
        return normalizeAngle(direction_motor.getSelectedSensorPosition(0) / ticks_per_radian);
    }

    private void setModule(double angle, double magnitude) {
        double error = normalizeAngle(angle - getAngle()); // power the wheel forward if error > 0

        int negate = 1;

        if (Math.abs(error) > Math.PI / 2.0) { // if we're more than 90 degrees away, invert angle and power
            negate = -1;
            error += Math.PI * (error > 0 ? -1 : 1);
        }
        if (Math.abs(error) < 0.02) {
            error = 0;
        }

        direction_motor.set(ControlMode.PercentOutput, Math.max(Math.min(error * directing_motor_sensitivity, 1), -1));
        driving_motor.set(ControlMode.PercentOutput, Math.abs(error) < tolerance ? 0 : magnitude * negate);
    }
}