package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class SwerveDriveTrain {

    private SwerveModule[] wheels; // RF, LF, RB, LB

    private Translation2d right_front_location = new Translation2d(wheel_to_center, wheel_to_center);
    private Translation2d left_front_location = new Translation2d(-wheel_to_center, wheel_to_center);
    private Translation2d right_back_location = new Translation2d(wheel_to_center, -wheel_to_center);
    private Translation2d left_back_location = new Translation2d(-wheel_to_center, -wheel_to_center);

    // only used for pose estimators
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(right_front_location, left_front_location,
            right_back_location, left_back_location);

    private SwerveDrivePoseEstimator poseEstimator;

    private ADIS16448_IMU gyro;
    private Pigeon2 imu;

    private final double starting_yaw;
    private double target_angle;

    public SwerveDriveTrain() {

        wheels = new SwerveModule[] {
                new SwerveModule(rf_direction, rf_driving, right_front_location),
                new SwerveModule(lf_direction, lf_driving, left_front_location),
                new SwerveModule(rb_direction, rb_driving, right_back_location),
                new SwerveModule(lb_direction, lb_driving, left_back_location)
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

        // these VecBuilders are basically how much to trust the trust state, local
        // measurement, and vision in respectively
        poseEstimator = new SwerveDrivePoseEstimator(new Rotation2d(starting_yaw), startingPose, kinematics,
                VecBuilder.fill(0.05, 0.05, 0.05), VecBuilder.fill(0.05), VecBuilder.fill(0.05, 0.05, 0.05));

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

    // TeleOp

    public void drive(double left_stick_x, double left_stick_y, double right_stick_x, double right_stick_y,
            double speed_factor, boolean makeX) {

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

        if (left_stick_magnitude == 0 && right_stick_magnitude == 0) { // both sticks are in dead zones, so we don't do
                                                                       // anything
            double error = normalizeAngle(target_angle - angle());

            if (Math.abs(error) < 0.02) { // if we are close enough to our target then brake
                brake();
                return;
            }

            double[] strafevector = { 0, 0 }; // otherwise we try to turn toward the target
            double turningfactor = Math.max(Math.min(error * 4.0, 1), -1);
            drive(strafevector, turningfactor, 1);
            return;
        }

        double[] strafevector = angleToVector(
                normalizeAngle(vectorToAngle(new double[] { left_stick_x, left_stick_y }) - angle()));
        strafevector[0] *= left_stick_magnitude;
        strafevector[1] *= left_stick_magnitude; // this handles it if left joystick is within the dead zone

        double turning_factor;

        if (absolute_directing) {
            turning_factor = Math
                    .max(Math.min(normalizeAngle(vectorToAngle(new double[] { right_stick_x, right_stick_y }) - angle())
                            * directing_sensitivity, 1), -1)
                    * right_stick_magnitude;
            // if the right button isn't pressed we don't want to turn
        } else {
            turning_factor = right_stick_x;
        }

        if (turning_factor == 0) {
            turning_factor = Math.max(Math.min(normalizeAngle(target_angle - angle()) * directing_sensitivity, 1), -1);
        } else {
            target_angle = angle();
        }

        double multiplier = Math.min(speed_factor, 1) / Math.sqrt(Math.max(Math.max(
                Math.max(wheels[0].calculateSquaredPower(strafevector, turning_factor),
                        wheels[1].calculateSquaredPower(strafevector, turning_factor)),
                Math.max(wheels[2].calculateSquaredPower(strafevector, turning_factor),
                        wheels[3].calculateSquaredPower(strafevector, turning_factor))),
                1.0));
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

    // Autonomous

    public void turnClockwise(double degrees) { // we can make this negative to turn clockwise
        turnToDegree(angle() * 180 / Math.PI + degrees);
    }

    public void turnToDegree(double degrees) { // we can make this negative as well
        // note that this doesn't do optimized direction, but it will always turn less
        // than 180 degrees
        double target_angle = normalizeAngle(degrees * Math.PI / 180.0);

        double error = normalizeAngle(target_angle - angle());
        while (error > 0.05) { // 2.8 degrees
            error = normalizeAngle(target_angle - angle());
            drive(new double[] { 0, 0 }, Math.max(Math.min(error * directing_sensitivity, 1), -1), 1);
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
            drive(angleToVector(normalizeAngle(vectorToAngle(vector) - angle())), 0, 0.01); // turn the wheels to our
                                                                                            // target position first
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
                drive(new double[] { 0, 1.0 }, 0, 0.1 * error / Math.abs(error)); // go forward or backward depending on
                                                                                  // direction of error
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

    public Rotation2d getRotation() {
        return new Rotation2d(angle());
    }

    /**
     * Updates the position using the swerve module encoders.
     * should be called once every loop
     */
    public void updatePose() {
        var moduleStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            moduleStates[i] = wheels[i].getState();
        }

        poseEstimator.update(getRotation(), moduleStates);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(pose, getRotation());
    }

    /**
     * Call this to update the pose using vision (apriltags, tf, etc.)
     * 
     * @param estimatedPose the Pose2d that the robot vision thinks it is at
     * @param timestamp     when the vision data was taken (in seconds for some
     *                      reason)
     */
    public void updatePoseWithVision(Pose2d estimatedPose, double timestamp) {
        poseEstimator.addVisionMeasurement(startingPose, f_pivot);
    }
}

class SwerveModule { // each module requires 2 Talon FX motors

    private TalonFX direction_motor;
    private TalonFX driving_motor;

    private final double x_;
    private final double y_;
    private final double tolerance = max_error * Math.PI / 180.0;
    private double[] target_motion_vector = { 0, 0 };
    private double target_turning_factor = 0;
    private double[] target_overall_vector = { 0, 0 };

    private double prevTime = System.nanoTime();
    private double prevEncoderVals;

    public SwerveModule(int direction_motor_port, int driving_motor_port, Translation2d position) {
        // position: relative position from the center of the circumcircle
        // units don't matter, they just have to be consistent
        direction_motor = new TalonFX(direction_motor_port);
        direction_motor.setSelectedSensorPosition(0, 0, 0);

        driving_motor = new TalonFX(driving_motor_port);

        prevEncoderVals = driving_motor.getSelectedSensorPosition();

        direction_motor.setNeutralMode(NeutralMode.Brake);
        driving_motor.setNeutralMode(NeutralMode.Brake);

        direction_motor.setInverted(false);
        driving_motor.setInverted(false);

        double radius = Math.sqrt(position.getX() * position.getX() + position.getY() * position.getY());

        x_ = position.getY() / radius; // ex, (1, 2) --> (2, -1); clockwise rotation
        y_ = 0 - position.getX() / radius;
    }

    public double calculateSquaredPower(double[] strafevector, double turnfactor) {
        double x = strafevector[0] + turnfactor * x_; // positive turnfactor -> rotate clockwise
        double y = strafevector[1] + turnfactor * y_;
        return x * x + y * y;
    }

    public void drive(double[] strafevector, double turnfactor, double multiplier) {
        double[] vector = { strafevector[0] + turnfactor * x_, strafevector[1] + turnfactor * y_ };
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
        double[] targetvector = { -y_, x_ };
        double targetangle = vectorToAngle(targetvector);
        double error = normalizeAngle(targetangle - getAngle());

        if (Math.abs(error) > Math.PI / 2.0)
            error += Math.PI * (error > 0 ? -1 : 1);
        if (Math.abs(error) > Math.PI / 2.0)
            error += Math.PI * (error > 0 ? -1 : 1);
        if (Math.abs(error) < 0.02)
            error = 0;

        direction_motor.set(ControlMode.PercentOutput,
                Math.abs(error) < 0.02 ? 0 : Math.max(Math.min(error * 4.0, 1), -1));
        direction_motor.set(ControlMode.PercentOutput, Math.max(Math.min(error * directing_motor_sensitivity, 1), -1));
        driving_motor.set(ControlMode.PercentOutput, 0);

        target_motion_vector[0] = 0;
        target_motion_vector[1] = 0;
        target_turning_factor = 0;
        target_overall_vector[0] = -y_;
        target_overall_vector[1] = x_;
    }

    public double[] getTargets() {
        return new double[] { target_motion_vector[0], target_motion_vector[1], target_turning_factor,
                target_overall_vector[0], target_overall_vector[1] };
    }

    private double getAngle() {
        return normalizeAngle(direction_motor.getSelectedSensorPosition(0) / ticks_per_radian);
    }

    private Rotation2d getRotation() {
        return new Rotation2d(getAngle());
    }

    private void setModule(double angle, double magnitude) {
        double error = normalizeAngle(angle - getAngle()); // power the wheel forward if error > 0

        int negate = 1;

        if (Math.abs(error) > Math.PI / 2.0) { // if we're more than 90 degrees away, invert angle and power
            if (Math.abs(error) > Math.PI / 2.0) { // if we're more than 90 degrees away, invert angle and power
                negate = -1;
                error += Math.PI * (error > 0 ? -1 : 1);
            }
            if (Math.abs(error) < 0.02) {
                error = 0;
            }

            direction_motor.set(ControlMode.PercentOutput,
                    Math.abs(error) < 0.02 ? 0 : Math.max(Math.min(error * 4.0, 1), -1));
            // max power at 0.25 radians
            direction_motor.set(ControlMode.PercentOutput,
                    Math.max(Math.min(error * directing_motor_sensitivity, 1), -1));
            driving_motor.set(ControlMode.PercentOutput, Math.abs(error) < tolerance ? 0 : magnitude * negate);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getRotation());
    }

    private double getSpeed() {
        double currentEncoderVal = driving_motor.getSelectedSensorPosition();
        double currentTime = System.nanoTime();

        // encoder units per nano second
        double speed = (currentEncoderVal - prevEncoderVals) / (currentTime - prevTime);

        speed *= 1_000_000_000.0; // convert to encoder units per second

        speed /= 2048.0; // convert to rotations per second

        speed /= 8.14; // gear ratio

        speed *= 2 * Math.PI * wheel_radius; // convert to meters per second

        prevTime = currentTime;
        prevEncoderVals = currentEncoderVal;

        return speed;

    }
}
