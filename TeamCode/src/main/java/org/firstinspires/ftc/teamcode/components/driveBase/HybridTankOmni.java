package org.firstinspires.ftc.teamcode.components.driveBase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

import static org.firstinspires.ftc.teamcode.common.RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.common.RobotConstants.TICK_ALLOWED_ABS_ERROR;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
public class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

    float MAX_TURN_POWER = 0.2f;
    float GYRO_TURN_CLAMP_CUTOFF_DEGREES = 20;
    float MAX_FORWARD_POWER = 0.2f;
    float MAX_FORWARD_TURN_ADJUST_POWER = 0.1f;
    float FORWARD_CLAMP_CUTOFF_TICKS = 100;
    int ENCODER_EPSILON = 20;
    float TICKS_PER_MM = -2; // TODO: SET
    private float TICKS_PER_DEGREE = 6;
    int ANGLE_EPSILON = 1;
    int MOTOR_TIMEOUT_MS = 1000;

    public HybridTankOmni(DcMotor left, DcMotor right, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.left = left;
        this.right = right;

        set_mode_motors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized Hybrid Tank/Omni Drivebase");
        telemetry.update();
    }

    // TODO: maybe async?
    // autonomous control functions

    // r in degrees - relative turn
    public void imu_turn(float r, InertialSensor imu) {
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        float targetHeading = imu.getHeading() + r;
        float headingError;
        do {
            headingError = targetHeading - imu.getHeading();
            float v = ExtendedMath.clamp(-MAX_TURN_POWER, MAX_TURN_POWER, headingError * (MAX_TURN_POWER / GYRO_TURN_CLAMP_CUTOFF_DEGREES));
            left.setPower(-v);
            right.setPower(v);
        } while (Globals.OPMODE_ACTIVE && Math.abs(headingError) > ANGLE_EPSILON);
        stop();
    }


    public void turn(float r) throws InterruptedException {
        set_mode_motors(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = (int)(r * TICKS_PER_DEGREE);
        int left_target = left.getCurrentPosition() - ticks;
        int right_target = right.getCurrentPosition() + ticks;

        left.setTargetPosition(left_target);
        right.setTargetPosition(right_target);
        left.setPower(MAX_TURN_POWER);
        right.setPower(MAX_TURN_POWER);
        long timeout = 1000;

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (Globals.OPMODE_ACTIVE && right.isBusy() || left.isBusy()) {
            int right_current = right.getCurrentPosition();
            int left_current = left.getCurrentPosition();
            telemetry.addData("Left Target", left_target);
            telemetry.addData("Right Target", right_target);
            telemetry.addData("Left Current", left_current);
            telemetry.addData("Right Current", right_current);
            telemetry.update();

            if ((Math.abs(right_current - right_target) < ENCODER_EPSILON) &&
                    (Math.abs(left_current - left_target) < ENCODER_EPSILON)) {
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(right_current - last_right) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(left_current - last_left) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    break;
                }

                last_right = right_current;
                last_left = left_current;
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(5);
        }

        right.setPower(0);
        left.setPower(0);
    }

    // x in mm
    public void imu_forward_move(float x, InertialSensor imu) {
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        int targetMovement = (int)(x * TICKS_PER_MM);
        int rightTarget = right.getCurrentPosition() + targetMovement;
        int leftTarget = left.getCurrentPosition() + targetMovement;
        float targetHeading = imu.getHeading();
        float rightError, leftError, headingError;
        do {
            headingError = targetHeading - imu.getHeading();
            rightError = rightTarget - right.getCurrentPosition();
            leftError = leftTarget - left.getCurrentPosition();
            float heading_adjust_v = ExtendedMath.clamp(-MAX_FORWARD_TURN_ADJUST_POWER,
                    MAX_FORWARD_TURN_ADJUST_POWER,
                    headingError * (MAX_FORWARD_TURN_ADJUST_POWER / GYRO_TURN_CLAMP_CUTOFF_DEGREES));
            float right_v = ExtendedMath.clamp(-MAX_FORWARD_POWER,
                    MAX_FORWARD_POWER,
                    rightError * (MAX_FORWARD_POWER / FORWARD_CLAMP_CUTOFF_TICKS));
            float left_v = ExtendedMath.clamp(-MAX_FORWARD_POWER,
                    MAX_FORWARD_POWER,
                    leftError * (MAX_FORWARD_POWER / FORWARD_CLAMP_CUTOFF_TICKS));
            left.setPower(left_v - heading_adjust_v);
            right.setPower(right_v + heading_adjust_v);
        } while (Globals.OPMODE_ACTIVE && Math.abs(rightError) > ENCODER_EPSILON || Math.abs(leftError) > ENCODER_EPSILON);
        stop();
    }

    public void forward_move(float x) throws InterruptedException {
        set_mode_motors(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = (int)(x * TICKS_PER_MM);
        int left_target = left.getCurrentPosition() + ticks;
        int right_target = right.getCurrentPosition() + ticks;
        left.setTargetPosition(left_target);
        right.setTargetPosition(right_target);

        left.setPower(MAX_FORWARD_POWER);
        right.setPower(MAX_FORWARD_POWER);
        long timeout = 1000;

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (Globals.OPMODE_ACTIVE && right.isBusy() || left.isBusy()) {
            int right_current = right.getCurrentPosition();
            int left_current = left.getCurrentPosition();

            if ((Math.abs(right_current - right_target) < ENCODER_EPSILON) &&
                    (Math.abs(left_current - left_target) < ENCODER_EPSILON)) {
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(right_current - last_right) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(left_current - last_left) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    break;
                }

                last_right = right_current;
                last_left = left_current;
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(5);
        }

        right.setPower(0);
        left.setPower(0);
    }

    // direct control functions
    // x = [-1, 1], r = [-1, 1]
    public void direct_move_and_turn(float x, float r) {
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        double turn_contrib = Math.abs(r);
        double throttle_contrib = 1 - turn_contrib;
        left.setPower(MAX_FORWARD_POWER * (x * throttle_contrib - r));
        right.setPower(MAX_FORWARD_POWER * (x * throttle_contrib + r));
    }

    // shared
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public void report_encoder_ticks() {
        telemetry.addData("DRIVE LEFT TICKS", left.getCurrentPosition());
        telemetry.addData("DRIVE RIGHT TICKS", right.getCurrentPosition());
    }

    private void set_mode_motors(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
    }
}
