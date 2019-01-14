package org.firstinspires.ftc.teamcode.components.driveBase;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
@Config
public class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

    public static double MAX_TURN_POWER = 0.65f;
    public static double GYRO_TURN_CLAMP_CUTOFF_DEGREES = 90;
    public static double MAX_FORWARD_POWER = 0.4f;
    public static double MIN_FORWARD_POWER = 0.27f;
    float MAX_FORWARD_POWER_DIRECT = 0.9f;
    public static double MIN_TURN_POWER = 0.28f;
    public static double MAX_FORWARD_TURN_ADJUST_POWER = 0.8f;
    public static double FORWARD_CLAMP_CUTOFF_TICKS = 200;
    int ENCODER_EPSILON = 20;
    public static double TICKS_PER_MM = -2.15; // TODO: SET
    private float TICKS_PER_DEGREE = 6;


    int ANGLE_EPSILON = 2;
    float ANGLE_DERIV_EPSILON = 0.001f;
    public static double TURN_P_COEFF = 0.007;
    public static double TURN_D_COEFF = 1.45;
    public static double TURN_I_COEFF = 0;

    int MOTOR_TIMEOUT_MS = 1000;
    int ENCODER_TICKS_TIMEOUT_THRESHOLD = 10;

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
        set_mode_motors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float targetHeading = imu.getHeading() + r;
        float headingError = -getHeadingError(targetHeading, imu);
        float headingErrorDerivative = 0;
        float previousHeadingError = -getHeadingError(targetHeading, imu);

        telemetry.addLine("Run IMU turn");
        telemetry.update();

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;
        long previous_time_millis = System.currentTimeMillis();
        float headingErrorIntegral = 0;

        do {
            // PD Control (Integral not necessary - may cause windup)
            headingError = -getHeadingError(targetHeading, imu);
            long current_time_millis = System.currentTimeMillis();
            long time_change_millis = current_time_millis - previous_time_millis;
            previous_time_millis = current_time_millis;
            float heading_error_change = headingError - previousHeadingError;
            previousHeadingError = headingError;
            headingErrorDerivative = heading_error_change / time_change_millis;
            headingErrorIntegral += heading_error_change * time_change_millis;


            float v = ExtendedMath.clamp(
                    -(float)MAX_TURN_POWER,
                    (float)MAX_TURN_POWER,
                    Math.signum(headingError) * (float)MIN_TURN_POWER + headingError * (float)TURN_P_COEFF + headingErrorDerivative * (float)TURN_D_COEFF + headingErrorIntegral * (float)TURN_I_COEFF);

            telemetry.addData("headingError", headingError);
            telemetry.addData("headingErrorDerivative", headingErrorDerivative);
            telemetry.addData("v", v);
            telemetry.update();

            left.setPower(-v);
            right.setPower(v);

            // timeout
            if (System.currentTimeMillis() >= next_check_timestamp) {
                int right_current = right.getCurrentPosition();
                int left_current = left.getCurrentPosition();
                if ((Math.abs(right_current - last_right) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(left_current - last_left) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine("Turn timeout");
                    telemetry.update();
                    break;
                }
                last_right = right_current;
                last_left = left_current;
                next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;
            }
        } while (Globals.OPMODE_ACTIVE.get() && (Math.abs(headingError) > ANGLE_EPSILON || Math.abs(headingErrorDerivative) > ANGLE_DERIV_EPSILON));
        stop();
    }

    private float getHeadingError(float targetHeading, InertialSensor imu) {
        float positiveError = ExtendedMath.get_min_rotation(imu.getHeading(), targetHeading);
        // now normalize to +-180 for convenience
        return positiveError + (positiveError > 180 ? -360 : 0);
    }

    // x in mm
    public void imu_forward_move(float x, InertialSensor imu) {
        set_mode_motors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int targetMovement = (int)(x * TICKS_PER_MM);
        int rightTarget = right.getCurrentPosition() + targetMovement;
        int leftTarget = left.getCurrentPosition() + targetMovement;
        float targetHeading = imu.getHeading();
        float rightError, leftError, headingError;

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;

        do {
            headingError = getHeadingError(targetHeading, imu);
            rightError = right.getCurrentPosition() - rightTarget;
            leftError = left.getCurrentPosition() - leftTarget;
            float heading_adjust_v = ExtendedMath.clamp(-(float)MAX_FORWARD_TURN_ADJUST_POWER,
                    (float)MAX_FORWARD_TURN_ADJUST_POWER,
                    headingError * ((float)MAX_FORWARD_TURN_ADJUST_POWER / (float)GYRO_TURN_CLAMP_CUTOFF_DEGREES));
            float right_v = ExtendedMath.clamp(-(float)MAX_FORWARD_POWER,
                    (float)MAX_FORWARD_POWER,
                    rightError * ((float)MAX_FORWARD_POWER / (float)FORWARD_CLAMP_CUTOFF_TICKS));
            float left_v = ExtendedMath.clamp(-(float)MAX_FORWARD_POWER,
                    (float)MAX_FORWARD_POWER,
                    leftError * ((float)MAX_FORWARD_POWER / (float)FORWARD_CLAMP_CUTOFF_TICKS));
            telemetry.addData("heading adjust", heading_adjust_v);
            telemetry.addData("right v", right_v);
            telemetry.addData("left v", left_v);
            telemetry.addData("right error", rightError);
            telemetry.addData("left error", leftError);
            telemetry.update();

            float left_power = left_v + heading_adjust_v;
            float right_power = right_v - heading_adjust_v;
            left.setPower(left_power + MIN_FORWARD_POWER * Math.signum(left_power));
            right.setPower(right_power + MIN_FORWARD_POWER * Math.signum(right_power));

            if (System.currentTimeMillis() >= next_check_timestamp) {
                int right_current = right.getCurrentPosition();
                int left_current = left.getCurrentPosition();
                if ((Math.abs(right_current - last_right) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(left_current - last_left) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine("Forward move timeout");
                    telemetry.update();
                    break;
                }

                last_right = right_current;
                last_left = left_current;
                next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;
            }

        } while (Globals.OPMODE_ACTIVE.get() && Math.abs(rightError) > ENCODER_EPSILON || Math.abs(leftError) > ENCODER_EPSILON);
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

        while (Globals.OPMODE_ACTIVE.get() && right.isBusy() || left.isBusy()) {
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

        while (Globals.OPMODE_ACTIVE.get() && right.isBusy() || left.isBusy()) {
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
        set_mode_motors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double turn_contrib = Math.abs(r);
        double throttle_contrib = 1 - turn_contrib;
        left.setPower(MAX_FORWARD_POWER_DIRECT * (x * throttle_contrib - r));
        right.setPower(MAX_FORWARD_POWER_DIRECT * (x * throttle_contrib + r));
    }

    // shared
    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    public void report_encoder_ticks() {
        telemetry.addData("DRIVE LEFT TICKS", left.getCurrentPosition());
        telemetry.addData("DRIVE RIGHT TICKS", right.getCurrentPosition());
        telemetry.addData("DRIVE LEFT POWER", left.getPower());
        telemetry.addData("DRIVE RIGHT POWER", right.getPower());
    }

    private void set_mode_motors(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
    }
}
