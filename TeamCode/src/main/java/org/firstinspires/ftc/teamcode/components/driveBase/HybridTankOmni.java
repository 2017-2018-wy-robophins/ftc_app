package org.firstinspires.ftc.teamcode.components.driveBase;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.PIDController;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
@Config
public class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

    public static double MAX_TURN_POWER = 0.9;
    public static double GYRO_TURN_CLAMP_CUTOFF_DEGREES = 90;
    public static double MAX_FORWARD_POWER = 0.5f;
    public static double MIN_FORWARD_POWER = 0.27f;
    float MAX_FORWARD_POWER_DIRECT = 0.9f;
    public static double MIN_TURN_POWER = 0.32;
    public static double MAX_FORWARD_TURN_ADJUST_POWER = 0.8f;
    public static double FORWARD_CLAMP_CUTOFF_TICKS = 200;
    int ENCODER_EPSILON = 10;
    public static double TICKS_PER_MM = -2.15; // TODO: SET
    private float TICKS_PER_DEGREE = 6;


    float ANGLE_EPSILON = 3;
    float ANGLE_DERIV_EPSILON = 0.002f;
    float ENCODER_DERIV_EPSILON = 0;
    public static double TURN_P_COEFF = 0.02;
    public static double TURN_D_COEFF = 0.0039;
    public static double TURN_I_COEFF = 1.8;

    public static double TURN_ADJUST_P_COEFF = 0.06;
    public static double TURN_ADJUST_D_COEFF = 0;
    public static double TURN_ADJUST_I_COEFF = 0;

    public static double FORWARD_P_COEFF = 0.02;
    public static double FORWARD_D_COEFF = 0;
    public static double FORWARD_I_COEFF = 2.5;

    public static int TURN_OSCILLATION_CUTOFF = 2;

    int MOTOR_TIMEOUT_MS = 1000;
    int ENCODER_TICKS_TIMEOUT_THRESHOLD = 30;

    public HybridTankOmni(DcMotor left, DcMotor right, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.left = left;
        this.right = right;

        set_mode_motors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
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

        telemetry.addLine("Run IMU turn");
        telemetry.update();

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;
        long previous_time_millis = System.currentTimeMillis();

        float previousHeadingError = 0;
        float numOscillations = 0;
        PIDController headingPID = new PIDController((float)TURN_P_COEFF, (float)TURN_D_COEFF, (float)TURN_I_COEFF, -getHeadingError(targetHeading, imu));

        while (Globals.OPMODE_ACTIVE.get() && (Math.abs(headingPID.getError()) > ANGLE_EPSILON || Math.abs(headingPID.getErrorDerivative()) > ANGLE_DERIV_EPSILON)) {
            // PD Control (Integral not necessary - may cause windup)
            long current_time_millis = System.currentTimeMillis();
            long time_change_millis = current_time_millis - previous_time_millis;
            previous_time_millis = current_time_millis;
            float headingError = -getHeadingError(targetHeading, imu);
            headingPID.updateError(headingError, time_change_millis);

            float v = ExtendedMath.clamp(
                    -(float)MAX_TURN_POWER,
                    (float)MAX_TURN_POWER,
                    Math.signum(headingError) * (float)MIN_TURN_POWER + headingPID.getValue());

            telemetry.addData("headingError", headingPID.getError());
            telemetry.addData("headingErrorDerivative", headingPID.getErrorDerivative());
            telemetry.addData("v", v);
            telemetry.addData("num oscillations", numOscillations);
            telemetry.update();

            left.setPower(v);
            right.setPower(-v);

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

            if (roundedSignum(previousHeadingError) != roundedSignum(headingError)) {
                numOscillations++;
            }

            if (numOscillations >= TURN_OSCILLATION_CUTOFF + 1) {
                break;
            }

            previousHeadingError = headingError;
        }
        stop();
    }

    private float roundedSignum(float x) {
        float s = Math.signum(x);
        if (s >= 0) {
            return 1;
        } else {
            return 0;
        }
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

        long previous_time_millis = System.currentTimeMillis();

        int last_right = right.getCurrentPosition();
        int last_left = left.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + MOTOR_TIMEOUT_MS;

        PIDController headingAdjustPID = new PIDController((float)TURN_ADJUST_P_COEFF, (float)TURN_ADJUST_D_COEFF, (float)TURN_ADJUST_I_COEFF, getHeadingError(targetHeading, imu));
        PIDController rightPID = new PIDController((float)FORWARD_P_COEFF, (float)FORWARD_D_COEFF, (float)FORWARD_I_COEFF, right.getCurrentPosition() - rightTarget);
        PIDController leftPID = new PIDController((float)FORWARD_P_COEFF, (float)FORWARD_D_COEFF, (float)FORWARD_I_COEFF, left.getCurrentPosition() - leftTarget);

        do {
            float headingError = getHeadingError(targetHeading, imu);
            long current_time_millis = System.currentTimeMillis();
            long time_change_millis = current_time_millis - previous_time_millis;
            previous_time_millis = current_time_millis;
            headingAdjustPID.updateError(headingError, time_change_millis);
            rightPID.updateError(right.getCurrentPosition() - rightTarget, time_change_millis);
            leftPID.updateError(left.getCurrentPosition() - leftTarget, time_change_millis);

            float heading_adjust_v = ExtendedMath.clamp(-(float)MAX_FORWARD_TURN_ADJUST_POWER, (float)MAX_FORWARD_TURN_ADJUST_POWER, headingAdjustPID.getValue());
            float right_v = ExtendedMath.clamp(-(float)MAX_FORWARD_POWER, (float)MAX_FORWARD_POWER, rightPID.getValue());
            float left_v = ExtendedMath.clamp(-(float)MAX_FORWARD_POWER, (float)MAX_FORWARD_POWER, leftPID.getValue());

            telemetry.addData("heading adjust", heading_adjust_v);
            telemetry.addData("right v", right_v);
            telemetry.addData("left v", left_v);
            telemetry.addData("heading error", headingAdjustPID.getError());
            telemetry.addData("right error", rightPID.getError());
            telemetry.addData("left error", leftPID.getError());
            telemetry.update();

            float left_power = left_v - heading_adjust_v;
            float right_power = right_v + heading_adjust_v;
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

        } while (Globals.OPMODE_ACTIVE.get() &&
                Math.abs(rightPID.getError()) > ENCODER_EPSILON && Math.abs(leftPID.getError()) > ENCODER_EPSILON);

        stop();
    }

    // direct control functions
    // x = [-1, 1], r = [-1, 1]
    public void direct_move_and_turn(float x, float r) {
        set_mode_motors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double turn_contrib = Math.abs(r);
        double throttle_contrib = 1 - turn_contrib;
        left.setPower(MAX_FORWARD_POWER_DIRECT * (x * throttle_contrib + r));
        right.setPower(MAX_FORWARD_POWER_DIRECT * (x * throttle_contrib - r));
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
