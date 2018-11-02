package org.firstinspires.ftc.teamcode.components.driveBase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

    float MAX_TURN_POWER = 0.8f;
    float GYRO_TURN_CLAMP_CUTOFF_DEGREES = 10;
    float MAX_FORWARD_POWER = 0.8f;
    float MAX_FORWARD_TURN_ADJUST_POWER = 0.1f;
    float FORWARD_CLAMP_CUTOFF_TICKS = 100;
    int ENCODER_EPSILON = 20;
    float TICKS_PER_MM = 1/100; // TODO: SET
    int ANGLE_EPSILON = 1;
    int MOTOR_TIMEOUT_MS = 1000;

    public HybridTankOmni(DcMotor left, DcMotor right, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.left = left;
        this.right = right;

        set_mode_motors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

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
            left.setPower(v);
            right.setPower(-v);
        } while (Math.abs(headingError) > ANGLE_EPSILON);
        stop();
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
            left.setPower(left_v + heading_adjust_v);
            right.setPower(right_v - heading_adjust_v);
        } while (Math.abs(rightError) > ENCODER_EPSILON || Math.abs(leftError) > ENCODER_EPSILON);
        stop();
    }

    // direct control functions
    // x = [-1, 1], r = [-1, 1]
    public void direct_move_and_turn(float x, float r) {
        set_mode_motors(DcMotor.RunMode.RUN_USING_ENCODER);
        double turn_contrib = Math.abs(r);
        double throttle_contrib = 1 - turn_contrib;
        left.setPower(MAX_FORWARD_POWER * (x * throttle_contrib + r));
        right.setPower(MAX_FORWARD_POWER * (x * throttle_contrib - r));
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
