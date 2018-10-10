package org.firstinspires.ftc.teamcode.components.driveBase;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

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

    public void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {

    }

    // TODO: remove if unused (backup in case strafe can't be trusted)
    public void rotate_and_move_only_vertical_drive(float initial_rotation, float movement, float final_rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {

    }

    public void move(float x, float y) {

    }

    public void turn(float r) {

    }

    public void imu_turn(float r, InertialSensor imu) {

    }

    public void move_and_turn(float x, float y, float r) {

    }

    public void stop() {

    }

    public void report_encoder_ticks() {

    }

    private void set_mode_motors(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
    }
}
