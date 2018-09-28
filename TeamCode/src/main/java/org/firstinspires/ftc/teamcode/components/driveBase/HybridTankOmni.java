package org.firstinspires.ftc.teamcode.components.driveBase;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

// keep in mind that the center of rotation/the robot will be considered as in between the drive wheels
class HybridTankOmni extends DriveBase {
    DcMotor left;
    DcMotor right;
    Telemetry telemetry;

    public HybridTankOmni(DcMotor left, DcMotor right, Telemetry telemetry) {
        this.left = left;
        this.right = right;
        this.telemetry = telemetry;
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
}
