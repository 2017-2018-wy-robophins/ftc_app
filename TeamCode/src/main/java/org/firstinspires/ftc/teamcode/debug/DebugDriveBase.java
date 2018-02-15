package org.firstinspires.ftc.teamcode.debug;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.drive_base.DriveBase;

/**
 * Created by efyang on 2/13/18.
 */

public class DebugDriveBase extends DriveBase {
    private final Telemetry telemetry;

    public DebugDriveBase(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        telemetry.addLine("Running function: move_by_vector_and_rotation");
        telemetry.addData("Movement (mm)", movement);
        telemetry.addData("Rotation (degrees)", rotation);
        telemetry.update();
        Thread.sleep(500);
    }

    public void rotate_and_move_only_vertical_drive(float initial_rotation, float movement, float final_rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        telemetry.addLine("Running function: rotate_and_move_only_vertical_drive");
        telemetry.addData("Initial Rotation (degrees)", initial_rotation);
        telemetry.addData("Movement (mm)", movement);
        telemetry.addData("Final Rotation (degrees)", final_rotation);
        telemetry.update();
        Thread.sleep(500);
    }

    public void move(float x, float y) {
        telemetry.addLine("Running function: move");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.update();
    }

    public void turn(float r) {
        telemetry.addLine("Running function: turn");
        telemetry.addData("r", r);
        telemetry.update();
    }

    public void move_and_turn(float x, float y, float r) {
        telemetry.addLine("Running function: move_and_turn");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("r", r);
        telemetry.update();
    }

    public void stop() {
        telemetry.addLine("Drivebase STOP");
        telemetry.update();
    }

    public void report_encoder_ticks() {}
}
