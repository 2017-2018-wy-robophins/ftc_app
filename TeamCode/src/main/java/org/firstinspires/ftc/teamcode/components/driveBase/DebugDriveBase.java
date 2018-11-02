package org.firstinspires.ftc.teamcode.components.driveBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

/**
 * Created by efyang on 2/13/18.
 */

public class DebugDriveBase extends DriveBase {
    private final Telemetry telemetry;

    public DebugDriveBase(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void imu_forward_move(float x, InertialSensor imu) {
        telemetry.addLine("Running function: imu forward move");
        telemetry.addData("x", x);
        telemetry.update();
    }

    public void imu_turn(float r, InertialSensor imu) {
        telemetry.addLine("Running function: imu turn");
        telemetry.addData("r", r);
        telemetry.update();
    }

    public void direct_move_and_turn(float x, float r) {
        telemetry.addLine("Running function: direct_move_and_turn");
        telemetry.addData("x", x);
        telemetry.addData("r", r);
        telemetry.update();
    }

    public void stop() {
        telemetry.addLine("Drivebase STOP");
        telemetry.update();
    }

    public void report_encoder_ticks() {}
}
