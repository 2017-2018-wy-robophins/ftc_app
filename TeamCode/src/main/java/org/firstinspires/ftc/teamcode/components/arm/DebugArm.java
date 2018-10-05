package org.firstinspires.ftc.teamcode.components.arm;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class DebugArm implements Arm {
    Telemetry telemetry;
    public DebugArm(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean goToTarget(VectorF target) {
        return false;
    }

    public VectorF getOrientation() {
        return null;
    }
}
