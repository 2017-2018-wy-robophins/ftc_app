package org.firstinspires.ftc.teamcode.components.hook;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DebugHook implements Hook {
    Telemetry telemetry;

    public DebugHook(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void latch() {

    }

    public void delatch() {

    }
}
