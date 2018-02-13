package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by efyang on 2/13/18.
 */

class DebugGrabber implements Grabber {
    Telemetry telemetry;

    DebugGrabber(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void open() {
        telemetry.addLine("Open grabber");
        telemetry.update();
    }

    public void close() {
        telemetry.addLine("Close grabber");
        telemetry.update();
    }
}
