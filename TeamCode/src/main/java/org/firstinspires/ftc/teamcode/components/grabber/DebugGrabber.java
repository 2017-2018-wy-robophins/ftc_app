package org.firstinspires.ftc.teamcode.components.grabber;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.grabber.Grabber;

/**
 * Created by efyang on 2/13/18.
 */

public class DebugGrabber implements Grabber {
    Telemetry telemetry;

    public DebugGrabber(Telemetry telemetry) {
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

    public void top_grab() {
        telemetry.addLine("Close top grabber");
        telemetry.update();
    }

    public void bottom_grab() {
        telemetry.addLine("Close bottom grabber");
        telemetry.update();
    }
}
