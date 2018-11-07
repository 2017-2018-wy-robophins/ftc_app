package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {
    Telemetry telemetry;
    public Component (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public abstract void reportInfo();
}
