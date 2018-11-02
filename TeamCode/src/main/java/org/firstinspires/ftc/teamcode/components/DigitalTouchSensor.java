package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DigitalTouchSensor {
    DigitalChannel sensor;
    String name;

    public DigitalTouchSensor(HardwareMap hardwareMap, String name) {
        this.name = name;
        sensor = hardwareMap.get(DigitalChannel.class, name);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        return !sensor.getState();
    }
}
