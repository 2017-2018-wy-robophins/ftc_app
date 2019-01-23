package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Provides information on limit switch behavior.
public class DigitalLimitSwitch {
    DigitalChannel sensor;
    String name;

    public DigitalLimitSwitch(HardwareMap hardwareMap, String name) {
        this.name = name;
        sensor = hardwareMap.get(DigitalChannel.class, name);
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    //Returns true if the sensor is being triggered, false otherwise.
    public boolean isPressed() {
        return !sensor.getState();
    }
}
