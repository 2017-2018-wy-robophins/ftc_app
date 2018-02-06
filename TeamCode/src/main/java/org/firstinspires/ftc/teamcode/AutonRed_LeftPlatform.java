package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonRed_LeftPlatform", group = "Sensor")
public class AutonRed_LeftPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.RED_LEFT;
    }
}

