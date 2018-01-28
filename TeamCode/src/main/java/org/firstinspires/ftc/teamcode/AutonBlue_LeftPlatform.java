package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutonBlue_LeftPlatform", group = "Sensor")
public class AutonBlue_LeftPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.BLUE_LEFT;
    }
}

