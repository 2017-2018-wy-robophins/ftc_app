package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonPlatform;
import org.firstinspires.ftc.teamcode.common.StartLocation;

@Autonomous(name = "AutonRed_LeftPlatform", group = "Sensor")
class AutonRed_LeftPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.RED_LEFT;
    }
}

