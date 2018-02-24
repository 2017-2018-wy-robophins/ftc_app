package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonPlatform;
import org.firstinspires.ftc.teamcode.common.StartLocation;

// note: the registered opmodes can't be public, because the ftc opmode registrar will read them twice.
// Need to be package private to avoid already registered error.
@Autonomous(name = "AutonBlue_LeftPlatform", group = "Sensor")
public class AutonBlue_LeftPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.BLUE_LEFT;
    }
}

