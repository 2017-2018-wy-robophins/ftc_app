package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonPlatform;
import org.firstinspires.ftc.teamcode.common.StartLocation;

///Sets initial navigationalState information for the Red Alliance's left-most hanging location.
@Autonomous(name = "AutonRed_LeftPlatform", group = "Sensor")
public class AutonRed_LeftPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.RED_LEFT;
    }
}

