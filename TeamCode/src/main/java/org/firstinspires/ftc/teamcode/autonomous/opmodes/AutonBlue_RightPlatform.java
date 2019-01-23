package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonPlatform;
import org.firstinspires.ftc.teamcode.common.StartLocation;

///Sets initial navigationalState information for the Blue Alliance's right-most hanging location.
@Autonomous(name = "AutonBlue_RightPlatform", group = "Sensor")
public class AutonBlue_RightPlatform extends AbstractAutonPlatform {
    public StartLocation getStartLocation() {
        return StartLocation.BLUE_RIGHT;
    }
}

