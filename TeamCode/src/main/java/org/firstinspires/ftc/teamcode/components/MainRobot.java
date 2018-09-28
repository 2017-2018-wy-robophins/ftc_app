package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.driveBase.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.MecanumBase;


public class MainRobot {
    public DriveBase driveBase;
    //define all variables used
    boolean DEBUG_CLASSES;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap HM, Telemetry telemetry, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
            driveBase = new DebugDriveBase(telemetry);
        } else {
            // the main drive base
            DcMotor NE = HM.dcMotor.get("NE");
            DcMotor NW = HM.dcMotor.get("NW");
            DcMotor SE = HM.dcMotor.get("SE");
            DcMotor SW = HM.dcMotor.get("SW");

            // instantiate the drivebase with the given motors
            driveBase = new MecanumBase(NW, NE, SW, SE, telemetry);
        }
    }
}