package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.driveBase.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.hook.DebugHook;
import org.firstinspires.ftc.teamcode.components.hook.Hook;
import org.firstinspires.ftc.teamcode.components.hook.RackAndPinionHook;


public class MainRobot {
    public DriveBase driveBase;
    public Hook hook;
    //define all variables used
    boolean DEBUG_CLASSES;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap HM, Telemetry telemetry, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
            driveBase = new DebugDriveBase(telemetry);
            hook = new DebugHook(telemetry);
        } else {

        }
    }
}