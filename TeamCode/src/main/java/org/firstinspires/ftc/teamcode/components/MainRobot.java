package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.debug.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.drive_base.DriveBase;
import org.firstinspires.ftc.teamcode.components.drive_base.MecanumBase;


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

            // reset motor encoders
            NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // set them to brake when 0 power
            NW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            NE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // set motor directions
            NW.setDirection(DcMotor.Direction.FORWARD);
            SW.setDirection(DcMotor.Direction.FORWARD);
            NE.setDirection(DcMotor.Direction.REVERSE);
            SE.setDirection(DcMotor.Direction.REVERSE);

            // instantiate the drivebase with the given motors
            driveBase = new MecanumBase(NW, NE, SW, SE, telemetry);
        }
    }
}