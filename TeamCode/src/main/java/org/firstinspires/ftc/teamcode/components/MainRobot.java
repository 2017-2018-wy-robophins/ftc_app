package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.arm.Arm;
import org.firstinspires.ftc.teamcode.components.arm.ThreeDOFArm;
import org.firstinspires.ftc.teamcode.components.driveBase.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.MecanumBase;
import org.firstinspires.ftc.teamcode.components.hook.DebugHook;
import org.firstinspires.ftc.teamcode.components.hook.Hook;
import org.firstinspires.ftc.teamcode.components.hook.RackAndPinionHook;


public class MainRobot {
    public DriveBase driveBase;
    public Hook hook;
    public Arm arm;
    //define all variables used
    boolean DEBUG_CLASSES;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap HM, Telemetry telemetry, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
            driveBase = new DebugDriveBase(telemetry);
            hook = new DebugHook(telemetry);
        } else {
            // hook
            DcMotor hookMotor = HM.dcMotor.get("hookMotor");
            hook = new RackAndPinionHook(hookMotor, telemetry);

            // arm
            DcMotor arm1 = HM.dcMotor.get("arm1");
            DcMotor arm2 = HM.dcMotor.get("arm2");
            DcMotor arm3 = HM.dcMotor.get("arm3");
            arm = new ThreeDOFArm(arm1, arm2, arm3, (float)Math.toRadians(0), (float)Math.toRadians(150), (float)Math.toRadians(-180), telemetry);

            // the main drive base
            DcMotor NE = HM.dcMotor.get("NE");
            DcMotor NW = HM.dcMotor.get("NW");
            DcMotor SE = HM.dcMotor.get("SE");
            DcMotor SW = HM.dcMotor.get("SW");
            driveBase = new MecanumBase(NW, NE, SW, SE, telemetry);
        }
    }
}