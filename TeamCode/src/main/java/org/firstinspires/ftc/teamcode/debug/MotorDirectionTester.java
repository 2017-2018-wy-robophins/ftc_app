package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Direction Tester", group = "Robot")
class MotorDirectionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();

        DcMotor NE = hardwareMap.dcMotor.get("NE");
        DcMotor NW = hardwareMap.dcMotor.get("NW");
        DcMotor SE = hardwareMap.dcMotor.get("SE");
        DcMotor SW = hardwareMap.dcMotor.get("SW");

        NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        NW.setDirection(DcMotor.Direction.FORWARD);
        NE.setDirection(DcMotor.Direction.FORWARD);
        SW.setDirection(DcMotor.Direction.FORWARD);
        SE.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.addLine("All motors should be going forward");
            telemetry.update();
            NW.setPower(0.3);
            NE.setPower(0.3);
            SW.setPower(0.3);
            SE.setPower(0.3);
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
