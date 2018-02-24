package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.MainRobot;

@TeleOp(name = "Motor Encoder Tester", group = "Robot")
public class MotorEncoderTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        // MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, false);

        DcMotor NW = hardwareMap.dcMotor.get("NW");
        DcMotor SW = hardwareMap.dcMotor.get("SW");
        DcMotor NE = hardwareMap.dcMotor.get("NE");
        DcMotor SE = hardwareMap.dcMotor.get("SE");

        NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        NW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        NE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SW.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        NW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        NW.setDirection(DcMotor.Direction.FORWARD);
        SW.setDirection(DcMotor.Direction.FORWARD);
/*
        NW.setDirection(DcMotor.Direction.REVERSE);
        SW.setDirection(DcMotor.Direction.REVERSE);
        */
        NE.setDirection(DcMotor.Direction.REVERSE);
        SE.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        NW.setTargetPosition(171);
        NE.setTargetPosition(171);
        SW.setTargetPosition(171);
        SE.setTargetPosition(171);

        NW.setPower(0.3);
        NE.setPower(0.3);
        SW.setPower(0.3);
        SE.setPower(0.3);
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.addLine("All motors should be going forward");
            telemetry.addData("NW", NW.getCurrentPosition());
            telemetry.addData("NE", NE.getCurrentPosition());
            telemetry.addData("SW", SW.getCurrentPosition());
            telemetry.addData("SE", SE.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
