package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Arm Value Tester", group = "Debug")
public class ArmValueTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();

        DcMotorEx arm1 = (DcMotorEx)hardwareMap.dcMotor.get("arm1");
        DcMotorEx arm2 = (DcMotorEx)hardwareMap.dcMotor.get("arm2");
        DcMotorEx arm3 = (DcMotorEx)hardwareMap.dcMotor.get("arm3");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(DcMotorSimple.Direction.FORWARD);
        arm3.setDirection(DcMotorSimple.Direction.FORWARD);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        float speed = 0.3f;

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();


        while (opModeIsActive()) {
            float arm1diff = (gamepad1.left_bumper ? speed:0) + ((!gamepad1.left_bumper && gamepad1.right_bumper) ? -speed:0);
            float arm2diff = (gamepad1.a ? speed:0) + ((!gamepad1.a && gamepad1.b) ? -speed: 0);
            float arm3diff = (gamepad1.x ? speed:0) + ((!gamepad1.x && gamepad1.y) ? -speed: 0);

            arm1.setPower(arm1diff);
            arm2.setPower(arm2diff);
            arm3.setPower(arm3diff);

            telemetry.addData("L Bumper", gamepad1.left_bumper);
            telemetry.addData("R Bumper", gamepad1.right_bumper);
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);

            telemetry.addData("1 diff", arm1diff);
            telemetry.addData("2 diff", arm2diff);
            telemetry.addData("3 diff", arm3diff);

            telemetry.addData("M1", arm1.getCurrentPosition());
            telemetry.addData("M2", arm2.getCurrentPosition());
            telemetry.addData("M3", arm3.getCurrentPosition());

            telemetry.addLine("Running");
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
