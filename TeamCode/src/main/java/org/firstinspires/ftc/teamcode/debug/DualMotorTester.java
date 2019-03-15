package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Dual Motor Tester", group = "Debug")
public class DualMotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor mot1 = hardwareMap.dcMotor.get("test1");
        DcMotor mot2 = hardwareMap.dcMotor.get("test2");

        mot1.setDirection(DcMotorSimple.Direction.FORWARD);
        mot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mot2.setDirection(DcMotorSimple.Direction.FORWARD);
        mot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            float y = -gamepad1.right_stick_y;
            mot1.setPower(y);
            mot2.setPower(y);

            telemetry.addData("1 power", mot1.getPower());
            telemetry.addData("1 position", mot1.getCurrentPosition());
            telemetry.addData("2 power", mot2.getPower());
            telemetry.addData("2 position", mot2.getCurrentPosition());
            telemetry.update();
        }
    }
}
