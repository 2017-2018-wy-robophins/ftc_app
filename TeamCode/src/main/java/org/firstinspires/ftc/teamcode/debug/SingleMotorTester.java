package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Single Motor Tester", group = "Debug")
public class SingleMotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor mot = hardwareMap.dcMotor.get("test");

        mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            float y = -gamepad1.right_stick_y;
            mot.setPower(y);

            telemetry.addData("power", mot.getPower());
            telemetry.addData("position", mot.getCurrentPosition());
            telemetry.update();
        }
    }
}
