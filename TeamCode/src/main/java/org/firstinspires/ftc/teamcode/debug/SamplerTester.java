package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Sampler;

@TeleOp(name = "Sampler Tester", group = "Debug")
public class SamplerTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo rightSampler = hardwareMap.servo.get("rightSampler");
        Servo leftSampler = hardwareMap.servo.get("leftSampler");
        Servo centerSampler = hardwareMap.servo.get("centerSampler");

        Sampler sampler = new Sampler(rightSampler, leftSampler, centerSampler);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                sampler.extendRight();
            } else if (gamepad1.b) {
                sampler.extendLeft();
            } else if (gamepad1.right_bumper) {
                sampler.extendCenter();
            }

            if (gamepad1.x) {
                sampler.contractRight();
            } else if (gamepad1.y) {
                sampler.contractLeft();
            } else if (gamepad1.left_bumper) {
                sampler.contractCenter();
            }

            sampler.reportInfo(telemetry);
            telemetry.update();
        }
    }
}
