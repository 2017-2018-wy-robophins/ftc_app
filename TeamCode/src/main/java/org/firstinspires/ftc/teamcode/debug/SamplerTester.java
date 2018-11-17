package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Sampler;

@Disabled
@TeleOp(name = "Sampler Tester", group = "Debug")
public class SamplerTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo rightSampler = hardwareMap.servo.get("rightSampler");
        Servo leftSampler = hardwareMap.servo.get("leftSampler");

        Sampler sampler = new Sampler(rightSampler, leftSampler);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                sampler.extendRight();
            } else if (gamepad1.b) {
                sampler.contractRight();
            }

            if (gamepad1.x) {
                sampler.extendLeft();
            } else if (gamepad1.y) {
                sampler.contractLeft();
            }

            sampler.reportInfo(telemetry);
            telemetry.update();
        }
    }
}
