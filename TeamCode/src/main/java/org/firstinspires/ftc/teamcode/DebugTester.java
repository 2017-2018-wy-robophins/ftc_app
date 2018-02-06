package org.firstinspires.ftc.teamcode;

/**
 * Created by efyang on 1/25/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Debug Tester", group = "Robot")
public class DebugTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
