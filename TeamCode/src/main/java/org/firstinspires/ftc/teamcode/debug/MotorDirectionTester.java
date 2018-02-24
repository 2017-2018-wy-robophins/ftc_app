package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.MainRobot;

@TeleOp(name = "Motor Direction Tester", group = "Robot")
public class MotorDirectionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, false);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.addLine("All motors should be going forward");
            mainRobot.driveBase.move(0, 0.3f);
            mainRobot.driveBase.report_encoder_ticks();
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
