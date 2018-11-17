package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@Disabled
@TeleOp(name = "Motor Direction Tester", group = "Debug")
public class MotorDirectionTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted,false);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.addLine("All motors should be going forward");
            mainRobot.driveBase.direct_move_and_turn(0.3f, 0);
            mainRobot.driveBase.report_encoder_ticks();
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
