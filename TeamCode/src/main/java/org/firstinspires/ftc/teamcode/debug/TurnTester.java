package org.firstinspires.ftc.teamcode.debug;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

@Config
@TeleOp(name = "IMU Motion Tester", group = "Debug")
public class TurnTester extends LinearOpMode {
    public static double TURN_AMOUNT = 90;
    public static double FORWARD_AMOUNT = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted);
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start");
        telemetry.update();

        while (true) {
            if (gamepad1.a) {
                mainRobot.driveBase.imu_turn((float)TURN_AMOUNT, mainRobot.imu);
            } else if (gamepad1.b) {
                mainRobot.driveBase.imu_forward_move((float)FORWARD_AMOUNT, mainRobot.imu);
            }
        }
    }
}
