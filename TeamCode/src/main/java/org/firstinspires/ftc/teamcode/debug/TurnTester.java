package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

@TeleOp(name = "IMU Turn Tester", group = "Debug")
public class TurnTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, false);
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start");
        telemetry.update();

        telemetry.addLine("90 turn");
        telemetry.update();
        mainRobot.driveBase.imu_turn(90, mainRobot.imu);
        Thread.sleep(3000);
        telemetry.addLine("180 turn");
        telemetry.update();
        mainRobot.driveBase.imu_turn(180, mainRobot.imu);
        Thread.sleep(3000);
        telemetry.addLine("-180 turn");
        telemetry.update();
        mainRobot.driveBase.imu_turn(-179, mainRobot.imu);
        Thread.sleep(3000);
        telemetry.addLine("-90 turn");
        telemetry.update();
        mainRobot.driveBase.imu_turn(-90, mainRobot.imu);
    }
}
