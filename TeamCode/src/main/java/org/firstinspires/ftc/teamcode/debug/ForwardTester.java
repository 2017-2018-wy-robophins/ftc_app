package org.firstinspires.ftc.teamcode.debug;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@TeleOp(name = "IMU Forward Tester", group = "Debug")
public class ForwardTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, false);
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start");
        telemetry.update();

        telemetry.addLine("forward 4 feet");
        telemetry.update();

        //mainRobot.driveBase.imu_forward_move(6000f, mainRobot.imu);
        mainRobot.driveBase.imu_forward_move(1219.2f, mainRobot.imu);
    }
}
