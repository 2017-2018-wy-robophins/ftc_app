package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@TeleOp(name = "Claim Test", group = "Debug")
public class ClaimTest extends LinearOpMode {
    private int ROTATE_TIME = 200;
    private int SERVO_TIME = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, false);
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start");
        telemetry.update();

        mainRobot.grabber.rotate(-0.3f);
        Thread.sleep(ROTATE_TIME);
        mainRobot.grabber.rotate(0);
        mainRobot.grabber.openContainer();
        Thread.sleep(SERVO_TIME);
        mainRobot.grabber.closeContainer();
        mainRobot.grabber.rotate(0.3f);
        Thread.sleep(ROTATE_TIME);
        mainRobot.grabber.rotate(0);
    }
}
