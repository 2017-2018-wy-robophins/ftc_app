package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class ClaimCommand extends Command {
    private int SERVO_TIME = 500;
    private int ROTATE_ENCODER_TICKS = 0;
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException{
        mainRobot.grabber.rotateTicks(ROTATE_ENCODER_TICKS, 0.1f);
        mainRobot.grabber.openContainer();
        Thread.sleep(SERVO_TIME);
        mainRobot.grabber.closeContainer();
        mainRobot.grabber.rotateTicks(-ROTATE_ENCODER_TICKS, 0.1f);
    }
}
