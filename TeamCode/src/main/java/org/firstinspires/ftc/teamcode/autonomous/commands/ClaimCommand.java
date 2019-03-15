package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class ClaimCommand extends Command {
    private int SERVO_TIME = 500;
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException{
        // mainRobot.grabber.openContainer();
        Thread.sleep(SERVO_TIME);
        //mainRobot.grabber.intakeMotor.setPower(1);
        Thread.sleep(500);
        // mainRobot.grabber.intakeMotor.setPower(0);
        // mainRobot.grabber.closeContainer();
    }
}
