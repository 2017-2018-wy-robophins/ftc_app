package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class ArmDeployCommand extends Command {
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException{
        while (Globals.OPMODE_ACTIVE.get() && !mainRobot.armMaxDownLimit.isPressed()) {
            mainRobot.grabber.rotate(0.1f);
            Thread.sleep(10);
        }

        mainRobot.grabber.rotate(0);
    }
}
