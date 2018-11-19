package org.firstinspires.ftc.teamcode.autonomous.commands;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class SleepCommand extends Command {
    int ms;
    public SleepCommand(int ms) {
        this.ms = ms;
    }

    @Override
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            telemetry.addLine("Sleep interrupted");
            telemetry.update();
        }
    }
}
