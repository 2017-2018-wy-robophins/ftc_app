package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class HookControlCommand extends Command {
    //A command for movement of the elevator, the mechanism that allows the robot to perform a pull-up.
    private ElevatorHook.State targetState;
    public HookControlCommand(ElevatorHook.State targetState) {
        this.targetState = targetState;
    }

    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        mainRobot.hook.goToStateBlocking(targetState);
    }
}
