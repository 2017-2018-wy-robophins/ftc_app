package org.firstinspires.ftc.teamcode.autonomous.commands;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

import java.util.List;
import java.util.concurrent.CompletableFuture;

public class CommandTree {
    List<CommandTree> children;
    private Command value;

    public CommandTree() { this.value = null; }

    public CommandTree(Command value) {
        this.value = value;
    }

    public CompletableFuture<Void> toFuture(NavigationalState navigationalState, InertialSensor imu, VisionProcessor vision, MainRobot mainRobot, Telemetry telemetry) {
        if (this.value != null) {
            CompletableFuture<Void>[] childFutures = children.stream()
                    .map((c) -> c.toFuture(navigationalState, imu, vision, mainRobot, telemetry))
                    .toArray(CompletableFuture[]::new);
            return CompletableFuture.runAsync(
                    () -> {
                        try {
                            value.execute(navigationalState, imu, vision, mainRobot, telemetry);
                        } catch (InterruptedException e) {

                        }
                    })
                    .thenRunAsync(() -> CompletableFuture.allOf(childFutures));
        } else {
            return CompletableFuture.completedFuture(null);
        }
    }
}
