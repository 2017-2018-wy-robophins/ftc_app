package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class SampleCommand extends Command {
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) throws InterruptedException {
        Thread.sleep(1000);
        SamplingConfiguration samplingConfiguration = visionProcessor.getSamplingConfiguration();
        visionProcessor.stopTfod();
        switch (navigationalState.startLocation) {
            case BLUE_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case BLUE_RIGHT:
                switch (samplingConfiguration) {
                    case LEFT:
                        (new MovementCommand(-609.6f, -1219.2f, -90)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                    case RIGHT:
                        (new MovementCommand(-1219.2f, -609.6f, 180)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                }
                (new MovementCommand(-1219.2f, -1219.2f, -135)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                break;
            case RED_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case RED_RIGHT:
                switch (samplingConfiguration) {
                    case LEFT:
                        (new MovementCommand(1219.2f, 609.6f, 0)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                    case RIGHT:
                        (new MovementCommand(609.6f, 1219.2f, 90)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                        break;
                }
                (new MovementCommand(1219.2f, 1219.2f, 45)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                break;
        }
    }

    private void servoSample(SamplingConfiguration samplingConfiguration, MainRobot mainRobot) throws InterruptedException {
        switch (samplingConfiguration) {
            case LEFT:
                mainRobot.sampler.extendLeft();
                Thread.sleep(200);
                mainRobot.sampler.contractLeft();
                break;
            case RIGHT:
                mainRobot.sampler.extendRight();
                Thread.sleep(200);
                mainRobot.sampler.contractRight();
                break;
            case CENTER:
                mainRobot.sampler.extendCenter();
                Thread.sleep(200);
                mainRobot.sampler.contractCenter();
                break;
        }
    }
}
