package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class SampleCommand extends Command {
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        SamplingConfiguration samplingConfiguration = visionProcessor.getSamplingConfiguration();
        switch (navigationalState.startLocation) {
            case BLUE_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case BLUE_RIGHT:
                switch (samplingConfiguration) {
                    case LEFT:
                        break;
                    case RIGHT:
                        break;
                }
                (new MovementCommand(-1219.2f, -1219.2f, 90)).executeCommand(navigationalState, imu, visionProcessor, mainRobot, telemetry);
                break;
            case RED_LEFT:
                servoSample(samplingConfiguration, mainRobot);
                break;
            case RED_RIGHT:
                switch (samplingConfiguration) {

                }
                break;
        }
    }

    private void servoSample(SamplingConfiguration samplingConfiguration, MainRobot mainRobot) {

    }
}
