package org.firstinspires.ftc.teamcode.debug;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;

@Disabled
@TeleOp(name = "Vision Sample Tester HW", group = "Debug")
public class VisionSampleTesterHW extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Init");
        telemetry.update();

        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted);
        VuforiaVisionProcessor vision = new VuforiaVisionProcessor(hardwareMap);
        vision.initTfod();

        waitForStart();
        Thread.sleep(2000);
        SamplingConfiguration samplingConfiguration = vision.getSamplingConfigurationPhoneRightOnlyGold();
        if (samplingConfiguration == null) {
            samplingConfiguration = SamplingConfiguration.CENTER;
        }
        telemetry.addData("Sampling configuration", samplingConfiguration);
        servoSample(samplingConfiguration, mainRobot);

        vision.stopTfod();
        stop();
    }

    private void servoSample(SamplingConfiguration samplingConfiguration, MainRobot mainRobot) throws InterruptedException {
        int time = 2000;
        switch (samplingConfiguration) {
            case LEFT:
                mainRobot.sampler.extendLeft();
                Thread.sleep(time);
                mainRobot.sampler.contractLeft();
                break;
            case RIGHT:
                mainRobot.sampler.extendRight();
                Thread.sleep(time);
                mainRobot.sampler.contractRight();
                break;
            case CENTER:
                mainRobot.sampler.extendCenter();
                Thread.sleep(time);
                mainRobot.sampler.contractCenter();
                break;
        }
    }
}
