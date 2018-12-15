package org.firstinspires.ftc.teamcode.debug;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;

@Disabled
@TeleOp(name = "Vision Sample Tester", group = "Debug")
public class VisionSampleTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Init");
        telemetry.update();

        VuforiaVisionProcessor vision = new VuforiaVisionProcessor(hardwareMap);
        vision.initTfod();
        CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        SamplingConfiguration samplingConfiguration = SamplingConfiguration.CENTER;
        while (opModeIsActive()) {

            SamplingConfiguration tempSamplingConfiguration = vision.getSamplingConfigurationPhoneRightOnlyGold();
            if (tempSamplingConfiguration != null) {
                telemetry.addData("temp config", tempSamplingConfiguration);
                samplingConfiguration = tempSamplingConfiguration;
                telemetry.addData("Sampling Configuration", samplingConfiguration);
                telemetry.update();
            }
        }
        telemetry.addLine("finished");
        telemetry.update();
        vision.stopTfod();
        stop();
    }
}