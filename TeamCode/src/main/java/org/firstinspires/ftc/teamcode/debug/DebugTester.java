package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.StartLocation;
import org.firstinspires.ftc.teamcode.components.positionFinder.VuforiaPositionFinder;

@TeleOp(name = "Debug+Vuforia Tester", group = "Robot")
public class DebugTester extends LinearOpMode {
    private StartLocation startLocation = StartLocation.BLUE_LEFT;
    private NavigationalState navinfo;
    private VuforiaPositionFinder vuforiaPositionFinder;
    private RelicRecoveryVuMark vumark;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        navinfo = new NavigationalState();
        telemetry.addData("Start Location", startLocation);
        telemetry.addLine("Initializing vuforia...");
        telemetry.update();
        vuforiaPositionFinder = new VuforiaPositionFinder(startLocation, hardwareMap);
        telemetry.addLine("Initialized vuforia.");
        telemetry.update();
        vumark = RelicRecoveryVuMark.CENTER;
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = vuforiaPositionFinder.getCurrentPosition();


        while (opModeIsActive()) {
            telemetry.addLine("Running");
            position = vuforiaPositionFinder.getCurrentPosition();
            if (position != null) {
                navinfo = new NavigationalState(position.first);
                vumark = position.second;
                telemetry.addLine("Found position with vuforia: " + navinfo);
                telemetry.addData("Target", vumark);
                telemetry.update();
            }
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
