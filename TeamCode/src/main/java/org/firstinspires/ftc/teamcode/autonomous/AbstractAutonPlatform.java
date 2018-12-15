package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.common.StartLocation;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

/**
 * Created by efyang on 12/15/17.
 */

public abstract class AbstractAutonPlatform extends LinearOpMode {
    public abstract StartLocation getStartLocation();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        AutonMain runner = new AutonMain(hardwareMap, telemetry, getStartLocation());
        waitForStart();
        new Thread(() -> {
        System.out.println("opmode checker active");
            while (opModeIsActive()) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                }
            }
            Globals.OPMODE_ACTIVE = false;
        }).start();


        // initialize the more generic AutonMain container class
        // wait for the start button to be pressed.
        System.out.println("start mainbody");
        // run the stuff that we only want to run once
        runner.runOnce();

        // run stuff that we want to run repeatedly
        while (opModeIsActive()) {
            if (!runner.mainLoop()) {
                break;
            }
        }

        // clean up
        System.out.println("Finish");
        runner.finish();
        stop();
    }
}
