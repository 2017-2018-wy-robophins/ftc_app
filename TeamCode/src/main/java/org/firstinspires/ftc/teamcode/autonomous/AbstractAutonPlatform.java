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

    //A class that works to generalize between opmodes in code. Sets navigationalState.startLocation to the respective location, denoted
    //BLUE_LEFT, BLUE_RIGHT, RED_LEFT, or RED_RIGHT
    public abstract StartLocation getStartLocation();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.addLine("Initializing...");
        telemetry.update();
        AutonMain runner = new AutonMain(hardwareMap, telemetry, getStartLocation());
        waitForStart();
        Globals.OPMODE_ACTIVE.set(true);
        Thread opmodeCheckerThread = new Thread(() -> {
            System.out.println("opmode checker active");
            while (opModeIsActive()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    telemetry.addLine("Checker thread stopped");
                    telemetry.update();
                    break;
                }
            }
            Globals.OPMODE_ACTIVE.set(false);
        });
        opmodeCheckerThread.start();


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
        // INCREDIBLY IMPORTANT
        opmodeCheckerThread.interrupt();

        // clean up
        System.out.println("Finish");
        runner.finish();
        stop();
    }
}
