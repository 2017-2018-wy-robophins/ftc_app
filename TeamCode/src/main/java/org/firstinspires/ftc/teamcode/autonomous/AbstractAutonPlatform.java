package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        AutonMain runner = new AutonMain(hardwareMap, telemetry, getStartLocation());
        waitForStart();
        CompletableFuture<Void> opModeActiveChecker = CompletableFuture.runAsync(() -> {
            System.out.println("opmode checker active");
            while (opModeIsActive()) {
                idle();
            }
            Globals.OPMODE_ACTIVE = false;
            // Thread.currentThread().interrupt();
        });

        CompletableFuture<Void> mainBody = CompletableFuture.runAsync(() -> {
            try {
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
            } catch (InterruptedException e) {
                System.out.println("Interrupted");
            }
        });

        try {
            CompletableFuture.allOf(opModeActiveChecker, mainBody).get();
        } catch (ExecutionException e) {
            System.out.println("Error on execution");
        }
    }
}
