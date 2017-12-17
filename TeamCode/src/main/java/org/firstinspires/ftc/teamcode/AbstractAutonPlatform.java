package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by efyang on 12/15/17.
 */

abstract class AbstractAutonPlatform extends LinearOpMode {
    private MainRobot robot = new MainRobot();
    final StartLocation startLocation;

    AbstractAutonPlatform(StartLocation startLocation) {
        this.startLocation = startLocation;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize the more generic AutonMain container class
        AutonMain runner = new AutonMain(robot, hardwareMap, telemetry, startLocation);
        // wait for the start button to be pressed.
        waitForStart();
        // run the stuff that we only want to run once
        runner.runOnce();

        // run stuff that we want to run repeatedly
        while (opModeIsActive()) {
            runner.mainLoop();
        }

        // clean up
        runner.finish();
    }

}
