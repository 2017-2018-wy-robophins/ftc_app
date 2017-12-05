package org.firstinspires.ftc.teamcode;

/**
 * Created by nico on 11/4/17.
 */

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.app.Activity;
import android.graphics.Color;
import android.os.SystemClock;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Time;
import java.util.Locale;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.util.Locale;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "AutonBlue2", group = "Sensor")
public class AutonBlue2 extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    private MainRobot robot = new MainRobot();
    private double armPower = 0.47;
    private double armPowerWithGravity = 0.23;
    private boolean DEV_ARM_ADJUST = false;
    private Servo grab2 = robot.grab2;
    double servoClosed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize the more generic AutonMain container class
        AutonMain runner = new AutonMain(robot, hardwareMap, telemetry, TeamColor.BLUE, StartLocation.RIGHT_PLATFORM);
        // wait for the start button to be pressed.
        if (DEV_ARM_ADJUST) {
            waitForStart_tuneArmPowerWithGamepad(telemetry);
        } else {
            waitForStart();
        }
        // run the stuff that we only want to run once
        runner.runOnce();



        // move the arm up slightly so that it doesn't drag
        runner.moveArm(armPower, 800);
        Thread.sleep(500);
        // blue2 same as red2 (symmetrical)
        // get off platform, move to intersection of center line and first dotted line in diagram
        // runner.turn(1, 50);
        runner.move(0, -.8, 850);
        runner.turn(-1, 430);
        // "throw" block by bringing arm over
        runner.move(0, -.5, 500);
        runner.moveArm(armPower, 1700);
        // * after the arm has reached the point where gravity helps - don't throw it
        runner.moveArm(armPowerWithGravity, 500);
        Thread.sleep(1500);
        // drop glyph
        // move towards cryptobox (but backwards facing)
        runner.move(0, -.5, 500);
        Thread.sleep(100);
        robot.openServo();
        Thread.sleep(1000);
        // get arm back down
        runner.moveArm(-.25, 500);
        // make sure that glyph is off the robot by driving forward
        runner.move(0, .6, 700);
        // ram it in the cryptobox
        runner.move (0, -.4, 3000);



        runner.stop();

        // run stuff that we want to run repeatedly
        while (opModeIsActive()) {
            runner.mainLoop();
        }

        // clean up
        runner.finish();
    }

    private void waitForStart_tuneArmPowerWithGamepad(Telemetry telemetry) throws InterruptedException {
        double righty, lefty;
        final double joystick_threshold = 0.5;
        final double increase_value = 0.01;

        while (!isStarted()) {

            righty = gamepad1.right_trigger * 2 - 1;
            lefty = gamepad1.left_stick_y;
            telemetry.clear();
            telemetry.addData("R vertical", righty);
            telemetry.addData("L vertical", lefty);
            telemetry.addData("Arm power", robot.arm.getPower());
            telemetry.addData("armPower", armPower);
            telemetry.addData("armPowerWithGravity", armPowerWithGravity);
            telemetry.update();
            // increase if above appropriate thresholds
            // righty increases and decreases armPower, while lefty increases and decreases armPowerWithGravity
            armPower += ((Math.abs(righty) > joystick_threshold) ? 1:0) * Math.signum(righty) * increase_value;
            armPowerWithGravity += ((Math.abs(lefty) > joystick_threshold) ? 1:0) * Math.signum(lefty) * increase_value;
            Thread.sleep(100);
        }
    }

    // acceleration in power per 100ms chunk -- change power every 100 ms
    private void acceleratingMoveArm(AutonMain runner, double power, double acceleration, int ms) throws InterruptedException {
        for (int i = 0; i < ms/100; ms++) {
            runner.moveArm(power + (double)i * acceleration, 100);
        }
    }
}

