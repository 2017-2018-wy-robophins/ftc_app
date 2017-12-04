package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.os.SystemClock;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by nico on 11/14/17.
 */

class AutonMain {
    private Telemetry telemetry;
    private TeamColor teamColor;
    private StartLocation startLocation;
    // hardware components
    private DcMotor north;
    private DcMotor west;
    private DcMotor east;
    private DcMotor south;
    private DcMotor arm;
    private Servo grab1;
    private Servo grab2;
    private Servo colorSensorServo;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    private static final double servoClosed = 1.0;
    private static final double servoOpen = 0;

    // COLOR SENSOR STUFF----
    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    private final float values[] = hsvValues;
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private final double SCALE_FACTOR = 255;
    private final View relativeLayout;
    private long stopTime;

    // initializer
    AutonMain(MainRobot robot, HardwareMap hardwareMap, Telemetry telemetry, TeamColor teamColor, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.teamColor = teamColor;
        this.startLocation = startLocation;
        robot.init(hardwareMap);
        //initiate hardware variables
        north = robot.north;
        west = robot.west;
        east = robot.east;
        south = robot.south;
        arm = robot.arm;
        grab1 = robot.grab1;
        grab2 = robot.grab2;
        colorSensorServo = robot.colorSensorServo;

        // never gets used???
        // cs = robot.colorSensor;

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    }

    // run this once
    void runOnce() throws InterruptedException {
        stopTime = SystemClock.currentThreadTimeMillis() + 30000;
        grab2.setPosition(servoOpen);
        colorSensorServo.setPosition(1);

        Thread.sleep(1000);
        // note that the color sensor is on the left side of the arm
        switch (teamColor) {
            case BLUE:
                if (sensorColor.red() < sensorColor.blue()) {
                    // if left is blue
                    knock_left_jewel(colorSensorServo);
                } else {
                    // if left is red
                    knock_right_jewel(colorSensorServo);
                }
            case RED:
                if (sensorColor.red() > sensorColor.blue()) {
                    // if left is red
                    knock_left_jewel(colorSensorServo);
                } else {
                    // if left is blue
                    knock_right_jewel(colorSensorServo);
                }
        }

        // TODO: put block in - theoretically

        switch (startLocation) {
            case LEFT_PLATFORM:

            case RIGHT_PLATFORM:

        }

    }

    void mainLoop() throws InterruptedException {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Runtime ", stopTime - SystemClock.currentThreadTimeMillis());
        telemetry.addData("Hue", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        telemetry.update();
        if ((stopTime - SystemClock.currentThreadTimeMillis()) < 5000) {

        }
    }


    void finish() throws InterruptedException {
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }

    public void move(double xvector, double yvector, int ms) throws InterruptedException{
        west.setPower(-yvector);
        east.setPower(yvector);
        north.setPower(xvector);
        south.setPower(-xvector);
        Thread.sleep(ms);
        stop();
    }

    public void turn(double v, int ms) throws InterruptedException {
        west.setPower(v);
        east.setPower(v);
        north.setPower(v);
        south.setPower(v);
        Thread.sleep(ms);
        stop();
    }
    public void closeServo() {
        grab1.setPosition(servoClosed);
        //apparently grab2 is upside-down physically
        grab2.setPosition(servoOpen);
    }
    public void openServo() throws InterruptedException {
        grab1.setPosition(servoOpen);
        Thread.sleep(250);
        grab2.setPosition(servoClosed);
    }

    //up is positive
    public void moveArm(double v, int ms) throws InterruptedException {
        arm.setPower(-v);
        Thread.sleep(ms);
        arm.setPower(0);
    }

    private void knock_left_jewel(Servo colorSensorServo) throws InterruptedException {
        turn(1, 120); // turn left
        colorSensorServo.setPosition(0);
        Thread.sleep(500);
        turn(-1, 120); // turn right
    }

    private void knock_right_jewel(Servo colorSensorServo) throws InterruptedException {
        turn(-1, 120); // turn right
        colorSensorServo.setPosition(0);
        Thread.sleep(500);
        turn(1, 120); // turn left
    }

    public void stop() {
        west.setPower(0);
        east.setPower(0);
        north.setPower(0);
        south.setPower(0);
    }
}
