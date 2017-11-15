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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by nico on 11/14/17.
 */

public class AutonMain {
    public static void move(double xvector, double yvector,int ms, MainRobot r) throws InterruptedException{
        r.west.setPower(yvector);
        r.east.setPower(-yvector);
        r.north.setPower(xvector);
        r.south.setPower(-xvector);
        Thread.sleep(ms);
        stop(r);
    }
    public static void turn(double v, int ms, MainRobot r) throws InterruptedException {
        r.west.setPower(v);
        r.east.setPower(v);
        r.north.setPower(v);
        r.south.setPower(v);
        Thread.sleep(ms);
        stop(r);
    }

    public static void knock_jewel_left(MainRobot robot, Servo colorSensorServo) throws InterruptedException {
        turn(1, 150, robot); // turn right
        Thread.sleep(500);
        colorSensorServo.setPosition(0);
        turn(-1, 150, robot); // turn left
    }

    public static void knock_jewel_right(MainRobot robot, Servo colorSensorServo) throws InterruptedException {
        turn(-1, 150, robot); // turn left
        Thread.sleep(500);
        colorSensorServo.setPosition(0);
        turn(1, 150, robot); // turn right
    }

    public static void stop(MainRobot r) {
        r.west.setPower(0);
        r.east.setPower(0);
        r.north.setPower(0);
        r.south.setPower(0);
    }

    MainRobot robot;
    HardwareMap hardwareMap;
    DcMotor north;
    DcMotor west;
    DcMotor east;
    DcMotor south;
    DcMotor arm;
    Servo grab1;
    Servo colorSensorServo;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    public static final double servoClosed = 1.0;
    public static final double servoOpen = 0.1;


    public AutonMain(MainRobot robot, HardwareMap hardwareMap, teleme) throws InterruptedException {
        this.robot = robot;
        this.hardwareMap = hardwareMap;

        robot.init(hardwareMap);
        //initiate hardware variables
        north = robot.north;
        west = robot.west;
        east = robot.east;
        south =robot.south;
        arm = robot.arm;
        grab1 = robot.grab1;
        colorSensorServo = robot.colorSensorServo;
        // never gets used???
        // cs = robot.colorSensor;
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


    }

    public void first

    public void mainLoop() throws InterruptedException {

    }

    public void runOpMode() throws InterruptedException {


        long stopTime = SystemClock.currentThreadTimeMillis() + 30000;
        grab1.setPosition(servoClosed);
        colorSensorServo.setPosition(1);

        Thread.sleep(1000);
        // note that the color sensor is on the left side of the arm
        if (sensorColor.red() < sensorColor.blue()) {
            // if left is blue
            AutonMain.knock_jewel_left(robot, colorSensorServo);
        } else {
            // if left is red
            AutonMain.knock_jewel_right(robot, colorSensorServo);
        }

        // TODO: put block in - theoretically
        /*
        move(0, 1, 400, robot);
        turn(1, 350, robot);
        move(0, -1, 400, robot);
        arm.setPower(-.45);
        Thread.sleep(500);
        arm.setPower(0);
        grab1.setPosition(servoOpen);
        move(0, 1, 500, robot);
        move(0, -1, 500, robot);
        stop();
        */

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptable method.
        while (opModeIsActive()) {
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

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });


    }

}
