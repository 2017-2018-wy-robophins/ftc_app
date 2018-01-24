package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.graphics.Path;
import android.os.SystemClock;
import android.util.Pair;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Locale;
import java.util.Vector;

/**
 * Created by nico on 11/14/17.
 */

class AutonMain {
    private Telemetry telemetry;
    private StartLocation startLocation;
    private MainRobot robot;
    // hardware components
    private DcMotor north;
    private DcMotor west;
    private DcMotor east;
    private DcMotor south;
    private DcMotor arm;
    private Servo colorSensorServo;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;
    private NavigationalState navinfo;
    private AutonInstructions instructions;
    private VuforiaPositionFinder vuforiaPositionFinder;
    private RelicRecoveryVuMark vumark;


    // COLOR SENSOR STUFF----
    // hsvValues is an array that will hold the hue, saturation, and value information.
    // values is a reference to the hsvValues array.
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.

    // initializer
    AutonMain(MainRobot robot, HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        this.robot = robot;
        robot.init(hardwareMap);
        //initiate hardware variables
        north = robot.north;
        west = robot.west;
        east = robot.east;
        south = robot.south;
        arm = robot.arm;
        colorSensorServo = robot.colorSensorServo;

        // never gets used???
        // cs = robot.colorSensor;

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");

        navinfo = new NavigationalState();
        telemetry.addData("Start Location", startLocation);
        instructions = new AutonInstructions(startLocation);
        telemetry.addLine("Initializing vuforia...");
        telemetry.update();
        vuforiaPositionFinder = new VuforiaPositionFinder(startLocation, hardwareMap);
        telemetry.addLine("Initialized vuforia.");
        telemetry.update();
        vumark = RelicRecoveryVuMark.CENTER;
        /*
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        */
    }
    private float armPower = 0.2f;
    private float motorPower = 0.5f;
    private float ENCODER_TICKS_TIMEOUT_THRESHOLD = 10;
    private int TIMEOUT_MILLIS = 500;
    // run this once
    void runOnce() throws InterruptedException {
        // stopTime = SystemClock.currentThreadTimeMillis() + 30000;
        robot.closeServo();
        colorSensorServo.setPosition(1);

        Thread.sleep(1000);
        // note that the color sensor is on the left side of the arm
        switch (startLocation) {
            case BLUE_LEFT:
            case BLUE_RIGHT:
                if (sensorColor.red() < sensorColor.blue()) {
                    // if left is blue
                    knock_left_jewel(colorSensorServo);
                } else {
                    // if left is red
                    knock_right_jewel(colorSensorServo);
                }
                break;
            case RED_LEFT:
            case RED_RIGHT:
                if (sensorColor.red() > sensorColor.blue()) {
                    // if left is red
                    knock_left_jewel(colorSensorServo);
                } else {
                    // if left is blue
                    knock_right_jewel(colorSensorServo);
                }
                break;
        }

        // move the arm up slightly so that it doesn't drag
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        moveArm(0.4, 800);
        // get off the blocks
        move(-0.8, 0, 1400);

        // get vuforia position
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = vuforiaPositionFinder.getCurrentPosition();
        int vuforia_try_count = 1;
        int vuforia_max_tries = 4;

        while ((vuforia_try_count <= vuforia_max_tries) && (position == null)) {
            telemetry.addLine("Did not get vuforia position on try: " + vuforia_try_count + ", trying again.");
            telemetry.update();
            move(0.6, 0, 500);
            position = vuforiaPositionFinder.getCurrentPosition();
            vuforia_try_count += 1;
        }

        if (position != null) {
            navinfo = new NavigationalState(position.first);
            vumark = position.second;
            telemetry.addLine("Found position with vuforia: " + navinfo);
            telemetry.addData("Target", vumark);
            telemetry.update();
        } else {
            // fallback
            // set to the assumed positions for each starting location
            Pair<InstructionType, Pair<VectorF, Float>> first_instruction = instructions.next_instruction();
            Pair<VectorF, Float> default_nav_state = first_instruction.second;
            navinfo.set_position(default_nav_state.first);
            navinfo.set_heading(default_nav_state.second);
            telemetry.addLine("Could not get vuforia positions, running fallback");
            telemetry.addData("Using default position", navinfo);
            telemetry.addData("Using default target", vumark);
            telemetry.update();
        }
        // remove the vuforia finder when we're done
        // vuforiaPositionFinder = null;
        telemetry.addLine("Ok at line 171");
        telemetry.update();
        while (instructions.has_instructions()) {
            Pair<InstructionType, Pair<VectorF, Float>> instruction = instructions.next_instruction();
            Pair<VectorF, Float> instructionValues = instruction.second;
            float mmPerInch        = 25.4f;
            float mmPerBlock = mmPerInch * 24;
            switch (instruction.first) {
                case Move:
                    telemetry.addLine("Ok so far");
                    telemetry.update();
                    telemetry.addLine(String.format("Moving to target position %s and heading %s",
                            instructionValues.first,
                            String.valueOf(instructionValues.second)));
                    telemetry.update();
                    move_to_position_with_heading(instructionValues.first, instructionValues.second, motorPower, TIMEOUT_MILLIS);
                    break;
                case ArmUp:
                    telemetry.addLine("Moving arm up");
                    telemetry.update();
                    move_arm_up(armPower);
                    break;
                case ArmDown:
                    telemetry.addLine("Moving arm down");
                    telemetry.update();
                    move_arm_down(armPower);
                    break;
                case DropBlock:
                    telemetry.addLine("Dropping block");
                    telemetry.update();
                    robot.openServo();
                    Thread.sleep(500);
                    break;
                case GrabBlock:
                    telemetry.addLine("Grabbing block");
                    telemetry.update();
                    robot.openServo();
                    Thread.sleep(500);
                    robot.closeServo();
                    break;
                case MoveRelTarget:
                    // assume that robot is facing away from center, 1 block away

                    telemetry.addLine("Moving to relative target");
                    telemetry.update();
                    switch (vumark) {
                        case LEFT:
                            telemetry.addLine("Moving to LEFT");
                            telemetry.update();
                            move_by_vector_split(new VectorF(-0.25f * mmPerBlock,-0.5f * mmPerBlock), motorPower, TIMEOUT_MILLIS);
                            break;
                        case RIGHT:
                            telemetry.addLine("Moving to RIGHT");
                            telemetry.update();
                            move_by_vector_split(new VectorF(0.25f * mmPerBlock,-0.5f * mmPerBlock), motorPower, TIMEOUT_MILLIS);
                            break;
                        case UNKNOWN:
                        case CENTER:
                            telemetry.addLine("Moving to CENTER");
                            telemetry.update();
                            move_by_vector_split(new VectorF(0,-0.5f * mmPerBlock), motorPower, TIMEOUT_MILLIS);
                            break;
                    }
                    break;
                case BashBlock:
                    telemetry.addLine("Bashing block in");
                    telemetry.update();
                    move_by_vector_split(new VectorF(0, -0.5f * mmPerBlock), 0.5f, TIMEOUT_MILLIS);
                    move_by_vector_split(new VectorF(0, 0.2f * mmPerBlock), 0.5f, TIMEOUT_MILLIS);
                    move_by_vector_split(new VectorF(0, -0.5f * mmPerBlock), 0.5f, TIMEOUT_MILLIS);
                    break;
            }
        }

        telemetry.addLine("Finished");
        telemetry.update();
        stop();

        /*
        // TODO: put block in - theoretically
        switch (startLocation) {
            case RED_LEFT:
            case BLUE_LEFT:
                // move the arm up slightly so that it doesn't drag
                moveArm(armPower, 800);
                Thread.sleep(500);
                // get off platform, move to intersection of center line and first dotted line in diagram
                move(0, -.8, 850);
                turn(1,50);
                // move forward a tiny bit to leave space for arm
                move(0, 0.5, 400);
                 // "throw" block by bringing arm over
                moveArm(armPower, 1700);
                // * after the arm has reached the point where gravity helps - don't throw it
                moveArm(armPowerWithGravity, 500);
                Thread.sleep(1500);
                // drop glyph
                // move towards cryptobox (but backwards facing)
                move(0, -.5, 900);
                Thread.sleep(100);
                robot.openServo();
                Thread.sleep(1000);
                // get arm back down

                // make sure that glyph is off the robot by driving forward
                move(0, .6, 700);
                // ram it in the cryptobox
                move (0, -.4, 3000);

                moveArm(-.25, 500);
                break;
            case RED_RIGHT:
            case BLUE_RIGHT:
                // move the arm up slightly so that it doesn't drag
                moveArm(armPower, 800);
                Thread.sleep(500);
                // blue2 same as red2 (symmetrical)
                // get off platform, move to intersection of center line and first dotted line in diagram
                move(0, -.8, 850);
                turn(-1, 470);
                // "throw" block by bringing arm over
                move(0, -.5, 500);
                moveArm(armPower, 1900);
                // * after the arm has reached the point where gravity helps - don't throw it
                moveArm(armPowerWithGravity, 500);
                Thread.sleep(1500);
                // drop glyph
                // move towards cryptobox (but backwards facing)
                move(0, -.5, 900);
                Thread.sleep(100);
                robot.openServo();
                Thread.sleep(1000);
                // get arm back down

                // make sure that glyph is off the robot by driving forward
                move(0, .6, 700);
                // ram it in the cryptobox
                move (0, -.4, 3000);
                // move slightly forward so that we don't touch the glyph anymore
                move(0, 0.6, 100);

                moveArm(-.25, 500);
                break;
        }
        */
    }

    void mainLoop() throws InterruptedException {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double
        /*
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
        */
    }


    void finish() throws InterruptedException {
        /*
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
        */
    }

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6170-encodejavars-and-autonomous
    private void move_to_position_with_heading(VectorF pos, float heading, float power, int timeout) throws InterruptedException {
        move_to_position_split(pos, power, timeout);
        Thread.sleep(500);
        turn_to_heading(heading, power, timeout);
        Thread.sleep(500);
    }

    // TODO: set these
    private final float TICKS_PER_MM = 1.4f;
    private final float TICKS_PER_DEGREE = 4.5f;
    // TODO: make version without both vectors at the same time
    // target in mm

    private void move_by_vector(VectorF movement, float power, int timeout) throws InterruptedException {
        north.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        south.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        east.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        west.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        VectorF robotTickVector = movement.multiplied(TICKS_PER_MM);
        int x_vector = (int)robotTickVector.get(0);
        int y_vector = (int)robotTickVector.get(1);

        north.setTargetPosition(north.getCurrentPosition() + x_vector);
        south.setTargetPosition(south.getCurrentPosition() - x_vector);
        west.setTargetPosition(west.getCurrentPosition() - y_vector);
        east.setTargetPosition(east.getCurrentPosition() + y_vector);

        north.setPower(power);
        south.setPower(power);
        west.setPower(power);
        east.setPower(power);

        int last_north = north.getCurrentPosition();
        int last_south = south.getCurrentPosition();
        int last_west = west.getCurrentPosition();
        int last_east = east.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (north.isBusy() || south.isBusy() || east.isBusy() || west.isBusy()) {
            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(north.getCurrentPosition() - last_north) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(south.getCurrentPosition() - last_south) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(west.getCurrentPosition() - last_west) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(east.getCurrentPosition() - last_east) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(String.format("Move by vector %s interrupted: timed out.", movement));
                    telemetry.update();
                    break;
                }
                last_north = north.getCurrentPosition();
                last_south = south.getCurrentPosition();
                last_west = west.getCurrentPosition();
                last_east = east.getCurrentPosition();
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(10);
        }

        north.setPower(0);
        south.setPower(0);
        east.setPower(0);
        west.setPower(0);
    }

    private void move_to_position(VectorF target, float power, int timeout) throws InterruptedException {
        VectorF robotVector = navinfo.get_robot_movement_vector(target);
        telemetry.addLine(String.format("Moving %s to target position %s", robotVector, target));
        telemetry.update();
        move_by_vector(robotVector, power, timeout);
        navinfo.set_position(target);
    }

    private void move_to_position_split(VectorF target, float power, int timeout) throws InterruptedException {
        VectorF robotVector = navinfo.get_robot_movement_vector(target);
        telemetry.addLine(String.format("Moving %s to target position %s", robotVector, target));
        telemetry.update();
        move_by_vector_split(robotVector, power, timeout);
        navinfo.set_position(target);
    }

    private void move_by_vector_split(VectorF movement, float power, int timeout) throws InterruptedException {
        VectorF[] components = ExtendedMath.vector_components(movement);
        move_by_vector(components[0], power, timeout);
        move_by_vector(components[1], power, timeout);
    }


    private void turn_to_heading(float target, float power, int timeout) throws InterruptedException {
        north.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        south.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        east.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        west.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        float robotVector = navinfo.get_robot_rotation(target);
        float robotTickVector = robotVector * TICKS_PER_DEGREE;
        int rotation_vector = (int)robotTickVector;

        north.setTargetPosition(north.getCurrentPosition() - rotation_vector);
        south.setTargetPosition(south.getCurrentPosition() - rotation_vector);
        west.setTargetPosition(west.getCurrentPosition() + rotation_vector);
        east.setTargetPosition(east.getCurrentPosition() + rotation_vector);

        north.setPower(power);
        south.setPower(power);
        east.setPower(power);
        west.setPower(power);

        telemetry.addLine(String.format("Turning %s to target heading %s", String.valueOf(robotVector), String.valueOf(target)));
        telemetry.update();

        int last_north = north.getCurrentPosition();
        int last_south = south.getCurrentPosition();
        int last_west = west.getCurrentPosition();
        int last_east = east.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (north.isBusy() || south.isBusy() || east.isBusy() || west.isBusy()) {
            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(north.getCurrentPosition() - last_north) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(south.getCurrentPosition() - last_south) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(west.getCurrentPosition() - last_west) < ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(east.getCurrentPosition() - last_east) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(String.format("Turn to target heading %s interrupted: timed out.", String.valueOf(target)));
                    telemetry.update();
                    break;
                }
                last_north = north.getCurrentPosition();
                last_south = south.getCurrentPosition();
                last_west = west.getCurrentPosition();
                last_east = east.getCurrentPosition();
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }
            Thread.sleep(10);
        }

        north.setPower(0);
        south.setPower(0);
        east.setPower(0);
        west.setPower(0);

        navinfo.set_heading(target);
    }

    public void move(double xvector, double yvector, int ms) throws InterruptedException{
        north.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        south.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        east.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        west.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


    //up is positive
    private void moveArm(double v, int ms) throws InterruptedException {
        arm.setPower(v);
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

    private void stop() {
        west.setPower(0);
        east.setPower(0);
        north.setPower(0);
        south.setPower(0);
    }

    private int ARM_MOVEMENT_TICKS = -771;
    private void move_arm_ticks(int ticks, float power, int timeout) throws InterruptedException {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(arm.getCurrentPosition() + ticks);
        arm.setPower(power);
        int last_arm = arm.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;


        while (arm.isBusy()) {
            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(arm.getCurrentPosition() - last_arm) < ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(String.format("Move arm to target %s interrupted: timed out.", String.valueOf(ticks)));
                    telemetry.update();
                    break;
                }
            }

            Thread.sleep(10);
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void move_arm_up(float power) throws InterruptedException {
        move_arm_ticks(ARM_MOVEMENT_TICKS, power, TIMEOUT_MILLIS);
    }

    private void move_arm_down(float power) throws InterruptedException {
        move_arm_ticks(-ARM_MOVEMENT_TICKS, power, TIMEOUT_MILLIS);
    }
}
