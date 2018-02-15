package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Vector;

/**
 * Created by nico on 11/14/17.
 */

class AutonMain {
    private Telemetry telemetry;
    private StartLocation startLocation;
    private MainRobot robot;
    // hardware components
    private DcMotor arm;
    private Servo colorSensorServo;
    private ColorSensor sensorColor;
    private NavigationalState navinfo;
    private AutonInstructions instructions;
    private PositionFinder positionFinder;
    private RelicRecoveryVuMark vumark;
    private final boolean DEBUG = true;
    private final boolean DEBUG_CLASSES = true;

    AutonMain(MainRobot robot, HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        this.robot = robot;
        robot.init(hardwareMap, telemetry, DEBUG_CLASSES);
        //initiate hardware variables
        arm = robot.arm;
        colorSensorServo = robot.colorSensorServo;

        // never gets used???
        // cs = robot.colorSensor;

        // get a reference to the color sensor.
        if (!DEBUG_CLASSES) {
            sensorColor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        }
        // get a reference to the distance sensor that shares the same name.

        navinfo = new NavigationalState();
        telemetry.addData("Start Location", startLocation);
        instructions = new AutonInstructions(startLocation);
        telemetry.addLine("Initializing vuforia...");
        telemetry.update();
        if (!DEBUG_CLASSES) {
            positionFinder = new VuforiaPositionFinder(startLocation, hardwareMap);
        } else {
            positionFinder = new DebugPositionFinder();
        }
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

    // run this once
    void runOnce() throws InterruptedException {
        // stopTime = SystemClock.currentThreadTimeMillis() + 30000;
        robot.grabber.close();
        servoSet(1);

        // note that the color sensor is on the left side of the arm

        if (!DEBUG_CLASSES) {
            telemetry.addData("red", sensorColor.red());
            telemetry.addData("blue", sensorColor.blue());
            telemetry.update();
            Thread.sleep(500);
            switch (startLocation) {
                case BLUE_LEFT:
                case BLUE_RIGHT:
                    if (sensorColor.blue() < sensorColor.red()) {
                        // if left is blue
                        knock_left_jewel();
                    } else {
                        // if left is red
                        knock_right_jewel();
                    }
                    break;
                case RED_LEFT:
                case RED_RIGHT:
                    if (sensorColor.blue() > sensorColor.red()) {
                        // if left is red
                        knock_left_jewel();
                    } else {
                        // if left is blue
                        knock_right_jewel();
                    }
                    break;
            }
        } else {
            telemetry.addLine("Knocked jewel");
            telemetry.update();
            Thread.sleep(500);
        }

        if (DEBUG) {
            telemetry.addLine("Getting off block");
            telemetry.update();
            Thread.sleep(500);
        }

        float start_move_x = 0;
        float start_move_y = 0;
        // get off the blocks
        switch (startLocation) {
            case BLUE_LEFT:
            case BLUE_RIGHT:
                start_move_x = 1f;
                start_move_y = 0;
                break;
            case RED_LEFT:
                start_move_x = 0;
                start_move_y = -1f;
                break;
            case RED_RIGHT:
                start_move_x = 1f;
                start_move_y = 0;
                break;
        }
        robot.driveBase.move_ms(start_move_x, start_move_y, 1500);
        robot.driveBase.move_ms(-start_move_x, -start_move_y, 400);

        /* hard bash
        switch (startLocation) {
            case BLUE_LEFT:
                robot.driveBase.move_ms(0, 1, 2000);
                robot.driveBase.move_ms(-1, 0, 500);
                break;
            case BLUE_RIGHT:
                robot.driveBase.move_ms(0, 1, 1500);
                robot.driveBase.move_ms(-1, 0, 500);
                break;
            case RED_LEFT:
                robot.driveBase.move_ms(0, -1, 2000);
                robot.driveBase.move_ms(1, 0, 500);
                break;
            case RED_RIGHT:
                robot.driveBase.move_ms(0, -1, 1500);
                robot.driveBase.move_ms(1, 0, 500);
                break;
        }
        */

        // get position
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = positionFinder.getCurrentPosition();
        int vuforia_try_count = 1;
        int vuforia_max_tries = 4;

        while ((vuforia_try_count <= vuforia_max_tries) && (position == null)) {
            telemetry.addLine("Did not get vuforia position on try: " + vuforia_try_count + ", trying again.");
            telemetry.update();
            robot.driveBase.move_ms(-start_move_x, -start_move_y, 400);
            position = positionFinder.getCurrentPosition();
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

        while (instructions.has_instructions()) {
            if (DEBUG) {
                telemetry.addData("Navigational Info: ", navinfo);
                telemetry.update();
                Thread.sleep(500);
            }

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
                    Thread.sleep(500);
                    move_to_position_with_heading_no_diag_split_rotation(instructionValues.first, instructionValues.second, RobotConstants.AUTON_MOTOR_SPEED, RobotConstants.MOTOR_TIMEOUT_MILLIS);
                    break;
                case ArmUp:
                    telemetry.addLine("Moving arm up");
                    telemetry.update();
                    Thread.sleep(500);
                    robot.move_arm_up(RobotConstants.AUTON_ARM_SPEED);
                    break;
                case ArmDown:
                    telemetry.addLine("Moving arm down");
                    telemetry.update();
                    Thread.sleep(500);
                    robot.move_arm_down(RobotConstants.AUTON_ARM_SPEED);
                    break;
                case DropBlock:
                    telemetry.addLine("Dropping block");
                    telemetry.update();
                    Thread.sleep(500);
                    robot.grabber.open();
                    break;
                case GrabBlock:
                    telemetry.addLine("Grabbing block");
                    telemetry.update();
                    Thread.sleep(500);
                    robot.grabber.open();
                    robot.grabber.close();
                    break;
                case MoveRelTarget:
                    // assume that robot is facing away from center, 1 block away

                    telemetry.addLine("Moving to relative target");
                    telemetry.update();
                    Thread.sleep(500);
                    switch (vumark) {
                        case LEFT:
                            telemetry.addLine("Moving to LEFT");
                            telemetry.update();
                            Thread.sleep(500);
                            robot.driveBase.move_by_vector_and_rotation(
                                    new VectorF(-0.25f * mmPerBlock, -0.2f * mmPerBlock),
                                    0,
                                    RobotConstants.AUTON_MOTOR_SPEED,
                                    RobotConstants.TICK_ALLOWED_ABS_ERROR,
                                    RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                        case RIGHT:
                            telemetry.addLine("Moving to RIGHT");
                            telemetry.update();
                            Thread.sleep(500);
                            robot.driveBase.move_by_vector_and_rotation(
                                    new VectorF(0.25f * mmPerBlock, -0.2f * mmPerBlock),
                                    0,
                                    RobotConstants.AUTON_MOTOR_SPEED,
                                    RobotConstants.TICK_ALLOWED_ABS_ERROR,
                                    RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                        case UNKNOWN:
                        case CENTER:
                            telemetry.addLine("Moving to CENTER");
                            telemetry.update();
                            Thread.sleep(500);
                            robot.driveBase.move_by_vector_and_rotation(new VectorF(0, -0.2f * mmPerBlock),
                                    0,
                                    RobotConstants.AUTON_MOTOR_SPEED,
                                    RobotConstants.TICK_ALLOWED_ABS_ERROR,
                                    RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                    }
                    break;
                case BashBlock:
                    telemetry.addLine("Bashing block in");
                    telemetry.update();
                    Thread.sleep(500);
                    robot.driveBase.move_ms(0, 0.8f, 500);
                    robot.driveBase.move_ms(0, -0.8f, 500);
                    break;
            }
        }

        telemetry.addLine("Finished");
        telemetry.update();
        robot.driveBase.stop();
    }

    // return whether we want to continue or not
    boolean mainLoop() throws InterruptedException {
        return false;
    }

    void finish() throws InterruptedException {}

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6170-encodejavars-and-autonomous
    private void move_to_position_with_heading(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        robot.driveBase.move_by_vector_and_rotation(
                navinfo.get_robot_movement_vector(target_pos),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_split_rotation(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        robot.driveBase.move_by_vector_and_rotation(
                navinfo.get_robot_movement_vector(target_pos),
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                new VectorF(0, 0),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_no_diag(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        VectorF movement_vector = navinfo.get_robot_movement_vector(target_pos);
        VectorF[] movement_component_vectors = ExtendedMath.vector_components(movement_vector);
        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[0],
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[1],
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_no_diag_split_rotation(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        VectorF movement_vector = navinfo.get_robot_movement_vector(target_pos);
        VectorF[] movement_component_vectors = ExtendedMath.vector_components(movement_vector);
        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[0],
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[1],
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                new VectorF(0, 0),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_only_vdrive(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        // get the angle we need to rotate to first to move correctly
        VectorF movement_vector = navinfo.get_robot_movement_vector(target_pos);
        float movement_angle = (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));

        float initial_rotation = navinfo.get_robot_rotation(movement_angle);
        navinfo.set_heading(movement_angle);
        float final_rotation = navinfo.get_robot_rotation(target_heading);
        float movement_amount = movement_vector.magnitude();
        robot.driveBase.rotate_and_move_only_vertical_drive(initial_rotation, movement_amount, final_rotation, power, RobotConstants.TICK_ALLOWED_ABS_ERROR, timeout);

        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    //up is positive
    private void moveArm(double v, int ms) throws InterruptedException {
        arm.setPower(v);
        Thread.sleep(ms);
        arm.setPower(0);
    }

    private void knock_left_jewel() throws InterruptedException {
        robot.driveBase.turn_ms(1, 120); // turn left
        servoSet(0);
        robot.driveBase.turn_ms(-1, 120); // turn right
    }

    private void knock_right_jewel() throws InterruptedException {
        robot.driveBase.turn_ms(-1, 120); // turn right
        servoSet(0);
        robot.driveBase.turn_ms(1, 120); // turn left
    }

    private void servoSet(double position) throws InterruptedException {
        if (DEBUG_CLASSES) {
            telemetry.addLine("Servo Down");
            telemetry.update();
        } else {
            colorSensorServo.setPosition(position);
        }
        Thread.sleep(500);
    }
}
