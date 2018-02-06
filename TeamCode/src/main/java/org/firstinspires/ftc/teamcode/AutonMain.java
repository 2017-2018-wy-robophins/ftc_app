package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
    private DistanceSensor sensorDistance;
    private NavigationalState navinfo;
    private AutonInstructions instructions;
    private VuforiaPositionFinder vuforiaPositionFinder;
    private RelicRecoveryVuMark vumark;

    AutonMain(MainRobot robot, HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        this.robot = robot;
        robot.init(hardwareMap);
        //initiate hardware variables
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

    // run this once
    void runOnce() throws InterruptedException {
        // stopTime = SystemClock.currentThreadTimeMillis() + 30000;
        robot.grabber.close();

        colorSensorServo.setPosition(1);

        Thread.sleep(250);
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

        // get off the blocks
        switch (startLocation) {
            case BLUE_LEFT:
            case BLUE_RIGHT:
                move(-0.8, 0, 1000);
                move(0.8, 0, 400);
                break;
            case RED_LEFT:
                move(0.8, 0, 1000);
                move(-0.8, 0, 400);
            case RED_RIGHT:
                break;
        }

        // get vuforia position
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = vuforiaPositionFinder.getCurrentPosition();
        int vuforia_try_count = 1;
        int vuforia_max_tries = 4;

        while ((vuforia_try_count <= vuforia_max_tries) && (position == null)) {
            telemetry.addLine("Did not get vuforia position on try: " + vuforia_try_count + ", trying again.");
            telemetry.update();
            move(0.8, 0, 400);
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
                    move_to_position_with_heading(instructionValues.first, instructionValues.second, RobotConstants.AUTON_MOTOR_SPEED, RobotConstants.MOTOR_TIMEOUT_MILLIS);
                    break;
                case ArmUp:
                    telemetry.addLine("Moving arm up");
                    telemetry.update();
                    robot.move_arm_up(RobotConstants.AUTON_ARM_SPEED);
                    break;
                case ArmDown:
                    telemetry.addLine("Moving arm down");
                    telemetry.update();
                    robot.move_arm_down(RobotConstants.AUTON_ARM_SPEED);
                    break;
                case DropBlock:
                    telemetry.addLine("Dropping block");
                    telemetry.update();
                    robot.grabber.open();
                    break;
                case GrabBlock:
                    telemetry.addLine("Grabbing block");
                    telemetry.update();
                    robot.grabber.open();
                    robot.grabber.close();
                    break;
                case MoveRelTarget:
                    // assume that robot is facing away from center, 1 block away

                    telemetry.addLine("Moving to relative target");
                    telemetry.update();
                    switch (vumark) {
                        case LEFT:
                            telemetry.addLine("Moving to LEFT");
                            telemetry.update();
                            move_by_vector_and_rotation(new VectorF(-0.25f * mmPerBlock, -0.2f * mmPerBlock), 0, RobotConstants.AUTON_MOTOR_SPEED, RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                        case RIGHT:
                            telemetry.addLine("Moving to RIGHT");
                            telemetry.update();
                            move_by_vector_and_rotation(new VectorF(0.25f * mmPerBlock, -0.2f * mmPerBlock), 0, RobotConstants.AUTON_MOTOR_SPEED, RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                        case UNKNOWN:
                        case CENTER:
                            telemetry.addLine("Moving to CENTER");
                            telemetry.update();
                            move_by_vector_and_rotation(new VectorF(0, -0.2f * mmPerBlock), 0, RobotConstants.AUTON_MOTOR_SPEED, RobotConstants.MOTOR_TIMEOUT_MILLIS);
                            break;
                    }
                    break;
                case BashBlock:
                    telemetry.addLine("Bashing block in");
                    telemetry.update();
                    move(0, 0.8, 500);
                    move(0, -0.8, 500);
                    break;
            }
        }
        telemetry.addLine("Bashing block in");
        telemetry.update();
        move(0, 0.8, 500);
        move(0, -0.8, 500);

        telemetry.addLine("Finished");
        telemetry.update();
        robot.stop();
    }

    void mainLoop() throws InterruptedException {}

    void finish() throws InterruptedException {}

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6170-encodejavars-and-autonomous
    private void move_to_position_with_heading(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        Thread.sleep(1000);
        move_by_vector_and_rotation(
                navinfo.get_robot_movement_vector(target_pos),
                navinfo.get_robot_rotation(target_heading),
                power,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_by_vector_and_rotation(VectorF movement, float rotation, float power, int timeout) throws InterruptedException {
        robot.NW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.NE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.SW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.SE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        VectorF movementTickVector = movement.multiplied(RobotConstants.TICKS_PER_MM);
        float rotationTickVector = rotation * RobotConstants.TICKS_PER_DEGREE;

        double[] multipliers = ExtendedMath.mechanum_multipliers(
                movementTickVector.magnitude(),
                Math.atan2(movementTickVector.get(1), movementTickVector.get(0)),
                rotationTickVector);

        int NW_target = robot.NW.getCurrentPosition() + (int)multipliers[0];
        int NE_target = robot.NE.getCurrentPosition() + (int)multipliers[1];
        int SW_target = robot.SW.getCurrentPosition() + (int)multipliers[2];
        int SE_target = robot.SE.getCurrentPosition() + (int)multipliers[3];

        robot.NW.setTargetPosition(NW_target);
        robot.NE.setTargetPosition(NE_target);
        robot.SW.setTargetPosition(SW_target);
        robot.SE.setTargetPosition(SE_target);

        robot.NW.setPower(power);
        robot.NE.setPower(power);
        robot.SW.setPower(power);
        robot.SE.setPower(power);

        int last_NW = robot.NW.getCurrentPosition();
        int last_NE = robot.NE.getCurrentPosition();
        int last_SW = robot.SW.getCurrentPosition();
        int last_SE = robot.SE.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (robot.NW.isBusy() || robot.NE.isBusy() || robot.SW.isBusy() || robot.NW.isBusy()) {
            int NW_current = robot.NW.getCurrentPosition();
            int NE_current = robot.NE.getCurrentPosition();
            int SW_current = robot.SW.getCurrentPosition();
            int SE_current = robot.SE.getCurrentPosition();

            if ((Math.abs(NW_current - NW_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR) &&
                    (Math.abs(NE_current - NE_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR) &&
                    (Math.abs(SW_current - SW_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR) &&
                    (Math.abs(SE_current - SW_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR)) {
                telemetry.addLine(String.format("Move by vector %s and rotation %s interrupted: within absolute error.", movementTickVector, rotationTickVector));
                telemetry.update();
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(NW_current - last_NW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(NE_current - last_NE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SW_current - last_SW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SE_current - last_SE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(String.format("Move by vector %s and rotation %s interrupted: timed out.", movementTickVector, rotationTickVector));
                    telemetry.update();
                    break;
                }

                last_NW = NW_current;
                last_NE = NE_current;
                last_SW = SW_current;
                last_SE = SE_current;
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(5);
        }

        robot.stop();
    }

    private void move(double xvector, double yvector, int ms) throws InterruptedException{
        robot.NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed = Math.sqrt(Math.pow(xvector, 2) + Math.pow(yvector, 2)) / FieldConstants.rt2;
        double translation_angle = Math.atan2(yvector, xvector);
        double[] multipliers = ExtendedMath.mechanum_multipliers(speed, translation_angle, 0);
        robot.NW.setPower(multipliers[0]);
        robot.NE.setPower(multipliers[1]);
        robot.SW.setPower(multipliers[2]);
        robot.SE.setPower(multipliers[3]);
        Thread.sleep(ms);
        robot.stop();
    }

    private void turn(double v, int ms) throws InterruptedException {
        robot.NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double[] multipliers = ExtendedMath.mechanum_multipliers(0, 0, v);
        robot.NW.setPower(multipliers[0]);
        robot.NE.setPower(multipliers[1]);
        robot.SW.setPower(multipliers[2]);
        robot.SE.setPower(multipliers[3]);
        Thread.sleep(ms);
        robot.stop();
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


}
