package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.components.grabber.HingedGrabber;
import org.firstinspires.ftc.teamcode.debug.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.grabber.DebugGrabber;
import org.firstinspires.ftc.teamcode.components.drive_base.DriveBase;
import org.firstinspires.ftc.teamcode.components.drive_base.MecanumBase;
import org.firstinspires.ftc.teamcode.components.grabber.Grabber;
import org.firstinspires.ftc.teamcode.components.grabber.QuadrupleGrabber;


public class MainRobot {
    public DriveBase driveBase;
    //define all variables used
    public DcMotor arm;
    public Servo colorSensorServo;
    public Grabber grabber;
    // DistanceSensor colorDistanceSensor;
    // ColorSensor colorSensor;
    boolean DEBUG_CLASSES;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap HM, Telemetry telemetry, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
            driveBase = new DebugDriveBase(telemetry);
            grabber = new DebugGrabber(telemetry);
        } else {
            // the main drive base
            DcMotor NE = HM.dcMotor.get("NE");
            DcMotor NW = HM.dcMotor.get("NW");
            DcMotor SE = HM.dcMotor.get("SE");
            DcMotor SW = HM.dcMotor.get("SW");

            NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            NW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            NE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            NW.setDirection(DcMotor.Direction.FORWARD);
            SW.setDirection(DcMotor.Direction.FORWARD);
/*
            NW.setDirection(DcMotor.Direction.REVERSE);
            SW.setDirection(DcMotor.Direction.REVERSE);
            */
            NE.setDirection(DcMotor.Direction.REVERSE);
            SE.setDirection(DcMotor.Direction.REVERSE);

            driveBase = new MecanumBase(NW, NE, SW, SE, telemetry);

            // set up the grabber
            Servo topLeft = HM.servo.get("topLeft");
            Servo topRight = HM.servo.get("topRight");
            Servo bottomLeft = HM.servo.get("bottomLeft");
            Servo bottomRight = HM.servo.get("bottomRight");
            /*
            Servo leftHinge = HM.servo.get("leftHinge");
            Servo rightHinge = HM.servo.get("rightHinge");
            */


            // set grabber servo directions
            topRight.setDirection(Servo.Direction.REVERSE);
            bottomRight.setDirection(Servo.Direction.REVERSE);
            /*
            leftHinge.setDirection(Servo.Direction.REVERSE);
            grabber = new HingedGrabber(topLeft, topRight, leftHinge, rightHinge, bottomLeft, bottomRight);
            */
            grabber = new QuadrupleGrabber(topLeft, topRight, bottomLeft, bottomRight);

            // set up the color sensor stuff
            colorSensorServo = HM.servo.get("colorSensorServo");
            // colorDistanceSensor = HM.get(DistanceSensor.class, "colorDistanceSensor");

            // set up the arm
            arm = HM.dcMotor.get("arm");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void move_arm_ticks(int ticks, float power, int timeout) throws InterruptedException {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int arm_target = arm.getCurrentPosition() + ticks;
        arm.setTargetPosition(arm_target);
        arm.setPower(power);
        int last_arm = arm.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;

        while (arm.isBusy()) {
            int arm_current = arm.getCurrentPosition();
            if (Math.abs(arm_current - arm_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR) {
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(arm_current - last_arm) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    break;
                }
                last_arm = arm.getCurrentPosition();
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(5);
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move_arm_up(float power) throws InterruptedException {
        if (!DEBUG_CLASSES) {
            move_arm_ticks(RobotConstants.ARM_MOVEMENT_TICKS, power, RobotConstants.MOTOR_TIMEOUT_MILLIS);
        }
    }

    public void move_arm_down(float power) throws InterruptedException {
        if (!DEBUG_CLASSES) {
            move_arm_ticks(-RobotConstants.ARM_MOVEMENT_TICKS, power, RobotConstants.MOTOR_TIMEOUT_MILLIS);
        }
    }
}