package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;


@TeleOp(name="Main Teleop", group="Robot")
public class MainOpMode extends LinearOpMode {

    //creates an instance variable fo the robot
    MainRobot robot = new MainRobot();


    private static double enginePower = 1.0;
    private static double turnCoefficient = .4;
    private static double leftx = 0.0;
    private static double righty = 0.0;
    private static double rightx = 0.0;
    private static double lefty = 0.0;

    private static double precision = 1.0;

    private static double servoClosed = 1.0;
    private static double servoOpen = 0;
    @Override
    public void runOpMode() throws InterruptedException  {
        // setup constants
        // TODO: use telemetry data to actually set this correctly
        final int ARM_POSITION_THRESHOLD = 30;
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        //initiate robot
        robot.init(hardwareMap);
        //initiate hardware variables
        DcMotor north = robot.north;
        DcMotor west = robot.west;
        DcMotor east = robot.east;//gey u r an be a gey boiii
        DcMotor south = robot.south;
        DcMotor arm = robot.arm;
        Servo grab1 = robot.grab1;
        Servo grab2 = robot.grab2;
        Servo colorSensorServo = robot.colorSensorServo;
        boolean targetSet = false;
        int target = 0;


        telemetry.addData("say", "before opmode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (GamepadUser.ONE != gamepad1.getUser()) {
                stop();
            }
            righty = gamepad1.right_trigger * 2 - 1;
            // temporary fix for bad controller
            // remap from 0 - 1 to -1 to 1
            rightx = (gamepad1.left_trigger * 2 - 1);
            leftx = gamepad1.left_stick_x;
            lefty = gamepad1.left_stick_y;
            telemetry.clear();
            colorSensorServo.setPosition(0);
            //hold y for precision
            if (gamepad1.y) {
                precision = 0.5;
            } else {
                precision = 1.0;
            }
            if (righty > ARM_JOYSTICK_MOVEMENT_THRESHOLD) {
                arm.setPower(righty*.30);
                targetSet = false;
            } else if (righty < -ARM_JOYSTICK_MOVEMENT_THRESHOLD){
                // when the arm is getting moved "up"
                if (arm.getCurrentPosition() > ARM_POSITION_THRESHOLD) {
                    // once past the threshold, gravity is helping
                    arm.setPower(righty * 0.30);
                } else {
                    arm.setPower(righty * .6);
                }
                targetSet = false;
            } else if (targetSet) {
                arm.setTargetPosition(target);
            } else {
                arm.setPower(0);
                target = arm.getCurrentPosition();
                targetSet = true;
            }
            //use only protocol for the currently used joystick
            if (((Math.abs(leftx) + Math.abs(lefty))/2) >= (Math.abs(rightx) + Math.abs(righty))/2) {
                //set motor powers for transposing
                west.setPower(lefty*precision*.7); // in commit - fix reversal
                east.setPower(-lefty*precision*.7);
                north.setPower(leftx*precision*.7);
                south.setPower(-leftx*precision*.7);

            } else {
                //set motor powers for turning
                west.setPower(rightx*precision*turnCoefficient);
                east.setPower(rightx*precision*turnCoefficient);
                north.setPower(rightx*precision*turnCoefficient);
                south.setPower(rightx*precision*turnCoefficient);
            }


            if (gamepad1.right_bumper) {
                grab1.setPosition(servoClosed);
                Thread.sleep(200);
                grab2.setPosition(servoOpen);
            }
            if (gamepad1.left_bumper) {
                grab1.setPosition(servoOpen);
                Thread.sleep(200);
                grab2.setPosition(servoClosed);
            }


            //update telemetry
            telemetry.addData("R vertical", righty);
            telemetry.addData("L vertical",lefty);
            telemetry.addData("R horizontal", rightx);
            telemetry.addData("L horizontal", leftx);
            telemetry.addData("Engine power", enginePower);
            telemetry.addData("Servo position", grab1.getPosition());
            telemetry.addData("Arm power", arm.getPower());
            telemetry.addData("Arm position", arm.getCurrentPosition()); // NICO - use this info to determine what ARM_POSITION_THRESHOLD is
            telemetry.addData("Precision coefficient", precision);
            telemetry.addData("Gamepad status",  GamepadUser.ONE == gamepad1.getUser());
            telemetry.update();

        }
    }



    //a function that returns a modified value, checking if it falls within logical boundaries first
    public double incr(double value, double incr, String sign) {
        if (sign.equals("+")) {
            if (value + incr <= 1.0) {
                return value + incr;
            }
        } else if (sign.equals("-")) {
            if (value - incr >= 0.0) {
                return value - incr;
            }
        }
        return value;
    }

}