package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;


@TeleOp(name="Main Teleop", group="Robot")
public class MainOpMode extends LinearOpMode {

    //creates an instance variable fo the robot
    private MainRobot robot = new MainRobot();

    private static double enginePower = 1.0;
    private static double turnCoefficient = .4;
    private static double leftx = 0.0;
    private static double righty = 0.0;
    private static double rightx = 0.0;
    private static double lefty = 0.0;

    private static double move_coeff = 0.7;
    @Override
    public void runOpMode() throws InterruptedException  {
        // setup constants
        // TODO: use telemetry data to actually set this correctly
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        //initiate robot
        robot.init(hardwareMap);
        //initiate hardware variables
        DcMotor north = robot.north;
        DcMotor west = robot.west;
        DcMotor east = robot.east;
        DcMotor south = robot.south;
        DcMotor arm = robot.arm;
        Servo grab1 = robot.grab1;
        Servo grab2 = robot.grab2;
        robot.openServo();
        Servo colorSensorServo = robot.colorSensorServo;

        telemetry.addData("say", "before opmode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (GamepadUser.ONE != gamepad1.getUser()) {
                stop();
            }
            righty = gamepad1.right_stick_y;
            rightx = gamepad1.right_stick_x;
            leftx = gamepad1.left_stick_x;
            lefty = gamepad1.left_stick_y;
            telemetry.clear();
            colorSensorServo.setPosition(0);

            if (Math.abs(righty) > ARM_JOYSTICK_MOVEMENT_THRESHOLD) {
                arm.setPower(righty*.40);
            } else {
                arm.setPower(0);
            }
            //use only protocol for the currently used joystick
            if (((Math.abs(leftx) + Math.abs(lefty))/2) >= (Math.abs(rightx) + Math.abs(righty))/2) {
                //set motor powers for transposing
                west.setPower(lefty * move_coeff); // in commit - fix reversal
                east.setPower(-lefty * move_coeff);
                north.setPower(leftx * move_coeff);
                south.setPower(-leftx * move_coeff);

            } else {
                //set motor powers for turning
                west.setPower(rightx * turnCoefficient);
                east.setPower(rightx * turnCoefficient);
                north.setPower(rightx * turnCoefficient);
                south.setPower(rightx * turnCoefficient);
            }


            if (gamepad1.right_bumper) {
                robot.closeServo();
            }
            if (gamepad1.left_bumper) {
                robot.openServo();
            }

            //update telemetry
            telemetry.addData("R vertical", righty);
            telemetry.addData("L vertical",lefty);
            telemetry.addData("R horizontal", rightx);
            telemetry.addData("L horizontal", leftx);
            telemetry.addData("North ticks", north.getCurrentPosition());
            telemetry.addData("East ticks", east.getCurrentPosition());
            telemetry.addData("West ticks", west.getCurrentPosition());
            telemetry.addData("South ticks", south.getCurrentPosition());
            telemetry.addData("Engine power", enginePower);
            telemetry.addData("Servo position", grab1.getPosition());
            telemetry.addData("Arm power", arm.getPower());
            telemetry.addData("Arm position", arm.getCurrentPosition()); // NICO - use this info to determine what ARM_POSITION_THRESHOLD is
            telemetry.addData("Gamepad status",  GamepadUser.ONE == gamepad1.getUser());
            telemetry.update();
        }
    }
}