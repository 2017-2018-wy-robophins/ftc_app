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

    @Override
    public void runOpMode() throws InterruptedException  {
        // setup constants
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        //initiate robot
        robot.init(hardwareMap, telemetry);
        //initiate hardware variables
        DcMotor arm = robot.arm;
        robot.grabber.open();
        Servo colorSensorServo = robot.colorSensorServo;
        double JOYSTICK_TRANSLATION_MULTIPLIER = 0.7;

        telemetry.addData("say", "before opmode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (GamepadUser.ONE != gamepad1.getUser()) {
                stop();
            }
            double righty = gamepad1.right_stick_y;
            double rightx = gamepad1.right_stick_x;
            double leftx = gamepad1.left_stick_x * JOYSTICK_TRANSLATION_MULTIPLIER;
            double lefty = gamepad1.left_stick_y * JOYSTICK_TRANSLATION_MULTIPLIER;
            telemetry.clear();
            colorSensorServo.setPosition(0);

            if (Math.abs(rightx) < Math.abs(righty)) {
                if (Math.abs(righty) > ARM_JOYSTICK_MOVEMENT_THRESHOLD) {
                    arm.setPower(righty*.25);
                } else {
                    arm.setPower(0);
                }
            } else {
                arm.setPower(0);
            }
            // robot.driveBase.move_and_turn((float) leftx, -(float) lefty, -(float) rightx);
            // reverted to previous conditional
            if (((Math.abs(leftx) + Math.abs(lefty))/2) >= (Math.abs(rightx) + Math.abs(righty))/2) {
                // no diagonals wanted by nico
                if (Math.abs(leftx) > Math.abs(lefty)) {
                    robot.driveBase.move_and_turn((float) leftx, 0, 0);
                } else {
                    robot.driveBase.move_and_turn(0, -(float) lefty, 0);
                }
            } else {
                if (Math.abs(rightx) > Math.abs(righty)) {
                    robot.driveBase.move_and_turn(0, 0, -(float) rightx);
                }
            }

            if (gamepad1.right_bumper) {
                robot.grabber.close();
            }
            if (gamepad1.left_bumper) {
                robot.grabber.open();
            }

            //update telemetry
            telemetry.addData("R vertical", righty);
            telemetry.addData("R horizontal", rightx);
            telemetry.addData("L vertical",lefty);
            telemetry.addData("L horizontal", leftx);
            robot.driveBase.report_encoder_ticks();
            telemetry.addData("Arm power", arm.getPower());
            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("Gamepad status",  GamepadUser.ONE == gamepad1.getUser());
            telemetry.update();
        }
    }
}