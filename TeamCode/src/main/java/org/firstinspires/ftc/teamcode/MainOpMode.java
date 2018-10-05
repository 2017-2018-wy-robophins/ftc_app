package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.components.MainRobot;

import java.util.ResourceBundle;

@TeleOp(name="Main Teleop", group="Robot")
public class MainOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException  {
        // setup constants
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);

        //initiate robot
        MainRobot robot = new MainRobot(hardwareMap, telemetry, false);

        DcMotor hookMotor = hardwareMap.dcMotor.get("hookMotor");
        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double JOYSTICK_TRANSLATION_MULTIPLIER = 0.7;
        boolean halfpower = false;

        telemetry.addData("say", "before opmode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double righty = gamepad1.right_stick_y;
            double rightx = gamepad1.right_stick_x;
            double leftx = gamepad1.left_stick_x * JOYSTICK_TRANSLATION_MULTIPLIER;
            double lefty = gamepad1.left_stick_y * JOYSTICK_TRANSLATION_MULTIPLIER;
            telemetry.clear();

            float motor_mult = halfpower ? 0.5f : 1;
            if (((Math.abs(leftx) + Math.abs(lefty))/2) >= (Math.abs(rightx) + Math.abs(righty))/2) {
                // no diagonals wanted by nico
                if (Math.abs(leftx) > Math.abs(lefty)) {
                    robot.driveBase.move_and_turn(0.8f * (float) leftx * motor_mult, 0, 0);
                } else {
                    robot.driveBase.move_and_turn(0, -(float) lefty * motor_mult, 0);
                }
            } else {
                if (Math.abs(rightx) > Math.abs(righty)) {
                    robot.driveBase.move_and_turn(0, 0, -(float) rightx * 0.8f * motor_mult);
                }
            }

            if (gamepad1.y) {
                halfpower = !halfpower;
            }
            if (gamepad1.right_bumper) {
                hookMotor.setPower(1);
            } else if (gamepad1.left_bumper) {
                hookMotor.setPower(-1);
            } else {
                hookMotor.setPower(0);
            }

            //update telemetry
            telemetry.addData("R vertical", righty);
            telemetry.addData("R horizontal", rightx);
            telemetry.addData("L vertical",lefty);
            telemetry.addData("L horizontal", leftx);
            robot.driveBase.report_encoder_ticks();
            telemetry.addData("Hook power", hookMotor.getPower());
            telemetry.addData("Hook position", hookMotor.getCurrentPosition());
            telemetry.addData("Gamepad 1 status",  GamepadUser.ONE == gamepad1.getUser());
            telemetry.addData("R Bumper", gamepad1.right_bumper);
            telemetry.addData("L Bumper", gamepad1.left_bumper);
            telemetry.addData("R Trigger", gamepad1.right_trigger);
            telemetry.addData("L Trigger", gamepad1.left_trigger);
            telemetry.addData("Half power", halfpower);
            telemetry.update();
        }
        telemetry.addLine("Finished");
        telemetry.update();
    }
}
