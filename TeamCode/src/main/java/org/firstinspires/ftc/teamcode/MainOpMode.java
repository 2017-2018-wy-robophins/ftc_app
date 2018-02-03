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
        robot.init(hardwareMap);
        //initiate hardware variables
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
            double righty = gamepad1.right_stick_y;
            double rightx = gamepad1.right_stick_x;
            double leftx = gamepad1.left_stick_x;
            double lefty = gamepad1.left_stick_y;
            telemetry.clear();
            colorSensorServo.setPosition(0);

            if (Math.abs(righty) > ARM_JOYSTICK_MOVEMENT_THRESHOLD) {
                arm.setPower(righty*.40);
            } else {
                arm.setPower(0);
            }

            double speed = Math.sqrt(Math.pow(leftx, 2) + Math.pow(lefty, 2)) / FieldConstants.rt2;
            double angle = Math.atan2(lefty, leftx);
            double[] multipliers = ExtendedMath.mechanum_multipliers(speed, angle, rightx);
            robot.NW.setPower(multipliers[0]);
            robot.NE.setPower(multipliers[1]);
            robot.SW.setPower(multipliers[2]);
            robot.SE.setPower(multipliers[3]);

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
            telemetry.addData("NW ticks", robot.NW.getCurrentPosition());
            telemetry.addData("NE ticks", robot.NE.getCurrentPosition());
            telemetry.addData("SW ticks", robot.SW.getCurrentPosition());
            telemetry.addData("SE ticks", robot.SE.getCurrentPosition());
            telemetry.addData("Servo position", grab1.getPosition());
            telemetry.addData("Arm power", arm.getPower());
            telemetry.addData("Arm position", arm.getCurrentPosition()); // NICO - use this info to determine what ARM_POSITION_THRESHOLD is
            telemetry.addData("Gamepad status",  GamepadUser.ONE == gamepad1.getUser());
            telemetry.update();
        }
    }
}