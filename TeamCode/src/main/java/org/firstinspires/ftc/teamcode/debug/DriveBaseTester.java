package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.text.BreakIterator;

@TeleOp(name = "Drive Base Tester", group = "Debug")
public class DriveBaseTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.05;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        float MAX_SPEED = 0.9f;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        boolean rb_pressed = false;
        ControlMode mode = ControlMode.Joy;
        while (opModeIsActive()) {
            double leftx = gamepad1.left_stick_x;
            double lefty = -gamepad1.left_stick_y;
            double rightx = gamepad1.right_stick_x;
            double righty = -gamepad1.right_stick_y;
            boolean right_bumper = gamepad1.right_bumper;
            telemetry.addData("Left x", leftx);
            telemetry.addData("Left y", lefty);
            telemetry.addData("Right x", rightx);
            telemetry.addData("Right y", righty);
            telemetry.addData("Right bumper", right_bumper);

            if (!rb_pressed && right_bumper) {
                mode = mode.toggle();
            }
            rb_pressed = right_bumper;
            telemetry.addData("Mode", mode);

            switch (mode) {
                case Tank:
                    left.setPower(MAX_SPEED * lefty);
                    right.setPower(MAX_SPEED * righty);
                case Joy:
                    double turn_contrib = Math.abs(rightx);
                    double throttle_contrib = 1 - turn_contrib;
                    left.setPower(MAX_SPEED * (lefty * throttle_contrib + rightx));
                    right.setPower(MAX_SPEED * (lefty * throttle_contrib - rightx));
            }

            telemetry.addData("R speed", right.getPower());
            telemetry.addData("L speed", left.getPower());
            telemetry.addData("R count", right.getCurrentPosition());
            telemetry.addData("L count", left.getCurrentPosition());

            telemetry.addLine("Running");
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }

    enum ControlMode {
        Tank, Joy;

        private ControlMode next;
        static {
            Tank.next = Joy;
            Joy.next = Tank;
        }

        public ControlMode toggle() {
            return next;
        }
    }
}


