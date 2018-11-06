package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Grabber Tester", group = "Debug")
public class GrabberTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftGrabber = hardwareMap.dcMotor.get("leftGrabber");
        DcMotor rightGrabber = hardwareMap.dcMotor.get("rightGrabber");

        leftGrabber.setDirection(DcMotorSimple.Direction.FORWARD);
        rightGrabber.setDirection(DcMotorSimple.Direction.REVERSE);

        leftGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ControlMode mode = ControlMode.Direct;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                mode = mode.toggle();
            }

            if (gamepad1.right_bumper) {
                leftGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            switch (mode) {
                case Direct:
                    float y = -gamepad1.right_stick_y;
                    leftGrabber.setPower(y);
                    rightGrabber.setPower(y);
                case Button:
                    if (gamepad1.a) {
                        leftGrabber.setPower(0.5);
                        rightGrabber.setPower(0.5);
                    }

                    if (gamepad1.b) {
                        leftGrabber.setPower(-0.5);
                        rightGrabber.setPower(-0.5);
                    }

                    if (gamepad1.x) {
                        leftGrabber.setPower(1);
                        rightGrabber.setPower(1);
                    }

                    if (gamepad1.y) {
                        leftGrabber.setPower(-1);
                        rightGrabber.setPower(-1);
                    }
            }

            telemetry.addData("Mode", mode);
            telemetry.addData("left power", leftGrabber.getPower());
            telemetry.addData("right power", rightGrabber.getPower());
            telemetry.update();
        }
    }

    enum ControlMode {
        Direct, Button;

        private ControlMode next;
        static {
            Direct.next = Button;
            Button.next = Direct;
        }

        public ControlMode toggle() {
            return next;
        }
    }
}
