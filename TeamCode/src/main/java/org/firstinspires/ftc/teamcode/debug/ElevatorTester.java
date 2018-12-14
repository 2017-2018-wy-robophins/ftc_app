package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.DigitalLimitSwitch;

@TeleOp(name = "Elevator Tester", group = "Debug")
public class ElevatorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftGrabber = hardwareMap.dcMotor.get("leftElevator");
        DcMotor rightGrabber = hardwareMap.dcMotor.get("rightElevator");

        leftGrabber.setDirection(DcMotorSimple.Direction.REVERSE);
        rightGrabber.setDirection(DcMotorSimple.Direction.FORWARD);

        leftGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightGrabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGrabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightGrabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DigitalLimitSwitch limitSwitch = new DigitalLimitSwitch(hardwareMap, "elevatorLimit");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                leftGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            float y = -gamepad1.right_stick_y;
            leftGrabber.setPower(y);
            rightGrabber.setPower(y);

            telemetry.addData("limit switch", limitSwitch.isPressed());
            telemetry.addData("left power", leftGrabber.getPower());
            telemetry.addData("right power", rightGrabber.getPower());
            telemetry.update();
        }
    }
}
