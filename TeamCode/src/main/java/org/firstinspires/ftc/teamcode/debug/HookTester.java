package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.components.DigitalTouchSensor;
import org.firstinspires.ftc.teamcode.components.hook.LimitedRackAndPinionHook;

@TeleOp(name = "Hook Tester", group = "Debug")
public class HookTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);
        DcMotor hookMotor = hardwareMap.dcMotor.get("hookMotor");
        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DigitalTouchSensor touchSensor = new DigitalTouchSensor(hardwareMap, "touchLimit");

        LimitedRackAndPinionHook hook = new LimitedRackAndPinionHook(hookMotor, touchSensor, telemetry);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        while (opModeIsActive()) {
            boolean pressed = touchSensor.isPressed();

            if (gamepad1.left_bumper) {
                hook.latch();
            }
            if (gamepad1.right_bumper) {
                hook.delatch();
            }

            hook.update();
            telemetry.addLine("Running");
            telemetry.addData("Speed", hookMotor.getPower());
            telemetry.addData("Position", hookMotor.getCurrentPosition());
            telemetry.addData("Pressed", pressed);
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
