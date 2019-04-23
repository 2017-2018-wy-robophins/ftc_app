package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@Config
@TeleOp(name = "Dual Driver Main Op Mode", group = "main")
public class DualDriverMainOpMode extends LinearOpMode {
    // need double to be tuned with ftc dashboard
    public static double DRIVE_FORWARD_SCALE = 1;
    public static double DRIVE_ROTATION_SCALE = 1;
    public static double ROTATE_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.03;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);
        float TRIGGER_THRESHOLD = 0.2f;

        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.FullyExtended);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        boolean containerServoToggle = true;
        long previousServoCheck = System.currentTimeMillis();

        while (opModeIsActive()) {
            // prep the values
            float rightx1 = gamepad1.right_stick_x;
            float righty1 = -gamepad1.right_stick_y;
            float leftx1 = gamepad1.left_stick_x;
            float lefty1 = -gamepad1.left_stick_y;
            float heading = mainRobot.imu.getHeading();

            // handle the values
            // drive base controls
            // determine which joystick takes precedence based on magnitude
            double rightMag = Math.hypot(rightx1, righty1);
            double leftMag = Math.hypot(leftx1, lefty1);

            float forward;
            float turn;
            if (leftMag > rightMag) {
                forward = lefty1;
                turn = leftx1;
            } else {
                forward = righty1;
                turn = rightx1;
            }
            float correction = Math.signum(forward) == 0 ? 1: Math.signum(forward);
            mainRobot.driveBase.direct_move_and_turn(forward * (float)DRIVE_FORWARD_SCALE, turn * (float)Math.pow(Math.abs(turn), 1.2f) * (float)DRIVE_ROTATION_SCALE);

            // elevator controls
            if (gamepad1.x) {
               //  mainRobot.hook.goToState(ElevatorHook.State.Contracted);
            } else if (gamepad1.dpad_up) {
                // mainRobot.hook.goToState(ElevatorHook.State.PartiallyExtended);
                // mainRobot.hook.setPowerDirectControl(1);
                mainRobot.hook.setPower(1);
            } else if (gamepad1.dpad_down) {
                // mainRobot.hook.goToState(ElevatorHook.State.FullyExtended);
                // mainRobot.hook.setPowerDirectControl(-1);
                mainRobot.hook.setPower(-1);
            }

            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                mainRobot.hook.setPower(0);
                // mainRobot.hook.setPowerDirectControl(0);
            }

            if (gamepad1.right_bumper) {
            } else if (gamepad1.left_bumper) {
            } else {
            }

            float rightx2 = gamepad2.right_stick_x;
            float righty2 = -gamepad2.right_stick_y;
            float leftx2 = gamepad2.left_stick_x;
            float lefty2 = -gamepad2.left_stick_y;

            mainRobot.grabber.rotate(lefty2);
            mainRobot.grabber.extend(righty2);

            if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                mainRobot.grabber.activate_outtake();
            } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
                mainRobot.grabber.activate_intake();
            } else {
                mainRobot.grabber.stop_intake();
            }

            if (gamepad2.right_bumper) {
                // toggle servo
                if (System.currentTimeMillis() > previousServoCheck + 500) {
                    if (containerServoToggle) {
                        mainRobot.grabber.openContainer();
                    } else {
                        mainRobot.grabber.closeContainer();
                    }
                    containerServoToggle = !containerServoToggle;
                    previousServoCheck = System.currentTimeMillis();
                }
            }

            mainRobot.sampler.contractAll();
            // mainRobot.hook.update();
            // send out info
            mainRobot.sampler.reportInfo(telemetry);
            mainRobot.driveBase.report_encoder_ticks();
            telemetry.addData("heading", heading);
            mainRobot.grabber.reportInfo(telemetry);
            telemetry.addData("arm max down limit", mainRobot.armMaxDownLimit.isPressed());
            telemetry.update();
        }
    }
}
