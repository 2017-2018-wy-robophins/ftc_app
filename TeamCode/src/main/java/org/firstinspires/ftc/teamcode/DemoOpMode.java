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
@TeleOp(name = "Demo Op Mode", group = "main")
public class DemoOpMode extends LinearOpMode {
    // need double to be tuned with ftc dashboard
    public static double DRIVE_FORWARD_SCALE = 1;
    public static double DRIVE_ROTATION_SCALE = 0.8;
    public static double ROTATE_POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.05;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);
        float TRIGGER_THRESHOLD = 0.3f;

        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.FullyExtended);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        ControlMode controlMode = ControlMode.Improved;
        boolean containerServoToggle = false;
        long previousServoCheck = System.currentTimeMillis();

        while (opModeIsActive()) {
            // prep the values
            float rightx = gamepad1.right_stick_x;
            float righty = -gamepad1.right_stick_y;
            float leftx = gamepad1.left_stick_x;
            float lefty = -gamepad1.left_stick_y;
            float heading = mainRobot.imu.getHeading();

            // if all 4 pressed at the same time, toggle control mode
            boolean toggling = false;
            if (gamepad1.x && gamepad1.y && gamepad1.a && gamepad1.b) {
                controlMode = controlMode.toggle();
                toggling = true;
            }

            // handle the values
            switch (controlMode) {
                case Improved:
                    // drive base controls
                    // determine which joystick takes precedence based on magnitude
                    double rightMag = Math.hypot(rightx, righty);
                    double leftMag = Math.hypot(leftx, lefty);

                    float forward;
                    float turn;
                    if (leftMag > rightMag) {
                        forward = lefty;
                        turn = leftx;
                    } else {
                        forward = righty;
                        turn = rightx;
                    }
                    mainRobot.driveBase.direct_move_and_turn(forward * (float)DRIVE_FORWARD_SCALE, turn * (float)DRIVE_ROTATION_SCALE);

                    // elevator controls
                    if (!toggling) {
                        if (gamepad1.x) {
                           //  mainRobot.hook.goToState(ElevatorHook.State.Contracted);
                        } else if (gamepad1.a) {
                            // mainRobot.hook.goToState(ElevatorHook.State.PartiallyExtended);
                            // mainRobot.hook.setPowerDirectControl(1);
                            mainRobot.hook.setPower(1);
                        } else if (gamepad1.b) {
                            // mainRobot.hook.goToState(ElevatorHook.State.FullyExtended);
                            // mainRobot.hook.setPowerDirectControl(-1);
                            mainRobot.hook.setPower(-1);
                        } else if (gamepad1.y) {
                            // toggle servo
                            if (System.currentTimeMillis() > previousServoCheck + 500) {
                                if (containerServoToggle) {
                                    mainRobot.sampler.extendAll();
                                } else {
                                    mainRobot.sampler.contractAll();
                                }
                                containerServoToggle = !containerServoToggle;
                                previousServoCheck = System.currentTimeMillis();
                            }
                        }
                    }

                    if (!gamepad1.a && !gamepad1.b) {
                        mainRobot.hook.setPower(0);
                        // mainRobot.hook.setPowerDirectControl(0);
                    }

                    // intake controls
                    if (gamepad1.dpad_up) {
                        // rotate up
                        mainRobot.grabber.rotate((float)ROTATE_POWER);
                    } else if (gamepad1.dpad_down) {
                        // rotate down
                        mainRobot.grabber.rotate(-(float)ROTATE_POWER);
                    } else {
                        mainRobot.grabber.rotate(0);
                    }
                    mainRobot.grabber.autoBalance();

                    if (gamepad1.right_bumper) {
                        // contract
                        mainRobot.grabber.extend(-1);
                    } else if (gamepad1.left_bumper) {
                        // extend
                        mainRobot.grabber.extend(1);
                    } else {
                        mainRobot.grabber.extend(0);
                    }

                    if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                        mainRobot.intake.setPosition(0.5f + 0.5*Math.pow(gamepad1.right_trigger, 5));
                    } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                        mainRobot.intake.setPosition(0.5f + 0.5*-Math.pow(gamepad1.left_trigger, 5));
                    } else {
                        // TODO: not sure if this should be done
                        mainRobot.grabber.stop_intake();
                    }

                    break;
                case Manual:
                    mainRobot.rightDrive.setPower(righty);
                    mainRobot.leftDrive.setPower(lefty);

                    conditionalPower(mainRobot.leftRotate, gamepad1.dpad_up, gamepad1.dpad_down);
                    conditionalPower(mainRobot.rightRotate, gamepad1.dpad_up, gamepad1.dpad_down);
                    conditionalPower(mainRobot.armExtend, gamepad1.dpad_left, gamepad1.dpad_right);
                    // conditionalPower(mainRobot.intake, gamepad1.right_bumper, gamepad1.right_trigger > TRIGGER_THRESHOLD);

                    conditionalPower(mainRobot.leftElevator, gamepad1.y, gamepad1.x);
                    conditionalPower(mainRobot.rightElevator, gamepad1.b, gamepad1.a);

                    // TODO: FINISH
                    break;
            }

            // mainRobot.sampler.contractAll();
            // mainRobot.hook.update();
            // send out info
            mainRobot.sampler.reportInfo(telemetry);
            mainRobot.driveBase.report_encoder_ticks();
            telemetry.addData("heading", heading);
            telemetry.addData("Control Mode", controlMode);
            mainRobot.grabber.reportInfo(telemetry);
            telemetry.update();
        }
    }

    private void conditionalPower(DcMotor motor, boolean c1, boolean c2) {
        if (c1) {
            motor.setPower(1);
        } else if (c2) {
            motor.setPower(-1);
        } else {
            motor.setPower(0);
        }
    }

    enum ControlMode {
        Improved,
        Manual;

        private ControlMode next;
        static {
            Improved.next = Manual;
            Manual.next = Improved;
        }

        public ControlMode toggle() {
            return next;
        }
    }
}
