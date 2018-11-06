package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MainRobot;

@TeleOp(name = "Main Op Mode", group = "main")
public class MainOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.05;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);

        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, false);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        while (opModeIsActive()) {
            float rightx = gamepad1.right_stick_x;
            float righty = -gamepad1.right_stick_y;
            float leftx = gamepad1.left_stick_x;
            float lefty = -gamepad1.left_stick_y;

            mainRobot.driveBase.direct_move_and_turn(lefty, rightx);

            float heading = mainRobot.imu.getHeading();

            mainRobot.driveBase.report_encoder_ticks();
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
}
