package org.firstinspires.ftc.teamcode.debug;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensorBNO055;

@Disabled
@TeleOp(name = "IMU Tester", group = "Debug")
@Config
public class ImuTester extends LinearOpMode {
    static float targetHeading = 90;
    @Override
    public void runOpMode() throws InterruptedException {
        InertialSensor imu = new InertialSensorBNO055(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Heading error", getHeadingError(targetHeading, imu));
            telemetry.update();
        }
    }

    private float getHeadingError(float targetHeading, InertialSensor imu) {
        float positiveError = ExtendedMath.get_min_rotation(imu.getHeading(), targetHeading);
        // now normalize to +-180 for convenience
        return positiveError + (positiveError > 180 ? -360 : 0);
    }
}
