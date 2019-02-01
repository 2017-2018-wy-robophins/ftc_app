package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.components.MPU6050;

@TeleOp(name = "MPU6050 Tester", group = "Debug")
@Config
public class MPU6050Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        MPU6050 mpu6050 = hardwareMap.get(MPU6050.class, "armGyro");
        mpu6050.initialize();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Device Name", mpu6050.getDeviceName());
            telemetry.addData("Raw HW ID", mpu6050.getHardwareIDRaw());
            AngularVelocity angularVelocity = mpu6050.getAngularVelocity();
            telemetry.addData("Angular Velocity x", angularVelocity.xRotationRate);
            telemetry.addData("Angular Velocity y", angularVelocity.yRotationRate);
            telemetry.addData("Angular Velocity z", angularVelocity.zRotationRate);
            Acceleration acceleration = mpu6050.getAcceleration();
            telemetry.addData("Acceleration", acceleration);
            telemetry.addData("Acceleration magnitude", Math.sqrt(acceleration.xAccel * acceleration.xAccel + acceleration.yAccel * acceleration.yAccel + acceleration.zAccel * acceleration.zAccel));
            telemetry.update();
        }
    }
}
