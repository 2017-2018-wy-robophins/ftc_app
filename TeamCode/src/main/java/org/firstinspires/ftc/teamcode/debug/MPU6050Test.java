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
        MPU6050 mpu6050 = hardwareMap.get(MPU6050.class, "armGyro");
        mpu6050.initialize();

        double kA = 0.98;
        double kG = 0.02;

        double r = 0.15;

        double theta = 0;
        long time_millis = System.currentTimeMillis();

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

            double omega = angularVelocity.zRotationRate;
            double ay = acceleration.yAccel;
            double ax = acceleration.xAccel;
            long current_time_millis = System.currentTimeMillis();
            long dt_millis = current_time_millis - time_millis;
            double dt = ((double)dt_millis)/1000;
            time_millis = current_time_millis;

            theta = kA * Math.atan2(omega * omega * r - ay, ax) + kG * (theta + omega * dt);
            telemetry.addData("Calculated angle", theta);

            telemetry.update();
        }
    }
}
