package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensorBNO055;

@TeleOp(name = "Gyro Tester", group = "Debug")
public class GyroTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        InertialSensorBNO055 gyro = new InertialSensorBNO055(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Heading", gyro.getHeading());
            telemetry.update();
        }
    }
}
