package org.firstinspires.ftc.teamcode;

/**
 * Created by efyang on 1/25/18.
 */

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name = "Color Sensor Tester", group = "Robot")
public class ColorSensorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        ColorSensor sensorColor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        sensorColor.enableLed(false);
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.addData("red", sensorColor.red());
            telemetry.addData("blue", sensorColor.blue());
            telemetry.addData("green", sensorColor.green());
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
