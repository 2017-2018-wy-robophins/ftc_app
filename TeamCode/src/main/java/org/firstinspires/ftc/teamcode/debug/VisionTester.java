package org.firstinspires.ftc.teamcode.debug;

/**
 * Created by efyang on 1/25/18.
 */

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.firstinspires.ftc.teamcode.common.StartLocation;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Vision Tester", group = "Debug")
public class VisionTester extends LinearOpMode {
    public static double SIDE_LENGTH = 17;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    private static void offsetPoints(double[] xPoints, double[] yPoints, double x, double y) {
        for (int i = 0; i < xPoints.length; i++) {
            xPoints[i] += x;
            yPoints[i] += y;
        }
    }

    private static void drawRobot(double x, double y, double heading, FtcDashboard dashboard) {
            double l = SIDE_LENGTH / 2;
            double[] bxPoints = { l, -l, -l, l, l + 1};
            double[] byPoints = { l, l, -l, -l, 0 };

            rotatePoints(bxPoints, byPoints, Math.toRadians(heading));
            offsetPoints(bxPoints, byPoints, x, y);

            rotatePoints(bxPoints, byPoints, Math.toRadians(-90));

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(4)
                    .setStroke("goldenrod")
                    .strokePolygon(bxPoints, byPoints)
                    .setStroke("purple")
                    .strokeCircle(y, -x, 1);
            dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("Init");
        telemetry.update();

        VuforiaVisionProcessor vision = new VuforiaVisionProcessor(hardwareMap);
        vision.initTfod();

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();
        NavigationalState nav = new NavigationalState(StartLocation.BLUE_LEFT);
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            OpenGLMatrix m = vision.getCurrentPosition();
            if (m != null) {
                nav = new NavigationalState(m, StartLocation.BLUE_LEFT);
            }
            double inx = nav.get_position().get(0) / FieldConstants.mmPerInch;
            double iny = nav.get_position().get(1) / FieldConstants.mmPerInch;
            drawRobot(inx, iny, nav.get_heading(), dashboard);

            telemetry.addData("Position", nav.get_position());
            telemetry.addData("Heading", nav.get_heading());
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        vision.stopTfod();
        stop();
    }
}
