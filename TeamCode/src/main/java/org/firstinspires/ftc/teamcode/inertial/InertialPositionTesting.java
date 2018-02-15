package org.firstinspires.ftc.teamcode.inertial;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.inertial.InertialPositionUpdater;

/**
 * Created by efyang on 1/4/18.
 */

@Autonomous(name="Inertial Position Testing", group="Concept")
class InertialPositionTesting extends LinearOpMode {
    InertialPositionUpdater position_updater;
    @Override public void runOpMode() {
        telemetry.addLine("Inertial Position Testing Module");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Still Ok");
        telemetry.update();
        position_updater = new InertialPositionUpdater(OpenGLMatrix.identityMatrix(), hardwareMap.appContext);
        while (opModeIsActive()) {
            if (position_updater.getAcceleration() != null && position_updater.getOmega() != null) {
                telemetry.clearAll();
                telemetry.addData("Raw Acceleration", position_updater.getAcceleration());
                telemetry.addData("Raw Omega", position_updater.getOmega());
                telemetry.addData("Position", position_updater.getPosition());
                telemetry.addData("Velocity", position_updater.getVelocity());
                telemetry.addData("Heading", position_updater.getHeading());
                telemetry.addData("Timestamp", position_updater.getPrevious_timestamp());
                telemetry.update();
            }
        }
    }

}
