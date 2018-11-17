package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.StartLocation;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensorBNO055;
import org.firstinspires.ftc.teamcode.components.positionFinder.DebugPositionFinder;
import org.firstinspires.ftc.teamcode.components.positionFinder.PositionFinder;
import org.firstinspires.ftc.teamcode.components.positionFinder.VuforiaPositionFinder;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;

import java.lang.reflect.Field;

/**
 * Created by nico on 11/14/17.
 */

public class AutonMain {
    private Telemetry telemetry;
    private StartLocation startLocation;
    private MainRobot mainRobot;
    // hardware components
    private NavigationalState navinfo;
    private VuforiaVisionProcessor visionProcessor;
    private InertialSensor imu;
    private final boolean DEBUG = false;
    private final boolean DEBUG_CLASSES = false;

    AutonMain(HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, DEBUG_CLASSES);
        // instantiate hardware variables

        // TODO: Determine if we always want this in mainrobot - it only gets used in auton really - have it specific to auton
        // create the imu
        imu = new InertialSensorBNO055(hardwareMap);

        // create the vision processor
        visionProcessor = new VuforiaVisionProcessor(hardwareMap);
        //visionProcessor.initTfod();

        OpenGLMatrix location = null;
        switch (startLocation) {
            case RED_LEFT:
                location = FieldConstants.redLeftStartLocation;
                break;
            case RED_RIGHT:
                location = FieldConstants.redRightStartLocation;
                break;
            case BLUE_LEFT:
                location = FieldConstants.blueLeftStartLocation;
                break;
            case BLUE_RIGHT:
                location = FieldConstants.blueRightStartLocation;
                break;
        }
        navinfo = new NavigationalState(location);

        telemetry.addData("Start Location", startLocation);
        telemetry.update();
    }
    void runOnce() throws InterruptedException {
        mainRobot.hook.goToStateBlocking(ElevatorHook.State.FullyExtended);

        switch (startLocation) {
            case RED_LEFT:
            case BLUE_LEFT:
                // 2
                mainRobot.driveBase.forward_move(862f);
                mainRobot.hook.goToState(ElevatorHook.State.Contracted);
                // 3
                mainRobot.driveBase.forward_move(-431f);
                // 4
                mainRobot.driveBase.turn(90);
                mainRobot.driveBase.forward_move(1300);
                // 5
                mainRobot.driveBase.turn(45);
                mainRobot.driveBase.forward_move(914.4f);
                mainRobot.grabber.activate_outtake();
                Thread.sleep(500);
                mainRobot.grabber.stop_intake();
                // 6
                mainRobot.driveBase.turn(180);
                mainRobot.driveBase.forward_move(1828.8f);
                mainRobot.grabber.deploy();
                break;
            case RED_RIGHT:
            case BLUE_RIGHT:
                // 2
                mainRobot.driveBase.forward_move(1300);
                mainRobot.hook.goToState(ElevatorHook.State.Contracted);
                mainRobot.grabber.activate_outtake();
                Thread.sleep(500);
                mainRobot.grabber.stop_intake();
                // 3
                mainRobot.driveBase.turn(45);
                mainRobot.driveBase.forward_move(431);
                // 4
                mainRobot.driveBase.turn(45);
                mainRobot.driveBase.forward_move(900);
                mainRobot.grabber.deploy();
                break;
        }
    }
    // true to continue, false to stop
    boolean mainLoop() throws InterruptedException {
        return false;
    }
    void finish() throws InterruptedException {}

    private void goToLocationWithHeading(VectorF targetPosition, float targetHeading) {
        telemetry.addData("Moving to target position", targetPosition);
        telemetry.addData("Moving to target heading", targetHeading);
        telemetry.update();

        // get the angle we need to rotate to first to move correctly
        VectorF movement_vector = targetPosition.subtracted(navinfo.get_position());
        float movement_angle = (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));
        float initial_rotation = navinfo.get_robot_rotation(movement_angle);
        navinfo.set_heading(movement_angle);
        float final_rotation = navinfo.get_robot_rotation(targetHeading);
        float movement_amount = movement_vector.magnitude();
        mainRobot.driveBase.imu_turn(initial_rotation, imu);
        mainRobot.driveBase.imu_forward_move(movement_amount, imu);
        mainRobot.driveBase.imu_turn(final_rotation, imu);
        navinfo.set_position(targetPosition);
        navinfo.set_heading(targetHeading);
    }
}
