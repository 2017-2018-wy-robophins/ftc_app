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
import java.util.concurrent.CompletableFuture;

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
        mainRobot.armRotate.setPower(0);
        // instantiate hardware variables

        // create the imu
        imu = new InertialSensorBNO055(hardwareMap);

        // create the vision processor
        visionProcessor = new VuforiaVisionProcessor(hardwareMap);
        visionProcessor.initTfod();

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
        navinfo.imuOffset = ExtendedMath.extract_z_rot(location);

        telemetry.addData("Start Location", startLocation);
        telemetry.update();
    }

    void runOnce() throws InterruptedException {
        telemetry.addLine("Starting Elevator");
        telemetry.update();
        mainRobot.hook.goToStateBlocking(ElevatorHook.State.FullyExtended);

        telemetry.addLine("Starting movement code");
        telemetry.update();
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
}
