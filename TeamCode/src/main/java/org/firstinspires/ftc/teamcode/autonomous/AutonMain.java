package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.autonomous.commands.ArmDeployCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.BeginCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.ClaimCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.Command;
import org.firstinspires.ftc.teamcode.autonomous.commands.CommandTree;
import org.firstinspires.ftc.teamcode.autonomous.commands.CommandTreeBuilder;
import org.firstinspires.ftc.teamcode.autonomous.commands.FinishCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.HookControlCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.MovementCommand;
import org.firstinspires.ftc.teamcode.autonomous.commands.SampleCommand;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
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
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

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

    AutonMain(HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted);
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
        navinfo = new NavigationalState(location, startLocation);

        telemetry.addData("location", location);
        telemetry.addData("Start Location", startLocation);
        telemetry.addData("location2", navinfo.get_position());
        telemetry.update();
    }

    void runOnce() throws InterruptedException {
        // currently no need for commandtree - everything is being run serially
        Command[] commandList = null;
        switch (startLocation) {
            case RED_LEFT:
                commandList = new Command[] {
                        new BeginCommand(),
                        //new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(609.6f, -609.6f, -45, true),
                        new SampleCommand(),
                        new MovementCommand(1524f, 304.8f, 90, true),
                        new MovementCommand(1524f, 1219.2f, 90, true),
                        new ClaimCommand(),
                        new MovementCommand(1524f, -304.8f, 90, false),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case BLUE_LEFT:
                commandList = new Command[] {
                        new BeginCommand(),
                        //new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(-609.6f, 609.6f, 135, true),
                        new SampleCommand(),
                        new MovementCommand(-1524f, -304.8f, -90, true),
                        new MovementCommand(-1524f, -1219.2f, -90, true),
                        new ClaimCommand(),
                        new MovementCommand(-1524f, 304.8f, -90, false),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case RED_RIGHT:
                commandList = new Command[] {
                        new BeginCommand(),
                        //new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(609.6f, 609.6f, 45, true),
                        new SampleCommand(),
                        new ClaimCommand(),
                        new MovementCommand(1219.2f, 1524, 180, true),
                        new MovementCommand(-609.6f, 1524, 180, true),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case BLUE_RIGHT:
                commandList = new Command[] {
                        new BeginCommand(),
                        //new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(-609.6f, -609.6f, -135, true),
                        new SampleCommand(),
                        new ClaimCommand(),
                        //TODO Reverse onto crater please
                        new MovementCommand(-1219.2f, -1524, 0, true),
                        new MovementCommand(609.6f, -1524, 0, true),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
        }
        System.out.println("Running command list");
        for (Command command: commandList) {
            command.execute(navinfo, imu, visionProcessor, mainRobot, telemetry);
            Thread.sleep(2000);
        }
        System.out.println("Done");
        /*
        telemetry.addLine("Starting Elevator");
        telemetry.update();
        mainRobot.hook.goToStateBlocking(ElevatorHook.State.FullyExtended);

        mainRobot.driveBase.imu_forward_move(375, mainRobot.imu);

        Thread.sleep(2000);
        SamplingConfiguration samplingConfiguration = visionProcessor.getSamplingConfigurationPhoneRightOnlyGold();
        if (samplingConfiguration == null) {
            telemetry.addLine("Didn't get sampling configuration, defaulting to center");
            samplingConfiguration = SamplingConfiguration.CENTER;
        } else {
            telemetry.addData("Got sampling configuration", samplingConfiguration);
        }
        telemetry.update();
        visionProcessor.stopTfod();

        servoSample(samplingConfiguration, mainRobot);
        */
    }

    // true to continue, false to stop
    boolean mainLoop() throws InterruptedException {
        return false;
    }
    void finish() throws InterruptedException {
        visionProcessor.stopTfod();
    }
}
