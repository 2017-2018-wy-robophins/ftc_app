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
    private final boolean DEBUG = false;
    private final boolean DEBUG_CLASSES = false;

    AutonMain(HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, DEBUG_CLASSES);
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

    /*
    private void runCommandTree(CommandTree commandTree) {
        try {
            commandTree.toFuture(navinfo, imu, visionProcessor, mainRobot, telemetry).get();
        } catch (ExecutionException e) {
            telemetry.addLine("Failed on commandtree execution");
            telemetry.update();
        } catch (InterruptedException e) {
            telemetry.addLine("CommandTree execution interrupted");
            telemetry.update();
        }
    }*/

    void runOnce() throws InterruptedException {
        // currently no need for commandtree - everything is being run serially
        /*
        CommandTree leftCommandTree = new CommandTreeBuilder(new BeginCommand())
                .addChildCommand(new CommandTreeBuilder(new HookControlCommand(ElevatorHook.State.FullyExtended))
                        .create())
                .create();
        CommandTree rightCommandTree = new CommandTreeBuilder(new BeginCommand())
                .addChildCommand(new CommandTreeBuilder(new HookControlCommand(ElevatorHook.State.FullyExtended))
                        .create())
                .create();

        switch (startLocation) {
            case RED_LEFT:
            case BLUE_LEFT:
                runCommandTree(leftCommandTree);
                break;
            case RED_RIGHT:
            case BLUE_RIGHT:
                runCommandTree(rightCommandTree);
                break;
        }*/

        Command[] commandList = null;
        switch (startLocation) {
            case RED_LEFT:
                commandList = new Command[] {
                        new BeginCommand(),
                        // new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(609.6f, -609.6f, -45),
                        new SampleCommand(),
                        new MovementCommand(1524f, 304.8f, 90),
                        new MovementCommand(1524f, 1219.2f, 90),
                        new ClaimCommand(),
                        new MovementCommand(1524f, 304.8f, -90),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case BLUE_LEFT:
                commandList = new Command[] {
                        new BeginCommand(),
                        // new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(-609.6f, 609.6f, -135),
                        new SampleCommand(),
                        new MovementCommand(-1524f, -304.8f, -90),
                        new MovementCommand(-1524f, -1219.2f, -90),
                        new ClaimCommand(),
                        new MovementCommand(-1524f, 304.8f, 90),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case RED_RIGHT:
                commandList = new Command[] {
                        new BeginCommand(),
                        // new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(609.6f, 609.6f, 45),
                        new SampleCommand(),
                        new ClaimCommand(),
                        new MovementCommand(1219.2f, 1524, 180),
                        new MovementCommand(-609.6f, 1524, 180),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
            case BLUE_RIGHT:
                commandList = new Command[] {
                        new BeginCommand(),
                        // new HookControlCommand(ElevatorHook.State.FullyExtended),
                        new MovementCommand(-609.6f, -609.6f, -135),
                        new SampleCommand(),
                        new ClaimCommand(),
                        new MovementCommand(-1219.2f, -1524, 0),
                        new MovementCommand(609.6f, -1524, 0),
                        new ArmDeployCommand(),
                        new FinishCommand()
                };
                break;
        }

        // mainRobot.hook.goToStateBlocking(ElevatorHook.State.FullyExtended);

        /*
        mainRobot.driveBase.imu_forward_move(400f, mainRobot.imu);
        mainRobot.sampler.extendCenter();
        Thread.sleep(500);
        mainRobot.sampler.contractCenter();*/

        /*System.out.println("Running command list");
        for (Command command: commandList) {
            command.execute(navinfo, imu, visionProcessor, mainRobot, telemetry);
            // Thread.sleep(1000);
        }
        System.out.println("DOne");*/
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
/*
        telemetry.addLine("Starting movement code");
        telemetry.update();
        switch (startLocation) {
            case RED_LEFT:
            case BLUE_LEFT:
                mainRobot.driveBase.imu_turn(90, mainRobot.imu);
                mainRobot.driveBase.imu_forward_move(1300, mainRobot.imu);

                mainRobot.grabber.openContainer();
                Thread.sleep(500);
                mainRobot.grabber.intakeMotor.setPower(0.1);
                Thread.sleep(200);
                mainRobot.grabber.intakeMotor.setPower(0);
                mainRobot.grabber.closeContainer();
                mainRobot.grabber.activate_outtake();
                Thread.sleep(500);
                mainRobot.grabber.stop_intake();

                mainRobot.driveBase.imu_turn(180, mainRobot.imu);
                // 6
                mainRobot.driveBase.imu_forward_move(1828.8f, mainRobot.imu);
                break;
            case RED_RIGHT:
            case BLUE_RIGHT:
                mainRobot.driveBase.imu_forward_move(-375, mainRobot.imu);
                mainRobot.driveBase.imu_turn(45, mainRobot.imu);
                mainRobot.driveBase.imu_forward_move(1219, mainRobot.imu);
                mainRobot.driveBase.imu_turn(-90, mainRobot.imu);
                mainRobot.driveBase.imu_forward_move(914, mainRobot.imu);
                // 2

                mainRobot.grabber.openContainer();
                Thread.sleep(500);
                mainRobot.grabber.intakeMotor.setPower(0.3);
                Thread.sleep(200);
                mainRobot.grabber.intakeMotor.setPower(0);
                mainRobot.grabber.closeContainer();
                mainRobot.grabber.activate_outtake();
                Thread.sleep(500);
                mainRobot.grabber.stop_intake();
                // 3
                mainRobot.driveBase.imu_turn(180, mainRobot.imu);
                mainRobot.driveBase.imu_forward_move(1524, mainRobot.imu);
                break;
        }*/
    }

        private void servoSample(SamplingConfiguration samplingConfiguration, MainRobot mainRobot) throws InterruptedException {
        int time = 2000;
        switch (samplingConfiguration) {
            case LEFT:
                mainRobot.sampler.extendLeft();
                Thread.sleep(time);
                //mainRobot.sampler.contractLeft();
                break;
            case RIGHT:
                mainRobot.sampler.extendRight();
                Thread.sleep(time);
                // mainRobot.sampler.contractRight();
                break;
            case CENTER:
                mainRobot.sampler.extendCenter();
                Thread.sleep(time);
                // mainRobot.sampler.contractCenter();
                break;
        }
    }

    // true to continue, false to stop
    boolean mainLoop() throws InterruptedException {
        return false;
    }
    void finish() throws InterruptedException {
        visionProcessor.stopTfod();
    }
}
