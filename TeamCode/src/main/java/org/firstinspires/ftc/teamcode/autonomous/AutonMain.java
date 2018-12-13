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

        telemetry.addData("Start Location", startLocation);
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
                        new HookControlCommand(ElevatorHook.State.FullyExtended),
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
                        new HookControlCommand(ElevatorHook.State.FullyExtended),
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
                        new HookControlCommand(ElevatorHook.State.FullyExtended),
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
                        new HookControlCommand(ElevatorHook.State.FullyExtended),
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

        for (Command command: commandList) {
            command.execute(navinfo, imu, visionProcessor, mainRobot, telemetry);
        }
        /*
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
        */
    }
    // true to continue, false to stop
    boolean mainLoop() throws InterruptedException {
        return false;
    }
    void finish() throws InterruptedException {}
}
