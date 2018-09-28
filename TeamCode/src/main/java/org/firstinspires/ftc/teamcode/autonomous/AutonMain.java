package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.StartLocation;
import org.firstinspires.ftc.teamcode.components.positionFinder.DebugPositionFinder;
import org.firstinspires.ftc.teamcode.components.positionFinder.PositionFinder;
import org.firstinspires.ftc.teamcode.components.positionFinder.VuforiaPositionFinder;

/**
 * Created by nico on 11/14/17.
 */

public class AutonMain {
    private Telemetry telemetry;
    private StartLocation startLocation;
    private MainRobot robot;
    // hardware components
    private NavigationalState navinfo;
    private PositionFinder positionFinder;
    private RelicRecoveryVuMark vumark;
    private final boolean DEBUG = false;
    private final boolean DEBUG_CLASSES = false;

    AutonMain(HardwareMap hardwareMap, Telemetry telemetry, StartLocation startLocation) throws InterruptedException {
        this.telemetry = telemetry;
        this.startLocation = startLocation;
        robot = new MainRobot(hardwareMap, telemetry, DEBUG_CLASSES);
        // instantiate hardware variables

        navinfo = new NavigationalState();
        telemetry.addData("Start Location", startLocation);
        telemetry.addLine("Initializing vuforia...");
        telemetry.update();
        if (!DEBUG_CLASSES) {
            positionFinder = new VuforiaPositionFinder(startLocation, hardwareMap);
        } else {
            positionFinder = new DebugPositionFinder();
        }
        telemetry.addLine("Initialized vuforia.");
        telemetry.update();
        vumark = RelicRecoveryVuMark.CENTER;
    }

    // run this once
    void runOnce() throws InterruptedException {
        // get position using vuforia
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = positionFinder.getCurrentPosition();
        int vuforia_try_count = 1;
        int vuforia_max_tries = 3;

        while ((vuforia_try_count <= vuforia_max_tries) && (position == null)) {
            telemetry.addLine("Did not get vuforia position on try: " + vuforia_try_count + ", trying again.");
            telemetry.update();
            // robot.driveBase.move_ms(-start_move_x * 0.7f, -start_move_y * 0.7f, 500);
            position = positionFinder.getCurrentPosition();
            vuforia_try_count += 1;
        }

        // if the position wasn't found
        boolean run_fallback = false;
        if (position != null) {
            navinfo = new NavigationalState(position.first);
            vumark = position.second;
            telemetry.addLine("Found position with vuforia: " + navinfo);
            telemetry.addData("Target", vumark);
            telemetry.update();
        } else {
            // fallback
            telemetry.addLine("Could not get vuforia position, running fallback");
            telemetry.update();
            run_fallback = true;
        }


        telemetry.addLine("Finished");
        telemetry.update();
        robot.driveBase.stop();
    }

    // return whether we want to continue or not
    boolean mainLoop() throws InterruptedException {
        return false;
    }

    void finish() throws InterruptedException {}

    //https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6170-encodejavars-and-autonomous
    private void move_to_position_with_heading(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        robot.driveBase.move_by_vector_and_rotation(
                navinfo.get_robot_movement_vector(target_pos),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_split_rotation(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        robot.driveBase.move_by_vector_and_rotation(
                navinfo.get_robot_movement_vector(target_pos),
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                new VectorF(0, 0),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_no_diag(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        VectorF movement_vector = navinfo.get_robot_movement_vector(target_pos);
        VectorF[] movement_component_vectors = ExtendedMath.vector_components(movement_vector);
        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[0],
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[1],
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_no_diag_split_rotation(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);

        VectorF movement_vector = navinfo.get_robot_movement_vector(target_pos);
        VectorF[] movement_component_vectors = ExtendedMath.vector_components(movement_vector);

        telemetry.addData("Movement vector", movement_vector);
        telemetry.addData("Rotation", navinfo.get_robot_rotation(target_heading));
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(1000);
        }
        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[0].multiplied(2f),
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        telemetry.addLine("Move X complete");
        telemetry.update();
        Thread.sleep(500);

        robot.driveBase.move_by_vector_and_rotation(
                movement_component_vectors[1],
                0,
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );
        telemetry.addLine("Move Y complete");
        telemetry.update();
        Thread.sleep(500);

        robot.driveBase.move_by_vector_and_rotation(
                new VectorF(0, 0),
                navinfo.get_robot_rotation(target_heading),
                power,
                RobotConstants.TICK_ALLOWED_ABS_ERROR,
                timeout
        );

        telemetry.addLine("Rotate complete");
        telemetry.update();
        Thread.sleep(500);
        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }

    private void move_to_position_with_heading_only_vdrive(VectorF target_pos, float target_heading, float power, int timeout) throws InterruptedException {
        telemetry.addData("Moving to target position", target_pos);
        telemetry.addData("Moving to target heading", target_heading);
        telemetry.update();
        if (DEBUG) {
            Thread.sleep(250);
        }
        // get the angle we need to rotate to first to move correctly
        VectorF movement_vector = target_pos.subtracted(navinfo.get_position());
        float movement_angle = (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));

        float initial_rotation = navinfo.get_robot_rotation(movement_angle);
        navinfo.set_heading(movement_angle);
        float final_rotation = navinfo.get_robot_rotation(target_heading);
        float movement_amount = movement_vector.magnitude();
        robot.driveBase.rotate_and_move_only_vertical_drive(initial_rotation, movement_amount, final_rotation, power, RobotConstants.TICK_ALLOWED_ABS_ERROR, timeout);

        navinfo.set_position(target_pos);
        navinfo.set_heading(target_heading);
    }
}
