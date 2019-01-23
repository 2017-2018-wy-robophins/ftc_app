package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class MovementCommand extends Command {
    //A command that instructs the robot to move based on imu sensor information. The robot will first turn towards the direction in question,
    //(or to face away from it, in event that "forward" = false), then move to target position. Finally, it will turn to face in the direction
    //specified by "targetHeading".

    //Implements PID control

    private VectorF targetPosition;
    private float targetHeading;
    private boolean forward;

    public MovementCommand(float x, float y, float targetHeading, boolean forward) {
        this(new VectorF(x, y), targetHeading, forward);
    }

    public MovementCommand(VectorF targetPosition, float targetHeading, boolean forward) {
        this.targetPosition = targetPosition;
        this.targetHeading = targetHeading;
        this.forward = forward;
    }

    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        System.out.println(navigationalState);
        System.out.println(targetPosition);
        System.out.println(targetHeading);
        telemetry.addData("Moving to target position", targetPosition);
        telemetry.addData("Moving to target heading", targetHeading);
        telemetry.update();

        // get the angle we need to rotate to first to move correctly
        VectorF movement_vector = targetPosition.subtracted(navigationalState.get_position());
        float movement_angle;
        if (forward) {
            movement_angle = (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));
        } else {
            movement_angle = 180 + (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));
        }
        telemetry.addData("Movement angle", movement_angle);
        float initial_rotation = navigationalState.get_robot_rotation(movement_angle);
        telemetry.addData("initial angle", initial_rotation);
        navigationalState.set_heading(movement_angle);
        float final_rotation = navigationalState.get_robot_rotation(targetHeading);
        telemetry.addData("final angle", final_rotation);
        telemetry.update();

        float movement_amount;
        if (forward) {
            movement_amount = movement_vector.magnitude();
        } else {
           movement_amount = -movement_vector.magnitude();
        }

        //Using calculated values, move robot accordingly.
        mainRobot.driveBase.imu_turn(initial_rotation, imu);
        mainRobot.driveBase.imu_forward_move(movement_amount, imu);
        mainRobot.driveBase.imu_turn(final_rotation, imu);

        //Record final position and heading to the navigationalState.
        navigationalState.set_position(targetPosition);
        navigationalState.set_heading(targetHeading);
    }
}
