package org.firstinspires.ftc.teamcode.autonomous.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

public class MovementCommand extends Command {
    private VectorF targetPosition;
    private float targetHeading;

    public MovementCommand(VectorF targetPosition, float targetHeading) {
        this.targetPosition = targetPosition;
        this.targetHeading = targetHeading;
    }

    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        telemetry.addData("Moving to target position", targetPosition);
        telemetry.addData("Moving to target heading", targetHeading);
        telemetry.update();

        // get the angle we need to rotate to first to move correctly
        VectorF movement_vector = targetPosition.subtracted(navigationalState.get_position());
        float movement_angle = (float)Math.toDegrees(Math.atan2(movement_vector.get(1), movement_vector.get(0)));
        float initial_rotation = navigationalState.get_robot_rotation(movement_angle);
        navigationalState.set_heading(movement_angle);
        float final_rotation = navigationalState.get_robot_rotation(targetHeading);
        float movement_amount = movement_vector.magnitude();
        mainRobot.driveBase.imu_turn(initial_rotation, imu);
        mainRobot.driveBase.imu_forward_move(movement_amount, imu);
        mainRobot.driveBase.imu_turn(final_rotation, imu);
        navigationalState.set_position(targetPosition);
        navigationalState.set_heading(targetHeading);
    }
}
