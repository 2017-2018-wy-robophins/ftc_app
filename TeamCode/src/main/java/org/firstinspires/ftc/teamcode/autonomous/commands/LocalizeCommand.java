package org.firstinspires.ftc.teamcode.autonomous.commands;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;

import static org.firstinspires.ftc.teamcode.common.ExtendedMath.decompose_opengl_matrix;
import static org.firstinspires.ftc.teamcode.common.ExtendedMath.extract_z_rot;

public class LocalizeCommand extends Command {

    //A command to locate the robot in the field, using the Vuforia vision processing library.
    void executeCommand(NavigationalState navigationalState, InertialSensor imu, VisionProcessor visionProcessor, MainRobot mainRobot, Telemetry telemetry) {
        OpenGLMatrix info = visionProcessor.getCurrentPosition();
        Pair<VectorF, MatrixF> decomposed = ExtendedMath.decompose_opengl_matrix(info);
        navigationalState.set_heading(extract_z_rot(info));
        navigationalState.set_position(decomposed.first);
    }
}
