package org.firstinspires.ftc.teamcode.components.positionFinder;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by efyang on 2/13/18.
 */

//Returns the robot's position matrix.
public interface PositionFinder {
    OpenGLMatrix getCurrentPosition() throws InterruptedException;
}
