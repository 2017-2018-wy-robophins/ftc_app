package org.firstinspires.ftc.teamcode.components.positionFinder;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by efyang on 2/13/18.
 */

public interface PositionFinder {
    OpenGLMatrix getCurrentPosition() throws InterruptedException;
}
