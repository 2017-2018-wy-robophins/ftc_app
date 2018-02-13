package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by efyang on 2/13/18.
 */

interface PositionFinder {
    Pair<OpenGLMatrix, RelicRecoveryVuMark> getCurrentPosition() throws InterruptedException;
}
