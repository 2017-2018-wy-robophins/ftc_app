package org.firstinspires.ftc.teamcode.components.positionFinder;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.components.positionFinder.PositionFinder;

/**
 * Created by efyang on 2/13/18.
 */

public class DebugPositionFinder implements PositionFinder {
    public Pair<OpenGLMatrix, RelicRecoveryVuMark> getCurrentPosition() {
            return null;
    }
}
