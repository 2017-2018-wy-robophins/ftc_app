package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by efyang on 2/2/18.
 */

public final class FieldConstants {
    public static final float mmPerInch = 25.4f;
    public static final float mmPerBlock = mmPerInch * 24;
    public static final float mmFTCFieldWidth = (24 * 6 - 2) * mmPerInch;
    public static final float mmPictographHeight = 2 * mmPerInch;

    public static final OpenGLMatrix blueRightTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float) (24 * 5 + 3 + 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    public static final OpenGLMatrix blueLeftTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float) (24 * 2 + 3 + 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    public static final OpenGLMatrix redRightTargetLocationOnField = OpenGLMatrix
            .translation(0, (float) (24 * 2 - 3 - 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    public static final OpenGLMatrix redLeftTargetLocationOnField = OpenGLMatrix
            .translation(0, (float) (24 * 5 - 3 - 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    public static final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(-15f * 10, -21.5f * 10, 22 * 10)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, -90, 90, 0));
    // we use the robot pointing to the right as the default axial system here
}
