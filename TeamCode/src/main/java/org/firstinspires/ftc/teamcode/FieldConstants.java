package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by efyang on 2/2/18.
 */

final class FieldConstants {
    static final float mmPerInch = 25.4f;
    static final float mmPerBlock = mmPerInch * 24;
    static final float mmFTCFieldWidth = (24 * 6 - 2) * mmPerInch;
    static final float mmPictographHeight = 2 * mmPerInch;

    static final OpenGLMatrix blueRightTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float) (24 * 5 + 3 + 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    static final OpenGLMatrix blueLeftTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float) (24 * 2 + 3 + 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    static final OpenGLMatrix redRightTargetLocationOnField = OpenGLMatrix
            .translation(0, (float) (24 * 2 - 3 - 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    static final OpenGLMatrix redLeftTargetLocationOnField = OpenGLMatrix
            .translation(0, (float) (24 * 5 - 3 - 11. / 2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    static final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation((float) 7.5 * mmPerInch, (float) -3 * mmPerInch, 8 * mmPerInch)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, 90, 135, 0));

}
