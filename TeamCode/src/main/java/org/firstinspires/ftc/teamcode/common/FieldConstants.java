package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by efyang on 2/2/18.
 */

public final class FieldConstants {
    public static final float mmPerInch = 25.4f;
    public static final float mmPerBlock = mmPerInch * 24;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    public static final OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
            .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
    public static final OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, 0, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
    public static final OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
            .translation(0, mmFTCFieldWidth, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
    public static final OpenGLMatrix frontCraterLocationOnField = OpenGLMatrix
            .translation(0, -mmFTCFieldWidth, mmTargetHeight)
            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
    public static final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation(0, 0, 0)
            .multiplied(Orientation.getRotationMatrix(
                    EXTRINSIC, AxesOrder.ZYX,
                    DEGREES, -90, 90, 0));
    // we use the robot pointing to the right as the default axial system here
}
