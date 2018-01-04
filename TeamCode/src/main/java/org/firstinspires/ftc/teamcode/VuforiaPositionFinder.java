package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by efyang on 12/19/17.
 */

// will be used as a component in auton: composition > inheritance for this
class VuforiaPositionFinder {
    private float mmPerInch        = 25.4f;
    private float mmFTCFieldWidth  = (24*6 - 2) * mmPerInch;
    private float mmPictographHeight = 2 * mmPerInch;

    private final OpenGLMatrix blueRightTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float)(24 * 5 + 3 + 11./2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    private final OpenGLMatrix blueLeftTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float)(24 * 2 + 3 + 11./2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, -90, 0));

    private final OpenGLMatrix redRightTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float)(24 * 2 - 3 - 11./2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    private final OpenGLMatrix redLeftTargetLocationOnField = OpenGLMatrix
            .translation(mmFTCFieldWidth, (float)(24 * 5 - 3 - 11./2) * mmPerInch, mmPictographHeight)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.XZX,
                    AngleUnit.DEGREES, 90, 90, 0));

    private final OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
            .translation((float)7.5 * mmPerInch,(float)-1.5 * mmPerInch,8 * mmPerInch)
            .multiplied(Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.YZY,
                    AngleUnit.DEGREES, -90, 45, 0));

    private OpenGLMatrix markerTargetPositionOnField;
    VuforiaPositionFinder(StartLocation start) {
        switch (start) {
            case BLUE_LEFT:
                this.markerTargetPositionOnField = blueLeftTargetLocationOnField;
            case BLUE_RIGHT:
                this.markerTargetPositionOnField = blueRightTargetLocationOnField;
            case RED_LEFT:
                this.markerTargetPositionOnField = redLeftTargetLocationOnField;
            case RED_RIGHT:
                this.markerTargetPositionOnField = redRightTargetLocationOnField;
        }
        // should we initialize all the vuforia stuff here???
    }

    // TODO: possibly change return type later on
    public OpenGLMatrix getCurrentPosition() throws InterruptedException {
        // we want to wait a second after finding a vumark so that we have a good read of it
        Thread.sleep(1000);
        // ATTENTION: NOT DONE YET
        return null;
    }
}
