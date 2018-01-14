package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by efyang on 12/19/17.
 */

// will be used as a component in auton: composition > inheritance for this
class VuforiaPositionFinder {
    private final float mmPerInch        = 25.4f;
    private final float mmFTCFieldWidth  = (24*6 - 2) * mmPerInch;
    private final float mmPictographHeight = 2 * mmPerInch;

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
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    public VuforiaPositionFinder(StartLocation start, HardwareMap hwmap) {
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
        this.hardwareMap = hwmap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = BuildConfig.VUFORIA_API_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }

    // TODO: possibly change return type later on
    // return the transformation matrix and the template type
    public Pair<OpenGLMatrix, RelicRecoveryVuMark> getCurrentPosition() throws InterruptedException {
        // we want to wait a second before finding a vumark so that we have a good read of it
        Thread.sleep(1000);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                if (pose != null) {
                    OpenGLMatrix robotLocationTransform = markerTargetPositionOnField
                            .multiplied(pose.inverted())
                            .multiplied(phoneLocationOnRobot.inverted());
                    return Pair.create(robotLocationTransform, vuMark);
                }

            }
        return null;
    }
}
