package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

    private OpenGLMatrix markerTargetPositionOnField;
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    public VuforiaPositionFinder(StartLocation start, HardwareMap hwmap) {
        switch (start) {
            case BLUE_LEFT:
                this.markerTargetPositionOnField = FieldConstants.blueLeftTargetLocationOnField;
                break;
            case BLUE_RIGHT:
                this.markerTargetPositionOnField = FieldConstants.blueRightTargetLocationOnField;
                break;
            case RED_LEFT:
                this.markerTargetPositionOnField = FieldConstants.redLeftTargetLocationOnField;
                break;
            case RED_RIGHT:
                this.markerTargetPositionOnField = FieldConstants.redRightTargetLocationOnField;
                break;
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
        Thread.sleep(1500);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                if (pose != null) {
                    OpenGLMatrix robotLocationTransform = markerTargetPositionOnField
                            .multiplied(pose.inverted())
                            .multiplied(FieldConstants.phoneLocationOnRobot.inverted());
                    return Pair.create(robotLocationTransform, vuMark);
                }

            }
        return null;
    }
}
