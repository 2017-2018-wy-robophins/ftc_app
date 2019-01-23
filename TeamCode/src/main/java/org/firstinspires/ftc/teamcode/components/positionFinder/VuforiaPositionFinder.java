package org.firstinspires.ftc.teamcode.components.positionFinder;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.firstinspires.ftc.teamcode.common.StartLocation;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.backSpaceLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.blueRoverLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.frontCraterLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.phoneLocationOnRobot;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.redFootprintLocationOnField;

/**
 * Created by efyang on 12/19/17.
 */

// will be used as a component in auton: composition > inheritance for this
public class VuforiaPositionFinder implements PositionFinder {

    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables;

    //Prepares the Vuforia library for image-processing.
    public VuforiaPositionFinder(HardwareMap hwmap) {
        this.hardwareMap = hwmap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = BuildConfig.VUFORIA_API_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        blueRover.setLocation(blueRoverLocationOnField);
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        redFootprint.setLocation(redFootprintLocationOnField);
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        frontCraters.setLocation(frontCraterLocationOnField);
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        backSpace.setLocation(backSpaceLocationOnField);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        targetsRoverRuckus.activate();
    }

    //Returns the transformation matrix and the template type based on field images.
    public OpenGLMatrix getCurrentPosition() throws InterruptedException {
        // we want to wait a second before finding a vumark so that we have a good read of it
        Thread.sleep(1000);
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    return robotLocationTransform;
                }
                break;
            }
        }

        return null;
    }
}
