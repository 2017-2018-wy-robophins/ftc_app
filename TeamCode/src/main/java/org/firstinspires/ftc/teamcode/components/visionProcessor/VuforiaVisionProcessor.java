package org.firstinspires.ftc.teamcode.components.visionProcessor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.components.positionFinder.PositionFinder;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.backSpaceLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.blueRoverLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.frontCraterLocationOnField;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.phoneLocationOnRobot;
import static org.firstinspires.ftc.teamcode.common.FieldConstants.redFootprintLocationOnField;

/**
 * Created by efyang on 12/19/17.
 */

// will be used as a component in auton: composition > inheritance for this
public class VuforiaVisionProcessor implements VisionProcessor {

    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    private Optional<TFObjectDetector> tfod;
    List<VuforiaTrackable> allTrackables;

    public VuforiaVisionProcessor(HardwareMap hwmap) {
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

    public void initTfod() throws InterruptedException {
        CameraDevice.getInstance().setFlashTorchMode(true);
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = Optional.of(ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia));
        tfod.get().loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfod.get().activate();
        while (!tfod.isPresent()) {
            Thread.sleep(20);
        }
    }

    public void stopTfod() {
        CameraDevice.getInstance().setFlashTorchMode(false);
        tfod.map(tf -> {tf.shutdown(); return tf;});
    }

    // return the transformation matrix and the template type
    public OpenGLMatrix getCurrentPosition() {
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

    public SamplingConfiguration getSamplingConfigurationPhoneRightOnlyGold() {
         if (tfod.isPresent()) {
            TFObjectDetector tf = tfod.get();

            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getTop() + (int)recognition.getHeight()/2;
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getTop() + (int)recognition.getHeight()/2;
                        }
                    }

                    if (goldMineralX != -1) {
                        // there was a gold
                        if (silverMineral1X < goldMineralX) {
                            return SamplingConfiguration.CENTER;
                        } else {
                            return SamplingConfiguration.RIGHT;
                        }
                    } else {
                        // no gold - must be to the very left
                        return SamplingConfiguration.LEFT;
                    }
                } else {
                    int goldMineralX = -1;
                    int imageHeight = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            imageHeight = recognition.getImageHeight();
                            goldMineralX = (int) recognition.getTop() + (int)recognition.getHeight()/2;
                        }
                    }

                    if (goldMineralX != -1) {
                        // there is a gold
                        System.out.println("Gold x " + goldMineralX);
                        System.out.println("thres " + imageHeight/2);
                        if (goldMineralX < imageHeight/2) {
                            System.out.println("right");
                            return SamplingConfiguration.RIGHT;
                        } else {
                            System.out.println("Center");
                            return SamplingConfiguration.CENTER;
                        }
                    } else {
                        return SamplingConfiguration.LEFT;
                    }
                }
            } else {
                return null;
            }
        } else {
            System.out.println("TRIED TO SAMPLE WITHOUT TFOD");
            // return null;
            return null;
        }
    }

    // phone mounted on the right side of the robot, vertically
    public SamplingConfiguration getSamplingConfigurationPhoneRight() {
        if (tfod.isPresent()) {
            TFObjectDetector tf = tfod.get();

            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getRight();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getRight();
                        }
                    }

                    if (goldMineralX != -1) {
                        // there was a gold
                        if (silverMineral1X > goldMineralX) {
                            return SamplingConfiguration.CENTER;
                        } else {
                            return SamplingConfiguration.RIGHT;
                        }
                    } else {
                        // no gold - must be to the very left
                        return SamplingConfiguration.LEFT;
                    }
                } else {
                    return null;
                }
            } else {
                return null;
            }
        } else {
            System.out.println("TRIED TO SAMPLE WITHOUT TFOD");
            return null;
        }
    }

    public SamplingConfiguration getSamplingConfiguration() {
        if (tfod.isPresent()) {
            TFObjectDetector tf = tfod.get();

            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            return SamplingConfiguration.LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            return SamplingConfiguration.RIGHT;
                        } else {
                            return SamplingConfiguration.CENTER;
                        }
                    } else {
                        return null;
                    }
                } else {
                    return null;
                }
            } else {
                return null;
            }
        } else {
            System.out.println("TRIED TO SAMPLE WITHOUT TFOD");
            return null;
        }
    }
}
