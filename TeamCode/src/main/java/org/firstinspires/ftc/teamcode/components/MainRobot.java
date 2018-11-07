package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.driveBase.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.HybridTankOmni;
import org.firstinspires.ftc.teamcode.components.hook.DebugHook;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.hook.Hook;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensorBNO055;
import org.firstinspires.ftc.teamcode.components.positionFinder.PositionFinder;
import org.firstinspires.ftc.teamcode.components.positionFinder.VuforiaPositionFinder;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;


public class MainRobot {
    public DriveBase driveBase;
    public ElevatorHook hook;
    public InertialSensor imu;
    public VisionProcessor visionProcessor;
    //define all variables used
    boolean DEBUG_CLASSES;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
            driveBase = new DebugDriveBase(telemetry);
        } else {
            // create the drivebase
            DcMotor leftDrive = hardwareMap.dcMotor.get("leftDrive");
            DcMotor rightDrive = hardwareMap.dcMotor.get("rightDrive");
            driveBase = new HybridTankOmni(leftDrive, rightDrive, telemetry);

            // create the elevator
            DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
            DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
            DigitalLimitSwitch limitSwitch = new DigitalLimitSwitch(hardwareMap, "elevatorLimit");
            hook = new ElevatorHook(leftElevator, rightElevator, limitSwitch, telemetry);

            // create the grabber

            // create the imu
            imu = new InertialSensorBNO055(hardwareMap);

            // create the vision processor
            visionProcessor = new VuforiaVisionProcessor(hardwareMap);
        }
    }
}