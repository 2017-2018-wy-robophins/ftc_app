package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.components.driveBase.DebugDriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.DriveBase;
import org.firstinspires.ftc.teamcode.components.driveBase.HybridTankOmni;
import org.firstinspires.ftc.teamcode.components.grabber.TwoDOFGrabber;
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
    public HybridTankOmni driveBase;
    public ElevatorHook hook;
    public TwoDOFGrabber grabber;
    public Sampler sampler;
    public InertialSensor imu;
    public VisionProcessor visionProcessor;
    boolean DEBUG_CLASSES;
    // keep hardware variables for manual control
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor leftElevator;
    public DcMotor rightElevator;

    public DcMotor rightGrabber;
    public DcMotor leftGrabber;
    public DcMotor armRotate;
    public DcMotor armExtend;
    public Servo grabDeployServo;

    public Servo rightSampler;
    public Servo leftSampler;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, ElevatorHook.State initialElevatorState, boolean DEBUG_CLASSES) {
        this.DEBUG_CLASSES = DEBUG_CLASSES;

        if (DEBUG_CLASSES) {
        } else {
            // create the drivebase
            leftDrive = hardwareMap.dcMotor.get("leftDrive");
            rightDrive = hardwareMap.dcMotor.get("rightDrive");
            driveBase = new HybridTankOmni(leftDrive, rightDrive, telemetry);

            // create the elevator
            leftElevator = hardwareMap.dcMotor.get("leftElevator");
            rightElevator = hardwareMap.dcMotor.get("rightElevator");
            DigitalLimitSwitch limitSwitch = new DigitalLimitSwitch(hardwareMap, "elevatorLimit");
            hook = new ElevatorHook(leftElevator, rightElevator, limitSwitch, initialElevatorState, telemetry);

            // create the grabber
            rightGrabber = hardwareMap.dcMotor.get("rightGrabber");
            leftGrabber = hardwareMap.dcMotor.get("leftGrabber");
            armRotate = hardwareMap.dcMotor.get("armRotate");
            armExtend = hardwareMap.dcMotor.get("armExtend");
            grabDeployServo = hardwareMap.servo.get("grabDeployServo");
            grabber = new TwoDOFGrabber(rightGrabber, leftGrabber, armRotate, armExtend, grabDeployServo, telemetry);

            imu = new InertialSensorBNO055(hardwareMap);

            // create the sampler
            rightSampler = hardwareMap.servo.get("rightSampler");
            leftSampler = hardwareMap.servo.get("leftSampler");
            sampler = new Sampler(rightSampler, leftSampler);
        }
    }
}