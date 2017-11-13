package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class MainRobot {
    //define all variables used


    public static DcMotor north;
    public static DcMotor west;
    public static DcMotor east;
    public static DcMotor south;
    public static DcMotor arm;
    public static Servo grab1;
    public static Servo colorSensorServo;
    public static OpticalDistanceSensor colorDistanceSensor;
    public static ColorSensor colorSensor;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public void init(HardwareMap HwMap) {
        HardwareMap HM = HwMap;
        north = HM.dcMotor.get("N");
        west = HM.dcMotor.get("W");
        east = HM.dcMotor.get("E");
        south = HM.dcMotor.get("S");
        arm = HM.dcMotor.get("arm");
        grab1 =  HM.servo.get("grab1");
        colorSensorServo = HM.servo.get("colorSensorServo");
        colorDistanceSensor = HM.opticalDistanceSensor.get("colorDistanceSensor");
        //colorSensor = HM.colorSensor.get("colorSensor");

        north.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        west.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        east.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        south.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //north.setDirection(DcMotor.Direction.REVERSE);
        //west.setDirection(DcMotor.Direction.REVERSE);
        //east.setDirection(DcMotor.Direction.REVERSE);
        //south.setDirection(DcMotor.Direction.REVERSE);
        //pulley.setDirection(DcMotor.Direction.REVERSE);

    }


}