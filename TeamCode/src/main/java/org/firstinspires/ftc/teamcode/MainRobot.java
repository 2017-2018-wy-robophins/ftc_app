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
import com.qualcomm.robotcore.hardware.DistanceSensor;


class MainRobot {
    //define all variables used


    DcMotor north;
    DcMotor west;
    DcMotor east;
    DcMotor south;
    DcMotor arm;
    Servo grab1;
    Servo grab2;
    Servo colorSensorServo;
    DistanceSensor colorDistanceSensor;
    // ColorSensor colorSensor;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    public void init(HardwareMap HM) {
        north = HM.dcMotor.get("N");
        west = HM.dcMotor.get("W");
        east = HM.dcMotor.get("E");
        south = HM.dcMotor.get("S");
        arm = HM.dcMotor.get("arm");
        grab1 =  HM.servo.get("grab1");
        grab2 =  HM.servo.get("grab2");
        colorSensorServo = HM.servo.get("colorSensorServo");
        colorDistanceSensor = HM.get(DistanceSensor.class, "colorDistanceSensor");
        north.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        west.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        east.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        south.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        north.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        west.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        east.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        south.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private static final double servoClosed = 1.0;
    private static final double servoOpen = 0;

    public void closeServo() {
        grab1.setPosition(servoClosed);
        //apparently grab2 is upside-down physically
        grab2.setPosition(servoOpen);
    }
    public void openServo() throws InterruptedException {
        grab1.setPosition(servoOpen);
        Thread.sleep(500);
        grab2.setPosition(servoClosed);
        Thread.sleep(500);
    }
}