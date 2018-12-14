package org.firstinspires.ftc.teamcode.components.grabber;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;

@Config
public class TwoDOFGrabber extends Component {
    public DcMotor rightRotate;
    public DcMotor leftRotate;
    public DcMotor extensionMotor;
    public DcMotor intakeMotor;
    public Servo containerServo;
    public static double RIGHT_POSITION = 0.2;
    public static double LEFT_POSITION = 0;
    Telemetry telemetry;

    public TwoDOFGrabber(DcMotor rightRotate, DcMotor leftRotate, DcMotor extensionMotor, DcMotor intakeMotor, Servo containerServo, Telemetry telemetry) {
        this.rightRotate = rightRotate;
        this.leftRotate = leftRotate;
        this.extensionMotor = extensionMotor;
        this.intakeMotor = intakeMotor;
        this.containerServo = containerServo;
        this.telemetry = telemetry;

        leftRotate.setDirection(DcMotor.Direction.REVERSE);
        rightRotate.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        containerServo.setDirection(Servo.Direction.FORWARD);
        openContainer();
        closeContainer();
    }

    public void openContainer() {
        containerServo.setPosition(RIGHT_POSITION);
    }

    public void closeContainer() {
        containerServo.setPosition(LEFT_POSITION);
    }

    public void activate_intake() {
        setIntakePower(1);
    }

    public void activate_outtake() {
        setIntakePower(-1);
    }

    public void stop_intake() {
        setIntakePower(0);
    }

    private void setIntakePower(float power) {
        intakeMotor.setPower(power);
    }

    public void extend(float extendPower) {
        extensionMotor.setPower(extendPower);
    }

    public void rotate(float rotatePower) {
        leftRotate.setPower(rotatePower);
        rightRotate.setPower(rotatePower);
    }

    public void reportInfo(Telemetry telemetry) {
        telemetry.addData("Right Rotate Power", rightRotate.getPower());
        telemetry.addData("Left Rotate Power", leftRotate.getPower());
        telemetry.addData("Extension Power", extensionMotor.getPower());
        telemetry.addData("Extension ticks", extensionMotor.getCurrentPosition());
        telemetry.addData("Intake Power", intakeMotor.getPower());
    }
}
