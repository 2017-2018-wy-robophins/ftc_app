package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;

public class SelfBalancingGrabber {
    public DcMotor rightRotate;
    public DcMotor leftRotate;
    public DcMotor extensionMotor;
    Servo intake;
    Servo box;
    Telemetry telemetry;
    boolean isOpen = false;
    public SelfBalancingGrabber(DcMotor rightRotate, DcMotor leftRotate, DcMotor extensionMotor, Servo intake, Servo box, Telemetry telemetry) {
        this.rightRotate = rightRotate;
        this.leftRotate = leftRotate;
        this.extensionMotor = extensionMotor;
        this.intake = intake;
        this.box = box;
        this.telemetry = telemetry;

        leftRotate.setDirection(DcMotor.Direction.FORWARD);
        rightRotate.setDirection(DcMotor.Direction.REVERSE);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(Servo.Direction.FORWARD);
        box.setDirection(Servo.Direction.REVERSE);
    }

    public static double CLOSE_POSITION = 0;
    public static double OPEN_POSITION = 1;
    public void openContainer() {
        isOpen = true;
        box.setPosition(OPEN_POSITION);
    }

    public void closeContainer() {
        isOpen = false;
        box.setPosition(CLOSE_POSITION);
    }

    public void activate_intake() {
        setIntakePower(1);
    }

    public void activate_outtake() {
        setIntakePower(0);
    }

    public void stop_intake() {
        setIntakePower(0.5f);
    }

    private void setIntakePower(float power) {
        intake.setPosition(power);
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
        telemetry.addData("Right encoder ticks", rightRotate.getCurrentPosition());
        telemetry.addData("Left encoder ticks", leftRotate.getCurrentPosition());
        telemetry.addData("Extension Power", extensionMotor.getPower());
        telemetry.addData("Extension ticks", extensionMotor.getCurrentPosition());
        telemetry.addData("Intake Power", intake.getPosition());
        telemetry.addData("Is Open", isOpen);
    }

    final int ENCODER_TICKS_BEGIN = 0;
    final int ENCODER_TICKS_END = -500;
    final int ANGLE_BEGIN_DEGREES = 135;
    final int ANGLE_END_DEGREES = 90;
    public void autoBalance() {
        if (!isOpen) {
            telemetry.addLine("Autobalancing");
            box.setPosition(getServoPosition());
        }
    }

    // return 0 to 1
    private float getServoPosition() {
        float currentEncoderTicks = (float) (rightRotate.getCurrentPosition() + leftRotate.getCurrentPosition()) / 2;
        float calculatedAngle = (((float)(ANGLE_END_DEGREES - ANGLE_BEGIN_DEGREES)) / ((float)(ENCODER_TICKS_END - ENCODER_TICKS_BEGIN))) * currentEncoderTicks + ANGLE_BEGIN_DEGREES;
        float calculatedPosition = (float)(((CLOSE_POSITION - OPEN_POSITION) / ((float)(ANGLE_BEGIN_DEGREES - ANGLE_END_DEGREES))) * (calculatedAngle - ANGLE_END_DEGREES) + OPEN_POSITION);
        return ExtendedMath.clamp((float)Math.min(OPEN_POSITION, CLOSE_POSITION), (float)Math.max(OPEN_POSITION, CLOSE_POSITION), calculatedPosition);
    }
}
