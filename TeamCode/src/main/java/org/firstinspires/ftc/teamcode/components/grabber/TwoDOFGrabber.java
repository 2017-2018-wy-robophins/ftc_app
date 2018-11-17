package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Component;

public class TwoDOFGrabber extends Component {
    public Servo deploymentServo;
    public DcMotor rightMotor;
    public DcMotor leftMotor;
    public DcMotor rotationMotor;
    public DcMotor extensionMotor;
    Telemetry telemetry;

    public TwoDOFGrabber(DcMotor rightMotor, DcMotor leftMotor, DcMotor rotationMotor, DcMotor extensionMotor, Servo deploymentServo, Telemetry telemetry) {
        this.rightMotor = rightMotor;
        this.leftMotor = leftMotor;
        this.rotationMotor = rotationMotor;
        this.extensionMotor = extensionMotor;
        this.deploymentServo = deploymentServo;
        this.telemetry = telemetry;

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rotationMotor.setDirection(DcMotor.Direction.REVERSE);
        extensionMotor.setDirection(DcMotor.Direction.FORWARD);
        deploymentServo.setDirection(Servo.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void deploy() throws InterruptedException {
        int targetRotation = -400;
        rotationMotor.setTargetPosition(-400);
        while (Math.abs(rotationMotor.getCurrentPosition() - targetRotation) > 20) {
            Thread.sleep(5);
        }

        deploymentServo.setPosition(1);

        ((ServoImplEx)deploymentServo).setPwmDisable();
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
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void extend(float extendPower) {
        extensionMotor.setPower(extendPower);
    }

    public void rotate(float rotatePower) {
        rotationMotor.setPower(rotatePower);
    }

    public void reportInfo(Telemetry telemetry) {
        telemetry.addData("Right Grabber Power", rightMotor.getPower());
        telemetry.addData("Left Grabber Power", leftMotor.getPower());
        telemetry.addData("Extension Power", extensionMotor.getPower());
        telemetry.addData("Extension ticks", extensionMotor.getCurrentPosition());
        telemetry.addData("Rotation Power", rotationMotor.getPower());
        telemetry.addData("Rotation ticks", rotationMotor.getCurrentPosition());
        telemetry.addData("Servo Position", deploymentServo.getPortNumber());
    }
}
