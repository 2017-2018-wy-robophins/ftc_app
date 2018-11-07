package org.firstinspires.ftc.teamcode.components.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.DigitalLimitSwitch;

public class ElevatorHook implements Hook {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DigitalLimitSwitch limitSwitch;
    private Telemetry telemetry;
    private State state = State.Contracted;
    private State nextState = State.Between;

    private boolean startedOnSwitch; // only do this if we started on a switch
    private boolean previousPressed; // when previous switch state was pressed, now not pressed, then off initial switch
    private boolean offInitialSwitch; // now when off initial switch, check for if pressed again

    public ElevatorHook(DcMotor leftMotor, DcMotor rightMotor, DigitalLimitSwitch limitSwitch, Telemetry telemetry) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.limitSwitch = limitSwitch;
        this.telemetry = telemetry;

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: fix until working encoder cable
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setMode(DcMotor.RunMode mode) {
        rightMotor.setMode(mode);
        leftMotor.setMode(mode);
    }

    private void setPower(float power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    public void latch() {
        startedOnSwitch = previousPressed = limitSwitch.isPressed();
        offInitialSwitch = false;
        setPower(1);
        /*
        if (state == State.Contracted) {
            setPower(1);
            state = State.Between;
            nextState = State.Extended;
            System.out.println("extend");
        }*/
    }

    public void delatch() {
        startedOnSwitch = previousPressed = limitSwitch.isPressed();
        offInitialSwitch = false;
        setPower(-1);
        /* if (state == State.Extended) {
            setPower(-1);
            state = State.Between;
            nextState = State.Contracted;
            System.out.println("contract");
        }*/
    }

    public void update() {
        boolean pressed = limitSwitch.isPressed();
        telemetry.addData("previous pressed", previousPressed);
        telemetry.addData("pressed", pressed);
        if (startedOnSwitch) {
            if (previousPressed && !pressed) {
                offInitialSwitch = true;
            } else if (offInitialSwitch && !previousPressed && pressed) {
                stop();
            }
        } else {
            if (pressed) {
                stop();
            }
        }
        previousPressed = pressed;
    }

    public void stop() {
        setPower(0);
        state = nextState;
        System.out.println("Stop");
    }

    enum State {
        Extended,
        Contracted,
        Between
    }
}
