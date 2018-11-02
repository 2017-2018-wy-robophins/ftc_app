package org.firstinspires.ftc.teamcode.components.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.DigitalTouchSensor;

public class LimitedRackAndPinionHook implements Hook {
    private DcMotor hookMotor;
    // TODO: make a component for this
    private DigitalTouchSensor touchLimit;
    private Telemetry telemetry;
    private State state = State.Contracted;
    private State nextState = State.Between;

    private boolean startedOnSwitch; // only do this if we started on a switch
    private boolean previousPressed; // when previous switch state was pressed, now not pressed, then off initial switch
    private boolean offInitialSwitch; // now when off initial switch, check for if pressed again


    private static final int extendPosition = 13000;
    private static final int contractPosition = 0;


    public LimitedRackAndPinionHook(DcMotor hookMotor, DigitalTouchSensor touchLimit, Telemetry telemetry) {
        this.hookMotor = hookMotor;

        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: fix until working encoder cable
        hookMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.touchLimit = touchLimit;
    }

    public void extend() {
        startedOnSwitch = previousPressed = touchLimit.isPressed();
        offInitialSwitch = false;
        if (state == State.Contracted) {
            hookMotor.setPower(1);
            state = State.Between;
            nextState = State.Extended;
            System.out.println("extend");
        }
    }

    public void contract() {
        startedOnSwitch = previousPressed = touchLimit.isPressed();
        offInitialSwitch = false;
        if (state == State.Extended) {
            hookMotor.setPower(-1);
            state = State.Between;
            nextState = State.Contracted;
            System.out.println("contract");
        }
    }

    public void update() {
        boolean pressed = touchLimit.isPressed();
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
    }

    public void stop() {
        hookMotor.setPower(0);
        state = nextState;
        System.out.println("Stop");
    }

    enum State {
        Extended,
        Contracted,
        Between
    }
}
