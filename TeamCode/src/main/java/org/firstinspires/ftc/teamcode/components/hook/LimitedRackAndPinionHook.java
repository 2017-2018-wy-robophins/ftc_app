package org.firstinspires.ftc.teamcode.components.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimitedRackAndPinionHook implements Hook {
    DcMotor hookMotor;
    // TODO: make a component for this
    DigitalChannel touchLimit;
    long lastExtensionCommandTime = System.currentTimeMillis();
    long touchLimitTimeBuffer = 1000; // ms
    Telemetry telemetry;
    State state = State.Contracted;
    State nextState = State.Between;

    private static final int extendPosition = 13000;
    private static final int contractPosition = 0;


    public LimitedRackAndPinionHook(DcMotor hookMotor, DigitalChannel touchLimit, Telemetry telemetry) {
        this.hookMotor = hookMotor;

        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: fix until working encoder cable
        hookMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.touchLimit = touchLimit;
    }

    public void extend() {
        lastExtensionCommandTime = System.currentTimeMillis();
        if (state == State.Contracted) {
            hookMotor.setPower(1);
            state = State.Between;
            nextState = State.Extended;
            System.out.println("extend");
        }
    }

    public void contract() {
        lastExtensionCommandTime = System.currentTimeMillis();
        if (state == State.Extended) {
            hookMotor.setPower(-1);
            state = State.Between;
            nextState = State.Contracted;
            System.out.println("contract");
        }
    }

    public void update() {
        boolean pressed = !touchLimit.getState();
        if (System.currentTimeMillis() - lastExtensionCommandTime > touchLimitTimeBuffer && pressed) {
            hookMotor.setPower(0);
            state = nextState;
            System.out.println("Stop");
        }
    }

    enum State {
        Extended,
        Contracted,
        Between
    }
}
