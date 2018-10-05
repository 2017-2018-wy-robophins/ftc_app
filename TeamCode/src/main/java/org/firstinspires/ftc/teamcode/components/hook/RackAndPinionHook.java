package org.firstinspires.ftc.teamcode.components.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RackAndPinionHook implements Hook {
    DcMotor hookMotor;
    Telemetry telemetry;

    private static final int extendPosition = 18000;
    private static final int contractPosition = 0;


    public RackAndPinionHook(DcMotor hookMotor, Telemetry telemetry) {
        this.hookMotor = hookMotor;

        hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void extend() {
        hookMotor.setTargetPosition(extendPosition);
        hookMotor.setPower(1);
    }

    public void contract() {
        hookMotor.setTargetPosition(contractPosition);
        hookMotor.setPower(1);
    }
}
