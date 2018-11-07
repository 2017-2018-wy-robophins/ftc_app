package org.firstinspires.ftc.teamcode.debug;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

@TeleOp(name = "Future Tester", group = "Debug")
public class FutureTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Start");
        telemetry.update();

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");

        CompletableFuture<Void> spinleft = CompletableFuture.runAsync(
                () -> {
                    try {
                        moveLeft(left);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Left interrupted");
                        telemetry.update();
                    }
                });

        CompletableFuture<Void> spinright = CompletableFuture.runAsync(
                () -> {
                    try {
                        moveRight(right);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Right interrupted");
                        telemetry.update();
                    }
                });

        CompletableFuture<Void> doit = CompletableFuture.allOf(spinleft, spinright);

        try {
            doit.get();
        } catch (ExecutionException e) {
            telemetry.addLine("Failed on execution");
            telemetry.update();
        }
    }

    private void moveLeft(DcMotor left) throws InterruptedException {
        while (true) {
            left.setPower(0.5);
            Thread.sleep(1000);
            left.setPower(0);
            Thread.sleep(1000);
            left.setPower(-0.5);
            Thread.sleep(1000);
        }
    }

    private void moveRight(DcMotor right) throws InterruptedException {
        while (true) {
            right.setPower(0.5);
            Thread.sleep(3000);
            right.setPower(-0.5);
            Thread.sleep(3000);
        }
    }
}
