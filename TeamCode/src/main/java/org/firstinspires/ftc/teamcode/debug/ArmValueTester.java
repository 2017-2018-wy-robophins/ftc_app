package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.arm.Arm;
import org.firstinspires.ftc.teamcode.components.arm.ThreeDOFArm;

@TeleOp(name = "Arm Value Tester", group = "Debug")
public class ArmValueTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);

        DcMotorEx arm1 = (DcMotorEx)hardwareMap.dcMotor.get("arm1");
        DcMotorEx arm2 = (DcMotorEx)hardwareMap.dcMotor.get("arm2");
        DcMotorEx arm3 = (DcMotorEx)hardwareMap.dcMotor.get("arm3");
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        while (opModeIsActive()) {
            double arm1Power = (gamepad1.right_bumper ? 0.1:0) + ((!gamepad1.right_bumper && gamepad1.left_bumper) ? -0.1:0);
            double arm2Power = (gamepad1.a ? 0.1:0) + ((!gamepad1.a && gamepad1.b) ? -0.1: 0);
            double arm3Power = (gamepad1.x ? 0.1:0) + ((!gamepad1.x && gamepad1.y) ? -0.1: 0);

            telemetry.addData("L Bumper", gamepad1.left_bumper);
            telemetry.addData("R Bumper", gamepad1.right_bumper);
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);

            telemetry.addData("1 Power", arm1Power);
            telemetry.addData("2 Power", arm2Power);
            telemetry.addData("3 Power", arm3Power);

            arm1.setPower(arm1Power);
            arm2.setPower(arm2Power);
            arm3.setPower(arm3Power);

            telemetry.addData("M1", arm1.getCurrentPosition());
            telemetry.addData("M2", arm2.getCurrentPosition());
            telemetry.addData("M3", arm3.getCurrentPosition());

            telemetry.addLine("Running");
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
