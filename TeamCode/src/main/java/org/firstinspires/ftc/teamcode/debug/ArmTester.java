package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.components.arm.Arm;
import org.firstinspires.ftc.teamcode.components.arm.ThreeDOFArm;

@TeleOp(name = "Arm Tester", group = "Debug")
public class ArmTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        final double ARM_JOYSTICK_MOVEMENT_THRESHOLD = 0.15;
        gamepad1.setJoystickDeadzone((float)ARM_JOYSTICK_MOVEMENT_THRESHOLD);

        DcMotorEx arm1 = (DcMotorEx)hardwareMap.dcMotor.get("arm1");
        DcMotorEx arm2 = (DcMotorEx)hardwareMap.dcMotor.get("arm2");
        DcMotorEx arm3 = (DcMotorEx)hardwareMap.dcMotor.get("arm3");

        Arm arm = new ThreeDOFArm(arm1, arm2, arm3, (float)Math.toRadians(0), (float)Math.toRadians(150), (float)Math.toRadians(-180), telemetry);
        VectorF target = new VectorF(0, 500, 20);
        float ADJUSTMENT_MULTIPLIERS = 5;

        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        while (opModeIsActive()) {
            double leftx = gamepad1.left_stick_x;
            double lefty = -gamepad1.left_stick_y;
            double righty = -gamepad1.right_stick_y;
            telemetry.addData("Left x", leftx);
            telemetry.addData("Left y", lefty);
            telemetry.addData("Right y", righty);

            VectorF change = new VectorF((float)leftx, (float)lefty, (float)righty).multiplied(ADJUSTMENT_MULTIPLIERS);
            VectorF newTarget = target.added(change);
            if (arm.goToTarget(newTarget)) {
                target = newTarget;
            }

            telemetry.addData("Current orientation", ExtendedMath.radians_to_degrees(arm.getOrientation()));
            telemetry.addData("Current position", ((ThreeDOFArm) arm).getCartesianPosition());
            telemetry.addData("Target", target);
            telemetry.addLine("Running");
            telemetry.update();
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
