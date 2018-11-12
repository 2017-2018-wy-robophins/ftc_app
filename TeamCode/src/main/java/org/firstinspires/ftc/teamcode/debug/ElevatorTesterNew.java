package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.DigitalLimitSwitch;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@TeleOp(name = "Elevator Tester New", group = "Debug")
public class ElevatorTesterNew extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftElevator = hardwareMap.dcMotor.get("leftElevator");
        DcMotor rightElevator = hardwareMap.dcMotor.get("rightElevator");
        DigitalLimitSwitch limitSwitch = new DigitalLimitSwitch(hardwareMap, "elevatorSwitch");

        ElevatorHook hook = new ElevatorHook(leftElevator, rightElevator, limitSwitch, ElevatorHook.State.Contracted, telemetry);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addLine("GOTO Fully Extended");
                hook.goToState(ElevatorHook.State.FullyExtended);
            } else if (gamepad1.b) {
                telemetry.addLine("GOTO Partially Extended");
                hook.goToState(ElevatorHook.State.PartiallyExtended);
            } else if (gamepad1.x) {
                telemetry.addLine("GOTO Contracted");
                hook.goToState(ElevatorHook.State.Contracted);
            }

            hook.update();

            telemetry.addData("limit switch", limitSwitch.isPressed());
            telemetry.addData("left power", leftElevator.getPower());
            telemetry.addData("right power", rightElevator.getPower());
            telemetry.update();
        }
    }
}
