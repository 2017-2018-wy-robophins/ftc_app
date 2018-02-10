package org.firstinspires.ftc.teamcode;

/**
 * Created by efyang on 1/25/18.
 */

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@TeleOp(name = "Debug+Vuforia Tester", group = "Robot")
public class DebugTester extends LinearOpMode {
    private StartLocation startLocation = StartLocation.BLUE_LEFT;
    private NavigationalState navinfo;
    private AutonInstructions instructions;
    private VuforiaPositionFinder vuforiaPositionFinder;
    private RelicRecoveryVuMark vumark;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();
        telemetry.addLine("start");
        telemetry.update();

        navinfo = new NavigationalState();
        telemetry.addData("Start Location", startLocation);
        instructions = new AutonInstructions(startLocation);
        telemetry.addLine("Initializing vuforia...");
        telemetry.update();
        vuforiaPositionFinder = new VuforiaPositionFinder(startLocation, hardwareMap);
        telemetry.addLine("Initialized vuforia.");
        telemetry.update();
        vumark = RelicRecoveryVuMark.CENTER;
        Pair<OpenGLMatrix, RelicRecoveryVuMark> position = vuforiaPositionFinder.getCurrentPosition();

        Pair<InstructionType, Pair<VectorF, Float>> instruction = instructions.next_instruction();
        Pair<VectorF, Float> instructionValues = instruction.second;

        while (opModeIsActive()) {
            telemetry.addLine("Running");
            position = vuforiaPositionFinder.getCurrentPosition();
            if (position != null) {
                navinfo = new NavigationalState(position.first);
                vumark = position.second;
                telemetry.addLine("Found position with vuforia: " + navinfo);
                telemetry.addData("Target", vumark);
                telemetry.addData("Target loc", instructionValues.first);
                telemetry.addData("Target angle", instructionValues.second);
                telemetry.addData("move", navinfo.get_robot_movement_vector(instructionValues.first));
                telemetry.addData("rotate", navinfo.get_robot_rotation(instructionValues.second));
                telemetry.update();
            }
        }
        telemetry.addLine("finished");
        telemetry.update();
        stop();
    }
}
