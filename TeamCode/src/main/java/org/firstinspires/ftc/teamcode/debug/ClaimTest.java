package org.firstinspires.ftc.teamcode.debug;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.NavigationalState;
import org.firstinspires.ftc.teamcode.autonomous.commands.ClaimCommand;
import org.firstinspires.ftc.teamcode.common.FieldConstants;
import org.firstinspires.ftc.teamcode.common.StartLocation;
import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VisionProcessor;
import org.firstinspires.ftc.teamcode.components.visionProcessor.VuforiaVisionProcessor;

@TeleOp(name = "Claim Test", group = "Debug")
public class ClaimTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted);
        // create the vision processor
        VisionProcessor visionProcessor = new VuforiaVisionProcessor(hardwareMap);

        NavigationalState navigationalState = new NavigationalState(FieldConstants.blueLeftStartLocation, StartLocation.BLUE_LEFT);

        (new ClaimCommand()).execute(navigationalState, mainRobot.imu, visionProcessor, mainRobot, telemetry);
    }
}
