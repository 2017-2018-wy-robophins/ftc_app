package org.firstinspires.ftc.teamcode.debug;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.MainRobot;
import org.firstinspires.ftc.teamcode.components.hook.ElevatorHook;

@TeleOp(name = "Elevator Auton Tester", group = "Debug")
public class ElevatorAutonTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, ElevatorHook.State.Contracted, false);
        telemetry.addLine("Init");
        telemetry.update();
        waitForStart();

        mainRobot.hook.goToState(ElevatorHook.State.FullyExtended);
        mainRobot.hook.update();
        // while (mainRobot.hook.currentState != mainRobot.hook.targetState) {
        while (opModeIsActive()) {
            mainRobot.hook.update();
        }
    }
}
