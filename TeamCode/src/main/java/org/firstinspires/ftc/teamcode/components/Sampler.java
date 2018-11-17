package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sampler extends Component {
    Servo right;
    Servo left;

    public Sampler(Servo right, Servo left) {
        this.right = right;
        this.left = left;
        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void contractLeft() {
        left.setPosition(0);
    }

    public void contractRight() {
        right.setPosition(0);
    }

    public void extendLeft() {
        left.setPosition(1);
    }

    public void extendRight() {
        right.setPosition(1);
    }

    public void extendAll() {
        extendLeft();
        extendRight();
    }

    public void contractAll() {
        extendLeft();
        extendRight();
    }

    public void reportInfo(Telemetry telemetry) {
        telemetry.addData("right sampler", right.getPosition());
        telemetry.addData("left sampler", left.getPosition());
    }
}
