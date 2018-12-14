package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sampler extends Component {
    Servo right;
    Servo left;
    Servo center;

    public Sampler(Servo right, Servo left, Servo center) {
        this.right = right;
        this.left = left;
        this.center = center;
        right.setDirection(Servo.Direction.FORWARD);
        left.setDirection(Servo.Direction.REVERSE);
        center.setDirection(Servo.Direction.REVERSE);

        // run this so servos get the right positions at the beginning
        this.extendAll();
        this.contractAll();
    }

    public void contractLeft() {
        left.setPosition(0);
    }

    public void contractRight() {
        right.setPosition(0);
    }

    public void contractCenter() {
        center.setPosition(0);
    }

    public void extendLeft() {
        left.setPosition(1);
    }

    public void extendRight() {
        right.setPosition(1);
    }

    public void extendCenter() {
        center.setPosition(1);
    }

    public void extendAll() {
        extendLeft();
        extendRight();
        extendCenter();
    }

    public void contractAll() {
        contractLeft();
        contractRight();
        contractCenter();
    }

    public void reportInfo(Telemetry telemetry) {
        telemetry.addData("right sampler", right.getPosition());
        telemetry.addData("left sampler", left.getPosition());
        telemetry.addData("center sampler", center.getPosition());
    }
}
