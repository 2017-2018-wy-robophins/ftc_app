package org.firstinspires.ftc.teamcode.components.arm;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public interface Arm {
    boolean goToTarget(VectorF target);
    VectorF getOrientation();
}
