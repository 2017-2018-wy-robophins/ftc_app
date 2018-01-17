package org.firstinspires.ftc.teamcode;

/**
 * Created by efyang on 1/14/18.
 */

enum InstructionType {
    Move,
    ArmDown,
    ArmUp,
    DropBlock,
    GrabBlock,
    MoveRelTarget,
    BashBlock,
    ;

    static final int MOVE = 0;
    static final int ARM_DOWN = 1;
    static final int ARM_UP = 2;
    static final int DROP_BLOCK = 3;
    static final int GRAB_BLOCK = 4;
    static final int MOVE_REL_TARGET = 5;
    static final int BASH_BLOCK = 6;
}
