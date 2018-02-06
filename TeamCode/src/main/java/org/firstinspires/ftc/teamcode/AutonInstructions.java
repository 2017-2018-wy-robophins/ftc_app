package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import static org.firstinspires.ftc.teamcode.FieldConstants.mmPerBlock;

/**
 * Created by efyang on 1/11/18.
 */

class AutonInstructions {
    // currently placeholder values
    private final float[][] BLUE_RIGHT_INSTRUCTIONS = new float[][] {
            // target x, y, heading
//            {4 * mmPerBlock, 5 * mmPerBlock, 90},
            {3.8f * mmPerBlock, 5 * mmPerBlock, 135},
     //       {4 * mmPerBlock, 3.5f * mmPerBlock, 90},
            {4.5f * mmPerBlock, 3.5f * mmPerBlock, 180},
            {5 * mmPerBlock, 3.5f * mmPerBlock, 180},
            // if single value then match to instruction enum
            {InstructionType.MOVE_REL_TARGET},
            {InstructionType.ARM_UP},
            {InstructionType.DROP_BLOCK},
            {InstructionType.ARM_DOWN},
            {InstructionType.BASH_BLOCK},
    };
    private final float[][] BLUE_LEFT_INSTRUCTIONS = new float[][] {
            {4 * mmPerBlock, 2 * mmPerBlock, 90},
            {4 * mmPerBlock, 1 * mmPerBlock, 90},
            {4.5f * mmPerBlock, 1 * mmPerBlock, 90},
            {InstructionType.MOVE_REL_TARGET},
            {InstructionType.ARM_UP},
            {InstructionType.DROP_BLOCK},
            {InstructionType.ARM_DOWN},
            {InstructionType.BASH_BLOCK},
    };
    private final float[][] RED_RIGHT_INSTRUCTIONS = new float[][] {
            {2 * mmPerBlock, 2 * mmPerBlock, -90},
            {2 * mmPerBlock, 1 * mmPerBlock, -90},
            {1.5f * mmPerBlock, 1 * mmPerBlock, 90},
            {InstructionType.MOVE_REL_TARGET},
            {InstructionType.ARM_UP},
            {InstructionType.DROP_BLOCK},
            {InstructionType.ARM_DOWN},
            {InstructionType.BASH_BLOCK},
    };
    private final float[][] RED_LEFT_INSTRUCTIONS = new float[][] {
//            {2 * mmPerBlock, 5 * mmPerBlock, -90},
            {2.5f * mmPerBlock, 5 * mmPerBlock, 45},
//            {2 * mmPerBlock, 3.5f * mmPerBlock, 0},
            {1 * mmPerBlock, 3.5f * mmPerBlock, 0},
            {InstructionType.MOVE_REL_TARGET},
            {InstructionType.ARM_UP},
            {InstructionType.DROP_BLOCK},
            {InstructionType.ARM_DOWN},
            {InstructionType.BASH_BLOCK},
    };

    private int ptr = 0;
    private float[][] instructions;
    AutonInstructions(StartLocation start) {
        switch (start) {
            case BLUE_RIGHT:
                this.instructions = BLUE_RIGHT_INSTRUCTIONS;
                break;
            case BLUE_LEFT:
                this.instructions = BLUE_LEFT_INSTRUCTIONS;
                break;
            case RED_RIGHT:
                this.instructions = RED_RIGHT_INSTRUCTIONS;
                break;
            case RED_LEFT:
                this.instructions = RED_LEFT_INSTRUCTIONS;
                break;
        }
    }

    boolean has_instructions() {
        return ptr < instructions.length - 1;
    }

    Pair<InstructionType, Pair<VectorF, Float>> next_instruction() {
        float[] instruction = instructions[ptr];
        ptr += 1;
        if (instruction.length == 3) {
            return Pair.create(InstructionType.Move,
                    Pair.create(new VectorF(instruction[0], instruction[1]),
                            instruction[2]));
        } else {
            InstructionType instType = null;
            switch ((int)instruction[0]) {
                case InstructionType.ARM_DOWN:
                    instType = InstructionType.ArmDown;
                    break;
                case InstructionType.ARM_UP:
                    instType = InstructionType.ArmUp;
                    break;
                case InstructionType.DROP_BLOCK:
                    instType = InstructionType.DropBlock;
                    break;
                case InstructionType.GRAB_BLOCK:
                    instType = InstructionType.GrabBlock;
                    break;
                case InstructionType.MOVE_REL_TARGET:
                    instType = InstructionType.MoveRelTarget;
                    break;
                case InstructionType.BASH_BLOCK:
                    instType = InstructionType.BashBlock;
                    break;
            }

            return Pair.create(instType, null);
        }
    }
}
