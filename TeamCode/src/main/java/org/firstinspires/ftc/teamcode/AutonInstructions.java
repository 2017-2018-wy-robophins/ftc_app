package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 1/11/18.
 */

class AutonInstructions {
    // currently placeholder values
    private final float[][] BLUE_RIGHT_INSTRUCTIONS = new float[][] {
            // target x, y, heading
            {50, 70, 45},
            // if single value then match to instruction enum
            {InstructionType.DROP_BLOCK}
    };
    private final float[][] BLUE_LEFT_INSTRUCTIONS = new float[][] {
            {50, 70, 45},
    };
    private final float[][] RED_RIGHT_INSTRUCTIONS = new float[][] {
            {50, 70, 45},
    };
    private final float[][] RED_LEFT_INSTRUCTIONS = new float[][] {
            {50, 70, 45},
    };

    private int ptr = 0;
    private float[][] instructions;
    AutonInstructions(StartLocation start) {
        switch (start) {
            case BLUE_RIGHT:
                this.instructions = BLUE_RIGHT_INSTRUCTIONS;
            case BLUE_LEFT:
                this.instructions = BLUE_LEFT_INSTRUCTIONS;
            case RED_RIGHT:
                this.instructions = RED_RIGHT_INSTRUCTIONS;
            case RED_LEFT:
                this.instructions = RED_LEFT_INSTRUCTIONS;
        }
    }

    boolean has_instructions() {
        return ptr > instructions.length - 1;
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
                case InstructionType.ARM_UP:
                    instType = InstructionType.ArmUp;
                case InstructionType.DROP_BLOCK:
                    instType = InstructionType.DropBlock;
                case InstructionType.GRAB_BLOCK:
                    instType = InstructionType.GrabBlock;
            }

            return Pair.create(instType, null);
        }
    }
}
