package org.firstinspires.ftc.teamcode.components.hook;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Globals;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.DigitalLimitSwitch;

import java.util.Optional;

//Implementation of the elevator.
public class ElevatorHook extends Component {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DigitalLimitSwitch limitSwitch;
    private Telemetry telemetry;

    public State currentState;
    public State targetState;

    private int velocity = 0;
    private boolean previousPressed; // when previous switch state was pressed, now not pressed, then off initial switch

    //Prepare the elevator for motion.
    public ElevatorHook(DcMotor leftMotor, DcMotor rightMotor, DigitalLimitSwitch limitSwitch, State initialState, Telemetry telemetry) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.limitSwitch = limitSwitch;
        this.telemetry = telemetry;
        currentState = initialState;
        targetState = initialState;

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // TODO: fix until working encoder cable
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Sets the RunMode of both elevator motors.
    private void setMode(DcMotor.RunMode mode) {
        rightMotor.setMode(mode);
        leftMotor.setMode(mode);
    }

    //Sets target state based on direction of elevator, preparing limit switches for function.
    public void setPowerDirectControl(float power) {
        velocity = (int)Math.signum(power);
        if (velocity == 1) {
            targetState = State.FullyExtended;
        } else if (velocity == -1) {
            targetState = State.Contracted;
        }
        System.out.println(power);
    }

    //Sets the power of both elevator motors.
    public void setPower(float power) {
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }

    //Moves elevator to target state, blocking asynchronous access until the operation has completed.
    public void goToStateBlocking(State targetState) {
        goToState(targetState);
        while (Globals.OPMODE_ACTIVE.get() && currentState != targetState) {
            telemetry.addLine("blocking update");
            telemetry.update();
            update();
        }
    }

    //Moves elevator to target state.
    public void goToState(State targetState) {
        this.targetState = targetState;
        int power = currentState.getDirection(targetState);
        velocity = power;
        setPower((float)-power);
    }

    //Updates elevator information, outputting to Telemetry.
    public void update() {
        boolean pressed = limitSwitch.isPressed();
        telemetry.addData("current state", currentState);
        telemetry.addData("target state", targetState);
        telemetry.addData("previous pressed", previousPressed);
        telemetry.addData("pressed", pressed);
        telemetry.addData("velocity", velocity);
        telemetry.addData("r s", rightMotor.getPower());
        telemetry.addData("l s ", leftMotor.getPower());
        telemetry.update();
        if (previousPressed && !pressed) {
            if (!currentState.isBetweenState()) {
                currentState = currentState.addVelocity(velocity).orElse(currentState);
            }
        } else if (!previousPressed && pressed) {
            if (currentState.isBetweenState()) {
                currentState = currentState.addVelocity(velocity).orElse(currentState);
            }
        }

        if (currentState == targetState) {
            stop();
        } else {
            setPower(velocity);
        }

        previousPressed = pressed;
    }

    //Stops both elevator motors.
    public void stop() {
        telemetry.addLine("Stop elevator");
        telemetry.update();
        setPower(0);
        velocity = 0;
    }

    //Reports information to Telemetry.
    public void reportInfo(Telemetry telemetry) {
        telemetry.addData("R Elev Power", rightMotor.getPower());
        telemetry.addData("L Elev Power", leftMotor.getPower());
        telemetry.addData("Previous pressed", previousPressed);
        telemetry.addData("Current State", currentState);
        telemetry.addData("Target State", targetState);
        telemetry.addData("Velocity", velocity);
    }

    //Defines elevator location information in terms of enums.
    public enum State {
        Contracted(0),
        ContractedPartialBetween(1),
        PartiallyExtended(2),
        PartiallyFullyBetween(3),
        FullyExtended(4);

        private static State[] map = new State[values().length];
        static {
            for (State state: values()) {
                map[state.height] = state;
            }
        }

        private final int height;

        State(int height) {
            this.height = height;
        }

        public int getDirection(State targetState) {
            return (int)Math.signum(targetState.height - height);
        }

        public Optional<State> addVelocity(int velocity) {
            if ((height == values().length - 1 && velocity == 1) || (height == 0 && velocity == -1)) {
                return Optional.empty();
            } else {
                return Optional.of(map[height + velocity]);
            }
        }

        public boolean isBetweenState() {
            return (height % 2) == 1;
        }
    }
}
