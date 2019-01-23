package org.firstinspires.ftc.teamcode.components;

//Calculates relevant information for implementation of PID control.
public class PIDController {
    private float P_COEFF, D_COEFF, I_COEFF;
    private float error;
    private float errorDerivative;
    private float errorIntegral;

    //Initializes variables for reference.
    public PIDController(float p, float i, float d, float initial_error) {
        this.P_COEFF = p;
        this.D_COEFF = d;
        this.I_COEFF = i;
        this.error = initial_error;
    }

    //Updates error information based on current heading.
    public void updateError(float newError, float timeStep) {
        float dError = newError - error;
        errorDerivative = dError / timeStep;
        errorIntegral = dError * timeStep;
        error = newError;
    }

    //Produces the information calculated by the generic PID controller.
    public float getValue() {
        return error * P_COEFF + errorDerivative * D_COEFF + errorIntegral * I_COEFF;
    }

    //Self-explanatory access functions.

    public float getError() {
        return error;
    }

    public float getErrorDerivative() {
        return errorDerivative;
    }

    public float getErrorIntegral() {
        return errorIntegral;
    }
}
