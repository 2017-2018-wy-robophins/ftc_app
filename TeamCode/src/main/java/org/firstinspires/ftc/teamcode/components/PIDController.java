package org.firstinspires.ftc.teamcode.components;

public class PIDController {
    private float P_COEFF, D_COEFF, I_COEFF;
    private float error;
    private float errorDerivative;
    private float errorIntegral;
    public PIDController(float p, float i, float d, float initial_error) {
        this.P_COEFF = p;
        this.D_COEFF = d;
        this.I_COEFF = i;
        this.error = initial_error;
    }

    public void updateError(float newError, float timeStep) {
        float dError = newError - error;
        errorDerivative = dError / timeStep;
        errorIntegral = dError * timeStep;
        error = newError;
    }

    public float getValue() {
        return error * P_COEFF + errorDerivative * D_COEFF + errorIntegral * I_COEFF;
    }

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
