package org.firstinspires.ftc.teamcode.PathPlanningLibrary.control;


public class PredictiveBrakingController {
    public PredictiveBrakingCoeffs coeffs;
    public PredictiveBrakingController(PredictiveBrakingCoeffs coefficients) {
        coeffs = coefficients;
    }
    public double computeOutput(double error, double velocity) {
        double directionOfMotion = Math.signum(velocity);

        return coeffs.P * (error - computeBrakingDisplacement(velocity,
                directionOfMotion));
    }
    public double computeBrakingDisplacement(double velocity, double directionOfMotion) {
        return directionOfMotion * velocity * velocity * coeffs.kQuadraticFriction
                + velocity * coeffs.kLinearBraking;
    }
}
