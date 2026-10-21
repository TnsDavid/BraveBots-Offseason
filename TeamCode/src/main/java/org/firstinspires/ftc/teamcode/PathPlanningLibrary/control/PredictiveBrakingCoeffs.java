package org.firstinspires.ftc.teamcode.PathPlanningLibrary.control;

public class PredictiveBrakingCoeffs {
    public double kLinearBraking;
    public double kQuadraticFriction;
    public double P;
    public double maximumBrakingPower = 0.2;

    public PredictiveBrakingCoeffs( double P , double kLinearBraking , double KQuadraticFriction ,double maximumBrakingPower){
        this.P = P;
        this.kLinearBraking = kLinearBraking;
        this.kQuadraticFriction = KQuadraticFriction;
        this.maximumBrakingPower = maximumBrakingPower;
    }

    public PredictiveBrakingCoeffs withMaximumBrakingPower( double maximumBrakingPower){
        this.maximumBrakingPower = maximumBrakingPower;
        return this;
    }

}
