package org.firstinspires.ftc.teamcode.util.wrappers;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.math.PDSFController;

@DeviceProperties(
        xmlTag = "betterMotorEx",
        name = "BetterMotorEx",
        description = "generic motor",
        builtIn = false
)
@MotorType(
        ticksPerRev = 1550,
        gearing = 55,
        maxRPM = 6000,
        achieveableMaxRPMFraction = 1.0,
        orientation = Rotation.CCW
)
@Configurable
public class BetterMotorEx extends DcMotorImplEx implements DcMotorEx, HardwareDevice {

    double ticksPerRevolution= 0;
    ElapsedTime time;
    long startTime;

    public BetterMotorEx(DcMotorController controller, int portNumber, int ticksPerRevolution) {
        super(controller, portNumber);
        this.ticksPerRevolution= ticksPerRevolution;
        startTime= System.nanoTime();
        time= new ElapsedTime();


    }
    public BetterMotorEx(DcMotorController controller, int portNumber) {
        super(controller, portNumber);

        time= new ElapsedTime();

    }


    double cachingTolerance= 0;
    public BetterMotorEx setCachingTolerance(double tolerance){
        this.cachingTolerance= tolerance;
        return this;
    }
    public BetterMotorEx(DcMotorControllerEx controllerEx, int portNumber){
        super(controllerEx, portNumber);
        time= new ElapsedTime();
    }


    public double maxVelocity= 0;
    public BetterMotorEx setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }
    PDSFController controller;
    public double f , s;
    public double targetVelocity= 0;
    public BetterMotorEx setPFCoefficients(double p, double f){
        controller= new PDSFController(p, 0, 0, 0);
        this.f= f;
        return this;
    }

    public  BetterMotorEx setPSFCoefficients( double p , double f , double s){
        controller= new PDSFController(p, 0, 0, 0);
        this.f= f;
        this.s = s;
        return this;
    }
    public void setVelocity(double targetVelocity){
        runMode= RunMode.VELOCITY_PID;
        this.targetVelocity= targetVelocity;

    }
    public enum RunMode{
        RUN, PID, VELOCITY_PID
    }
    RunMode runMode= RunMode.RUN;
    double velocity= 0;
    private double targetPos= 0;
    @Override
    public void setPower(double power){
        if(Math.abs(power-lastPower)>cachingTolerance) {
            super.setPower(power);
            lastPower = power;
        }
    }

    private double lastPower= 0;
    public double output;
    public static double nominalvoltage = 11;
    //double currentvoltage= Robot.voltageSensor.getVoltage();
//    public void update(double targetVelocity){
//            this.targetVelocity=targetVelocity;
//            double velocity= super.getVelocity();
//            double velocityCompensationFactor = currentvoltage < nominalvoltage ? nominalvoltage / currentvoltage : 1;
       //     double pid=controller.calculate(velocity,targetVelocity)+s+f * targetVelocity / maxVelocity;
//            output=pid*velocityCompensationFactor;
//            setPower(output);
//    }


    public void resetEncoder(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
