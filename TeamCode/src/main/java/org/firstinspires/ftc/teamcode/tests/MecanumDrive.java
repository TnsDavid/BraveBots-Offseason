package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.globals.Robot;


@Disabled
public class MecanumDrive {

    public DcMotorEx FrontRight;
    public DcMotorEx FrontLeft;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;
    double tpr = 1550;
    double diameter = 5;

    public MecanumDrive(Robot robot) {
        BackLeft = robot.BackLeft;
        BackRight = robot.BackRight;
        FrontRight = robot.FrontRight;
        FrontLeft = robot.FrontLeft;
    }
    boolean robotCentric;
    public MecanumDrive fieldCentric(boolean robotCentric){
        this.robotCentric= robotCentric;
        return this;
    }
    public  void drive(double x, double y, double rx){
        x*= 1.1;

        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
        double fl = (y + x + rx) / d;
        double bl = (y - x + rx) / d;
        double fr = (y - x - rx) / d;
        double br = (y + x - rx) / d;

        FrontLeft.setPower(fl);
        FrontLeft.setPower(fl);
        BackLeft.setPower(bl);
        BackRight.setPower(br);
        FrontRight.setPower(fr);
    }

    public double GetMotorPosition(DcMotorEx motor){
        double CPR = 1550;

        double diameter = 3.78;  // inch
        double circumference = Math.PI * diameter;

        double position = motor.getCurrentPosition();
        double revolutions = position/CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        return distance;
    }

    public void resetMotorEncoder(DcMotorEx motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}