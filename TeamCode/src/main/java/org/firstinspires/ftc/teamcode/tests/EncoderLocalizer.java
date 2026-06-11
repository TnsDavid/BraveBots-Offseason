package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

@Disabled
public class EncoderLocalizer extends LinearOpMode {

    DcMotorEx frontLeft,frontRight,backLeft,backRight;
    MecanumDrive drivetrain;

    double position, lastposition;

    IMU imu;
    public double heading;

    @Override
    public void runOpMode() throws InterruptedException {
        List<DcMotorEx> motors= hardwareMap.getAll(DcMotorEx.class);
        for(DcMotorEx motor: motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //drivetrain = new MecanumDriveTest(backLeft, backRight, frontLeft , frontRight);

        waitForStart();
        while(opModeIsActive()){

        }
    }
}
