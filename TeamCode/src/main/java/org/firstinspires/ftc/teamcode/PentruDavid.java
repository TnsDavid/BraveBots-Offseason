package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class PentruDavid extends LinearOpMode {
    public DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.crossWasPressed()){
                motor.setPower(0);
            }
            if(gamepad1.squareWasPressed()){
                motor.setPower(1);
            }
        }
    }
}
