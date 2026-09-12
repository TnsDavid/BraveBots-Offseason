package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class PentruDavid2 extends LinearOpMode {
    public DcMotorEx FR, FL, BR, BL;

    @Override
    public void runOpMode() throws InterruptedException {
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        FL = hardwareMap.get(DcMotorEx.class, "fl");
        BR = hardwareMap.get(DcMotorEx.class, "br");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            /*
            FL=y+x+rx
            FR=y-x+rx           y->left stick y     rx->right stick x
            BL=y-x-rx           x->left stick x
            BR=y+x-rx
            */
            FR.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad2.right_stick_x);
            FL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            BR.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad2.right_stick_x);
            BL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad2.right_stick_x);
        }
    }
}
