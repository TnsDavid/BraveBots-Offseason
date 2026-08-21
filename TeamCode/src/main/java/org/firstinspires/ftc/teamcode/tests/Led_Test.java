package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;

@Disabled
public class Led_Test extends LinearOpMode {
    BetterServo led;
    @Override
    public void runOpMode() throws InterruptedException{
        led=hardwareMap.get(BetterServo.class,"led");
        waitForStart();
        while(opModeIsActive()){
            led.setPosition(led.getPosition()+gamepad1.right_stick_x*0.0025);
            telemetry.addData("led position",led.getPosition());
            telemetry.update();
        }
    }
}
