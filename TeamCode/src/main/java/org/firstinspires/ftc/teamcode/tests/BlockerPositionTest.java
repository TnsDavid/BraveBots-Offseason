package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;

import java.util.List;
//0.673
@TeleOp
public class BlockerPositionTest extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() throws InterruptedException{
        r= new Robot(hardwareMap, telemetry, Alliance.BLUE).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        
        waitForStart();
        while(opModeIsActive()){
            Blocker.blocker.setPosition(Blocker.blocker.getPosition() + gamepad1.left_stick_x * 0.001);
            telemetry.addData("pos", Blocker.blocker.getPosition());
            telemetry.update();
        }
    }
}   
