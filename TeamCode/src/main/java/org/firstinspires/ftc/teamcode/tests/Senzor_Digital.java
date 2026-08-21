package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;

import java.util.List;

@Disabled
public class Senzor_Digital extends LinearOpMode {
    DigitalChannel Laser1;
    DigitalChannel Laser2;

    Robot r;
    DigitalChannel Laser3;

    List<LynxModule> hubs;
    @Override
    public void runOpMode() throws InterruptedException{

        r = new Robot(hardwareMap , telemetry , Alliance.RED ).initialize().setOpModeType(Robot.OpModeType.TELEOP);
        Laser1=hardwareMap.get(DigitalChannel.class, "laser");
        Laser2=hardwareMap.get(DigitalChannel.class, "laser1");

        Laser1.setMode(DigitalChannel.Mode.INPUT);
        Laser2.setMode(DigitalChannel.Mode.INPUT);
        boolean detected_1;
        boolean detected_2;
        waitForStart();
        while(opModeIsActive()){
            detected_1=Laser1.getState();
            detected_2=Laser2.getState();

            if(detected_1){
                telemetry.addLine("Object 1 Detected");
                Storage.stop();
            }
            else telemetry.addLine("No Object 1 Detected");

            if(detected_2){
                telemetry.addLine("Object 2 Detected");
            }
            else telemetry.addLine("No Object 2 Detected");

            telemetry.update();



        }
    }
}