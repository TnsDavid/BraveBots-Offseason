package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Senzor_Digital extends LinearOpMode {
    DigitalChannel Laser1;
    DigitalChannel Laser2;
    DigitalChannel Laser3;
    @Override
    public void runOpMode() throws InterruptedException{
        Laser1=hardwareMap.get(DigitalChannel.class, "Laser_Jos");
        Laser2=hardwareMap.get(DigitalChannel.class, "Laser_Mediu");
        Laser3=hardwareMap.get(DigitalChannel.class, "Laser_Sus");

        Laser1.setMode(DigitalChannel.Mode.INPUT);
        Laser2.setMode(DigitalChannel.Mode.INPUT);
        Laser3.setMode(DigitalChannel.Mode.INPUT);
        boolean detected_1;
        boolean detected_2;
        boolean detected_3;
        waitForStart();
        while(opModeIsActive()){

            detected_1=Laser1.getState();
            detected_2=Laser2.getState();
            detected_3=Laser3.getState();

            if(detected_1){
                telemetry.addLine("Object 1 Detected");
            }
            else telemetry.addLine("No Object 1 Detected");

            if(detected_2){
                telemetry.addLine("Object 2 Detected");
            }
            else telemetry.addLine("No Object 2 Detected");

            if(detected_3){
                telemetry.addLine("Object 3 Detected");
            }
            else telemetry.addLine("No Object 3 Detected");

            telemetry.update();



        }
    }
}