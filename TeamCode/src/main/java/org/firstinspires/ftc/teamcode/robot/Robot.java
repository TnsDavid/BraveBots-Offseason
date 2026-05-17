package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Robot {
    public static Pose robotPose;
    public static GoBildaPinpointDriver pinpoint;
    public HardwareMap hm;
    public Telemetry t;
    public static Alliance a= null;

    List<LynxModule> hubs;

    public enum OpModeType{
        TELEOP, AUTO
    }
    OpModeType opModeType= null;

    public Robot setOpModeType(OpModeType type){
        opModeType= type;
        return this;
    }
    public Robot(HardwareMap hm, Telemetry t, Alliance a){
        this.hm= hm;
        this.t= t;
        this.a= a;
    }
    public Robot(HardwareMap hm, Alliance a){
        this.hm= hm;

        this.a= a;
    }
    ElapsedTime timer;
    public void update(){
        if(timer== null)
            timer= new ElapsedTime();
        hubs.forEach(LynxModule::clearBulkCache);
        //hubs.forEach(LynxModule::close);



//        if(opModeType== OpModeType.TELEOP)
//            odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
//
//        robotHeading = odo.getHeading(AngleUnit.DEGREES);

        // odo.update();

        if(opModeType.equals(OpModeType.TELEOP)) {
            pinpoint.update();
            robotPose= new Pose(pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER), pinpoint.getHeading(AngleUnit.RADIANS));
        }


    }
}
