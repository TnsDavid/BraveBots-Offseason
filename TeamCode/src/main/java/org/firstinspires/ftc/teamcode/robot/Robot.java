package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

import java.util.List;

public class Robot {
    public static Pose robotPose;
    public static GoBildaPinpointDriver pinpoint;
    public DcMotorController controlHubMotors, expansionHubMotors;
    DcMotorControllerEx exExpansionHubMotors;
    public Limelight3A ll;

    public GoBildaPinpointDriver odo;

    public ServoController controlHubServos, expansionHubServos;

    public DigitalChannelController controller;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
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
    public void initializeControllers(){
        controlHubMotors= hm.get(DcMotorController.class, "Control Hub");
        expansionHubMotors= hm.get(DcMotorController.class, "Expansion Hub 2");
        controlHubServos= hm.get(ServoController.class, "Control Hub");
        expansionHubServos= hm.get(ServoController.class, "Expansion Hub 2");
        controlHubAnalogInputController= hm.get(AnalogInputController.class, "Control Hub");
        expansionHubAnalogInputController= hm.get(AnalogInputController.class, "Expansion Hub 2");
        exExpansionHubMotors= hm.get(DcMotorControllerEx.class, "Expansion Hub 2");
        controller = hm.get(DigitalChannelController.class, "Control Hub");
    }
    public void initializeMotors() {

    }
    private void assignHardware(){

    }

    public void initializeServos(){

    }

    public void initializeRest(){
        //odo= hm.get(GoBildaPinpointDriver.class, "nigg");
        if(opModeType== null)
            throw new IllegalArgumentException("OpMode type nesetat");
        if(opModeType== OpModeType.TELEOP) {
            odo = hm.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.CM);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        }
        ll= hm.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(0);
        ll.setPollRateHz(100);
        ll.start();
        //WebcamName name= hm.get(WebcamName.class, "camera");
        //camera= OpenCvCameraFactory.getInstance().createWebcam(name);

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
