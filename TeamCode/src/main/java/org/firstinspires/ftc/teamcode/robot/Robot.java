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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Hang;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

import java.util.List;

public class Robot {
    public static Pose robotPose;

    public static double odometryResolution = 34.310551; //ticks/mm*mm/inch = ticks/inch
    public static VoltageSensor voltageSensor;
    List<LynxModule> hubs;
    LynxModule controlHub, expansionHub;
    public DcMotorController controlHubMotors, expansionHubMotors;
    DcMotorControllerEx exExpansionHubMotors;
    public Limelight3A ll;
    public BetterServo hood, blocker;
    BreakBeam downLaser, upLaser;

    public BetterServoEx turret1, turret2;
    public GoBildaPinpointDriver odo;
    public BetterMotorEx intake, transfer, shooter, shooter2, hang,FrontRight,FrontLeft,BackLeft,BackRight;
    public ServoController controlHubServos, expansionHubServos;

    public DigitalChannelController controller;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public HardwareMap hm;
    public Telemetry t;
    public static Alliance a= null;

    public Robot(HardwareMap hm, Telemetry t){
        this.hm= hm;
        this.t= t;
    }
    public enum OpModeType{
        TELEOP, AUTO
    }
    public enum PlayStyle{
        CLOSE, FAR
    }

    public static PlayStyle playStyle;

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
    public double dtlength=.356;//metri
    public double dtwidth=.38; //metri
    public static void switchPlayStyle(){
        playStyle = playStyle == PlayStyle.CLOSE ? PlayStyle.FAR : PlayStyle.CLOSE;
    }
    public void resetPinPoint() {
        odo.setPosX(a == Alliance.RED ? dtwidth / 2 : 3.65 - (dtwidth / 2), DistanceUnit.METER);
        odo.setPosY(dtlength / 2, DistanceUnit.METER);
        odo.setHeading(Math.toRadians(90), AngleUnit.RADIANS);
        robotPose = new Pose(odo.getPosX(DistanceUnit.METER), odo.getPosY(DistanceUnit.METER), odo.getHeading(AngleUnit.RADIANS));
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
        FrontLeft=new BetterMotorEx(controlHubMotors,0);
        BackLeft=new BetterMotorEx(controlHubMotors,1);
        FrontRight=new BetterMotorEx(expansionHubMotors,0);
        BackRight=new BetterMotorEx(expansionHubMotors,1);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake= new BetterMotorEx(controlHubMotors,2);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new BetterMotorEx(expansionHubMotors, 2);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter= new BetterMotorEx(controlHubMotors, 3).setCachingTolerance(.01)
                .setMaxVelocity(2100)
                .setPFCoefficients(Shooter.p, Shooter.f);
        shooter2= new BetterMotorEx(expansionHubMotors, 3).setCachingTolerance(.01)
                .setMaxVelocity(2100)
                .setPFCoefficients(Shooter.p, Shooter.f);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void assignHardware(){
        Blocker.blocker=blocker;
        Turret.turret1=turret1;
        Turret.turret2=turret2;
        Shooter.servo=hood;
        Shooter.motor1=shooter;
        Shooter.motor2=shooter2;
        Storage.transfer=transfer;
        Storage.bDown=downLaser;
        Storage.bUp=upLaser;
        Intake.motor=intake;
        Drivetrain.leftBack=BackLeft;
        Drivetrain.leftFront=FrontLeft;
        Drivetrain.rightBack=BackRight;
        Drivetrain.rightFront=FrontRight;
        Drivetrain.pinpoint = odo;
    }

    public void initializeServos(){
        turret1= new BetterServoEx(expansionHubServos, 0);
        turret2= new BetterServoEx(controlHubServos, 0);
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hood= new BetterServo(expansionHubServos, 1);
        blocker = new BetterServo(controlHubServos, 1);
    }

    public void initializeRest(){
        if(opModeType== null)
            throw new IllegalArgumentException("OpMode type nesetat");
        if(opModeType== OpModeType.TELEOP) {
            odo = hm.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(Constants.xOffset, Constants.yOffset, DistanceUnit.INCH);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setEncoderResolution(odometryResolution, DistanceUnit.MM);
            resetPinPoint();
        }
        voltageSensor= hm.voltageSensor.iterator().next();
        downLaser = new BreakBeam(controller, 3);
        upLaser = new BreakBeam(controller, 5);
        Storage.count=0;
        Storage.startedTimer = false;
        playStyle = PlayStyle.CLOSE;
        //LLMath.init(hm);
    }
    ElapsedTime timer;
    public void update(){
        if(timer== null)
            timer= new ElapsedTime();
        hubs.forEach(LynxModule::clearBulkCache);
        //hubs.forEach(LynxModule::close);


        if(opModeType== OpModeType.TELEOP){
            odo.update();
            robotPose= new Pose(odo.getPosX(DistanceUnit.METER), odo.getPosY(DistanceUnit.METER), odo.getHeading(AngleUnit.RADIANS));
        }
//        robotHeading = odo.getHeading(AngleUnit.DEGREES);

        // odo.update();

//        if(opModeType.equals(OpModeType.TELEOP)) {
//            pinpoint.update();
//            robotPose= new Pose(pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER), pinpoint.getHeading(AngleUnit.RADIANS));
//        }


    }
    public Robot initialize()
    {
        hubs= hm.getAll(LynxModule.class);
        hubs.forEach((hub)-> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        controlHub= hubs.get(0);
        expansionHub= hubs.get(1);
        initializeControllers();
        initializeMotors();
        initializeServos();
        initializeRest();
        assignHardware();
        return this;
    }
}