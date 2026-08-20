package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HussamDogEater;
import org.firstinspires.ftc.teamcode.RaduFaceHam;
import org.firstinspires.ftc.teamcode.TanaseECiung;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.math.LowPassFilter;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServo;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterServoEx;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;
import org.firstinspires.ftc.teamcode.util.wrappers.RGBIndicator;

import java.util.List;

@RaduFaceHam
@TanaseECiung
@HussamDogEater
@Configurable
public class Robot {
    public static Pose robotPose;

    public static  double nominalVoltage = 10.1;
    public static double odometryResolution = 34.310551;
    public static VoltageSensor voltageSensor;
    List<LynxModule> hubs;
    LynxModule controlHub, expansionHub;
    public DcMotorController controlHubMotors, expansionHubMotors;
    DcMotorControllerEx exExpansionHubMotors;
    public Limelight3A ll;
    public BetterServo hood, blocker;
    public RGBIndicator ledST,ledDR;
    BreakBeam upLaser;
    DigitalChannel downLaser, middleLaser;

    public BetterServoEx turret1, turret2;

    public static GoBildaPinpointDriver odo;
    public BetterMotorEx intake, transfer, shooter, shooter2,FrontRight,FrontLeft,BackLeft,BackRight;
    public ServoController controlHubServos, expansionHubServos;

    public DigitalChannelController controller;
    public AnalogInputController expansionHubAnalogInputController, controlHubAnalogInputController;
    public HardwareMap hm;
    public Telemetry t;
    public static Alliance alliance = null;

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

    public static OpModeType opModeType= null;
    public Robot setOpModeType(OpModeType type){
        opModeType= type;
        return this;
    }
    public Robot setPlayStyle(PlayStyle style){
        playStyle = style;
        return this;
    }
    public Robot(HardwareMap hm, Telemetry t, Alliance a){
        this.hm= hm;
        this.t= t;
        this.alliance = a;
    }
    public Robot(HardwareMap hm, Alliance a){
        this.hm= hm;
        this.alliance = a;
    }
    public double dtlength=.356;//metri
    public double dtwidth=.414;//metri
    public double classifierWidth=.1875;//metri
    public double intakeWidth=.092;//metri
   // public double dtwidth=.3v8; //metri

    //4.01
    //
    public static void switchPlayStyle(){
        playStyle = playStyle == PlayStyle.CLOSE ? PlayStyle.FAR : PlayStyle.CLOSE;
    }
    public void resetPinPoint() {
        Odo.odo.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Odo.odo.setPosX(playStyle == PlayStyle.FAR ?
                ( alliance == Alliance.RED ? (dtlength / 2 + intakeWidth) : (3.65 - (dtlength / 2 + intakeWidth)) ) :
                ( alliance == Alliance.RED ? (3.65 - classifierWidth - dtlength / 2 - intakeWidth) : (classifierWidth + dtlength / 2 + intakeWidth) ), DistanceUnit.METER);
        Odo.odo.setPosY(playStyle == PlayStyle.FAR ?
                ( dtwidth / 2 ) :
                ( 2.00 ), DistanceUnit.METER);
        Odo.odo.setHeading(playStyle == PlayStyle.FAR ?
                ( alliance == Alliance.RED ? Math.PI : 0 ):
                ( alliance == Alliance.RED ? Math.toRadians(0) : Math.toRadians(180) ), AngleUnit.RADIANS);
        robotPose = new Pose(Odo.odo.getPosX(DistanceUnit.METER), Odo.odo.getPosY(DistanceUnit.METER), Odo.odo.getHeading(AngleUnit.RADIANS));
    }
    public void initializeControllers() {
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

        intake= new BetterMotorEx(expansionHubMotors,2);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = new BetterMotorEx(controlHubMotors, 2);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter2= new BetterMotorEx(expansionHubMotors, 3).setCachingTolerance(.01)
                .setMaxVelocity(1660)
                .setPFCoefficients(Shooter.pClose, Shooter.fClose);
        shooter= new BetterMotorEx(controlHubMotors, 3).setCachingTolerance(.01)
                .setMaxVelocity(1660)
                .setPFCoefficients(Shooter.pClose, Shooter.fClose);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void assignHardware(){
        Blocker.blocker=blocker;
        Turret.turret1=turret1;
        Turret.turret2=turret2;
        Shooter.hoodServo =hood;
        Shooter.motor1=shooter;
        Shooter.motor2=shooter2;
        Storage.transfer=transfer;
        Storage.bDown=downLaser;
        Storage.bUp=upLaser;
        Storage.laserMiddle=middleLaser;
        Storage.ledST=ledST;
        Storage.ledDR=ledDR;
        Intake.motor=intake;
        Drivetrain.leftBack=BackLeft;
        Drivetrain.leftFront=FrontLeft;
        Drivetrain.rightBack=BackRight;
        Drivetrain.rightFront=FrontRight;
    }

    public void initializeServos(){
        turret1= new BetterServoEx(controlHubServos, 0);
        turret2= new BetterServoEx(expansionHubServos, 0);
        turret1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        turret2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        hood= new BetterServo(controlHubServos, 1);
        blocker = new BetterServo(expansionHubServos, 1);
        ledST=new RGBIndicator(controlHubServos,5);
        ledDR=new RGBIndicator(expansionHubServos,5);

        hood.setDirection(Servo.Direction.REVERSE);
    }


    public static double lpf = 0.85;
    public static double chassislpf = 0.8;
    public void initializeRest(){
        if(opModeType== null)
            throw new IllegalArgumentException("OpMode type nesetat");

        voltageSensor= hm.voltageSensor.iterator().next();
        downLaser = hm.get(DigitalChannel.class, "laser intake");
        downLaser.setMode(DigitalChannel.Mode.INPUT);
        upLaser = new BreakBeam(controller, 0);
        middleLaser = hm.get(DigitalChannel.class, "laser");
        middleLaser.setMode(DigitalChannel.Mode.INPUT);

        Storage.startedTimer = false;
        Odo.init(hm, opModeType, playStyle);
        Drivetrain.robotCentric = false;
        robotPose = new Pose(Odo.odo.getPosX(DistanceUnit.METER), Odo.odo.getPosY(DistanceUnit.METER), Odo.odo.getHeading(AngleUnit.RADIANS));
        Turret.ChassisLpf = new LowPassFilter(0.8, 0.0);
        Shooter.HoodLpf = new LowPassFilter(lpf, 0.0);

    }
    ElapsedTime timer;
    public void update(){
        if(timer== null)
            timer= new ElapsedTime();
        hubs.forEach(LynxModule::clearBulkCache);

        if(opModeType== OpModeType.TELEOP) {
            Odo.odo.update();
            robotPose = new Pose(Odo.odo.getPosX(DistanceUnit.METER), Odo.odo.getPosY(DistanceUnit.METER), Odo.odo.getHeading(AngleUnit.RADIANS));
        }

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