package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Hang;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.command_based.command_types.ConditionalTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Scheduler;

import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootForta;

import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Spit;
import org.firstinspires.ftc.teamcode.tests.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

import java.util.function.DoubleSupplier;

@Configurable
public abstract class TeleOp extends BetterOpMode {

    public  double p=.8,d= 0.02;

    public Alliance a;
    volatile boolean driveThreadRunning;
    public boolean fieldCentric= true, readyForShoot = false;
    boolean updateTurret= true;
    public abstract void setAlliance();
    Robot robot;
    RevColorSensorV3 clr;

    volatile MecanumDrive drive;
    MultipleTelemetry telemetry;
    public DoubleSupplier forward, strafe, rotation;
    Thread shooterThread;
    boolean lastFull = false;
    Thread driveThread;
    volatile boolean shooterThreadRunning= false;
    private boolean readyForRaise=false;


    public abstract void setDriveSuppliers();
    @Override
    public void initialize() {
        setAlliance();
        setDriveSuppliers();
        telemetry= new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot= new Robot(hardwareMap, this.telemetry, this.a)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();

        gamepadEx2.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(()-> {updateTurret= !updateTurret;
                    Turret.reseted= true;}, BetterGamepad.Type.PARALLEL);
        drive= new MecanumDrive(robot);

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS).whenPressed(Intake::toggle, BetterGamepad.Type.PARALLEL);
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(()->{
                            double x= this.a== Alliance.BLUE ? Turret.FIELD_LENGTH - Constants.dtWidth/100 : Constants.dtWidth/100;
                            double y= Constants.dtLength/100;
                            double heading= Math.toRadians(90);
                            robot.odo.setPosX(x, DistanceUnit.METER);
                            robot.odo.setPosY(y, DistanceUnit.METER);
                            robot.odo.setHeading(heading, AngleUnit.RADIANS);
                            Turret.reseted = true;
                            Turret.offset =0;
                            Shooter.offset = 0;
                        }
                        ,
                        BetterGamepad.Type.PARALLEL);

        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_LEFT)
                .whenPressed(() -> drive = new MecanumDrive(robot), BetterGamepad.Type.MAIN);

        gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE)
                .whenPressed(Spit::new, BetterGamepad.Type.PARALLEL);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(()-> new ConditionalTask(new ShootForta(), new ShootForta(), ()-> Turret.dist> 3 ), BetterGamepad.Type.MAIN);
        shooterThread= new Thread(()->{
            while (shooterThreadRunning && !Thread.currentThread().isInterrupted()){
                try{
                    Shooter.update();
                    Thread.sleep(5);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        });
        driveThread= new Thread(()->{
            while (driveThreadRunning && !Thread.currentThread().isInterrupted()){
                try{
                    drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    Thread.sleep(3);
                }
                catch (InterruptedException e){
                    Thread.currentThread().interrupt();
                }
            }
        });
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(()-> {
                    fieldCentric = !fieldCentric;
                    drive.fieldCentric(fieldCentric);
                }, BetterGamepad.Type.MAIN);
    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void on_start() {
        Shooter.offset = 0;
        Turret.offset = 0;
        shooterThreadRunning= true;
        shooterThread.start();
        driveThreadRunning= true;
        //  driveThread.start();
    }

    long now, last, dt;
    volatile double slow;
    @Override
    public void active_loop() {
        slow= gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER);
        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;

        if(gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_UP).wasPressed())
            Shooter.offset+=25;
        if(gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_DOWN).wasPressed())
            Shooter.offset-=25;
        if(gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_LEFT).wasPressed())
            Turret.offset-=0.5;
        if(gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_RIGHT).wasPressed())
            Turret.offset+=0.5;

        robot.update();
        telemetry.update();
        drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if(updateTurret)
            Turret.update();
        else
            Turret.setNeutralPosition(.5);
    }

    @Override
    public void end() {
        shooterThreadRunning= false;
        driveThreadRunning= false;
    }
}