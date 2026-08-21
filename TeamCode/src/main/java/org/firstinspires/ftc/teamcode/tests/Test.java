package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.virtualDist;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.xCorner;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.yCorner;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@TeleOp(name = "Test", group = "Teste")
@Configurable
public class Test extends BetterOpMode {
    Robot robila;
    public static double pos,velocity;
    //MecanumDrive drive;
    public static boolean shooterEnabled = true;
    public static boolean readyForShoot = false;
    long last, now, dt;

    @Override
    public void initialize() {
        robila = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        Turret.turret1.setPosition(0.5);
        Turret.turret2.setPosition(0.5);
        Shooter.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Shoot::new, BetterGamepad.Type.MAIN);

       // gamepadEx1.getButton(BetterGamepad.Buttons.LEFT_BUMPER)
         //       .whenPressed(ShootFarTest::new, BetterGamepad.Type.MAIN);
        Storage.startedTimer = false;
    }
    @Override
    public void initialize_loop() {
        Blocker.block();
    }
    @Override
    public void on_start() {
        last = 0;
    }
    boolean eVasileProst = false;
    @Override
    public void active_loop() {

        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;
        //Turret.setNeutralPosition();
        Shooter.hoodServo.setPosition(Range.clip(Shooter.hoodServo.getPosition() + 0.0025 *(gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)), Shooter.minHoodpos, Shooter.maxHoodPos));
        if(gamepad2.crossWasPressed()) shooterEnabled = !shooterEnabled;
        if(gamepad1.crossWasPressed()) Intake.toggle();
        if(gamepad1.triangleWasPressed()) Blocker.toggle();
        if(gamepad2.rightBumperWasPressed()) Turret.toggleStatus();
        if(gamepad1.optionsWasPressed()) robila.resetPinPoint();
        if(gamepad2.touchpadWasPressed()) Drivetrain.robotCentric = !Drivetrain.robotCentric;
        if(gamepad1.shareWasPressed()) Robot.switchPlayStyle();
        if(gamepad2.dpadUpWasPressed() )
            eVasileProst = !eVasileProst;
        if( !eVasileProst )
            Drivetrain.betterDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        else
            Drivetrain.betterDrive(0, -1, 0);
        if( shooterEnabled ) {
            Shooter.AutoUpdate(Shooter.velocity , 0);
        } else
            Shooter.setPower(0);
        telemetry.addData("velocity", Shooter.motor1.getVelocity());
        telemetry.addData("target velocity", Shooter.velocity);
        telemetry.addData("hood pos", Shooter.hoodServo.getPosition());
        telemetry.addData("shooter enabled", shooterEnabled);
        telemetry.addData("x", Robot.robotPose.getX());
        telemetry.addData("y", Robot.robotPose.getY());
        telemetry.addData("heading", Math.toDegrees(Robot.robotPose.getHeading()));
        telemetry.addData("status", Storage.status);
        telemetry.addData("xCorner", xCorner);
        telemetry.addData("yCorner", yCorner);
        telemetry.addData("distance to goal", virtualDist);
        telemetry.addData("play style", Robot.playStyle);
        telemetry.addData("front left amps", Drivetrain.leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("front right amps", Drivetrain.rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back left amps", Drivetrain.leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("back right amps", Drivetrain.rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("e vasile prost?", eVasileProst);


        robila.update();
        Storage.update2();
        Intake.update();
        Turret.update();
        telemetry.update();

    }
    @Override
    public void end() {
    }
}