package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.dist;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.xCorner;
import static org.firstinspires.ftc.teamcode.robot.subsystem.Turret.yCorner;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.core.Scheduler;
import org.firstinspires.ftc.teamcode.tasks.core.Task;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootTest;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ptGaoc;
import org.firstinspires.ftc.teamcode.tests.BlockerPositionTest;
import org.firstinspires.ftc.teamcode.tests.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

import java.util.List;
import java.util.function.Supplier;

@TeleOp
@Configurable
public class Test extends BetterOpMode {
    Robot robila;
    public static double pos,velocity;
    //MecanumDrive drive;
    public static boolean shooterEnabled = true;
    public static boolean readyForShoot = false;

    @Override
   public void initialize() {
        robila = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        Turret.turret1.setPosition(0.5);
        Turret.turret2.setPosition(0.5);
        Shooter.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gamepadEx1.getButton(BetterGamepad.Buttons.LEFT_BUMPER)
                .whenPressed(ShootTest::new, BetterGamepad.Type.MAIN);
        //Shooter.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Shooter.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void initialize_loop() {
        Shooter.setCoefs();
        Blocker.block();
    }
    @Override
    public void on_start() {
    }

    @Override
    public void active_loop() {
            if(!shooterEnabled) Shooter.velocity = 0;
            Shooter.setVelocity(Shooter.velocity);
            Shooter.servo.setPosition(Shooter.servo.getPosition()+ 0.0025 *( gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER ) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)));
           // Turret.setNeutralPosition(Turret.getPositon()+gamepad2.right_stick_x*0.025);

            if(gamepad2.crossWasPressed())
                shooterEnabled = !shooterEnabled;

            if(gamepad1.crossWasPressed()) Intake.toggle();
            if(gamepad1.circleWasPressed()) Storage.toggle();
            if(gamepad1.triangleWasPressed()) Blocker.toggle();
            if(gamepad2.rightBumperWasPressed()) Turret.reseted = !Turret.reseted;
            if(gamepad1.optionsWasPressed()) robila.resetPinPoint();


            //Storage.checkUpLaser();
            robila.update();
            Drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("velocity", Shooter.motor1.getVelocity());
            telemetry.addData("target velocity", Shooter.velocity);
            telemetry.addData("hood pos", Shooter.servo.getPosition());
            telemetry.addData("tur pos", Turret.turret1.getPosition());
            telemetry.addData("shooter enabled", shooterEnabled);
            telemetry.addData("x", Robot.robotPose.getX());
            telemetry.addData("y", Robot.robotPose.getY());
            telemetry.addData("heading", Math.toDegrees(Robot.robotPose.getHeading()));
            telemetry.addData("status", Storage.status);
            telemetry.addData("xCorner", xCorner);
            telemetry.addData("yCorner", yCorner);
            telemetry.addData("distance to goal", dist);


            Scheduler.update();
            Turret.update();
            Shooter.motor1.update();
            telemetry.update();

        }
        @Override
    public void end() {

    }
}