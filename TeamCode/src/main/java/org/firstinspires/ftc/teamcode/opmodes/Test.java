package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.tests.MecanumDrive;

import java.util.List;

@TeleOp
@Configurable
public class Test extends LinearOpMode {
    Robot robila;
    MecanumDrive drive;
    boolean intakestarted=false;
    double velocity= 0;
    boolean shooterEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robila= new Robot(hardwareMap, telemetry, Alliance.BLUE).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        Turret.turret1.setPosition(0.5);
        Turret.turret2.setPosition(0.5);
        Shooter.setCoefs();
        Blocker.block();

        waitForStart();
        while(opModeIsActive())
        {
            if(shooterEnabled)
                Shooter.setVelocity(1000);
            else
                Shooter.setVelocity(0);

           Shooter.servo.setPosition(Shooter.servo.getPosition()+ gamepad2.left_stick_y * 0.0025);
            Turret.turret1.setPosition(Turret.turret1.getPosition() + gamepad2.right_stick_x * 0.0025);
            Turret.turret2.setPosition(Turret.turret2.getPosition() + gamepad2.right_stick_x * 0.0025);

            if(gamepad2.crossWasPressed())
                shooterEnabled = !shooterEnabled;

            if(gamepad1.crossWasPressed()) Intake.toggle();
            if(gamepad1.circleWasPressed()) Storage.toggle();
            if(gamepad1.triangleWasPressed()) Blocker.toggle();

            //Storage.checkUpLaser();

            Drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("velocity", velocity);
            telemetry.addData("hood pos", Shooter.hoodPos);

            Shooter.updateTest();
            robila.update();
            telemetry.update();
        }
    }
}