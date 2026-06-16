package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Spit;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@Configurable
public abstract class TeleOp extends BetterOpMode {

    public Alliance a;
    boolean updateTurret;
    Robot robot;
    Follower f;
    MultipleTelemetry telemetry;
    private boolean readyForRaise=false;
    volatile boolean shooterThreadRunning, turretThreadRunning, driveThreadRunning;
    Thread shooterThread, turretThread, driveThread;

    public abstract void setAlliance();

    @Override
    public void initialize() {
        setAlliance();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.setMsTransmissionInterval(300);
        robot = new Robot(hardwareMap, this.telemetry, this.a)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Shoot::new, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> robot.resetPinPoint(), BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                        .whenPressed(Intake::start,BetterGamepad.Type.PARALLEL);//intake fortat
        gamepadEx1.getButton(BetterGamepad.Buttons.TRIANGLE)
                .whenPressed(Blocker::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.CIRCLE)
                .whenPressed(Spit::new, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(Drivetrain::toggleControlMethod, BetterGamepad.Type.MAIN);

        gamepadEx2.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(() -> Shooter.shooterEnabled = !Shooter.shooterEnabled, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Turret::toggleStatus,  BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_RIGHT)
                .whenPressed(() -> Turret.offset -= .5, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_LEFT)
                .whenPressed(() -> Turret.offset += .5, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> {
                    Robot.switchPlayStyle();
                    Shooter.setCoefs();
                }, BetterGamepad.Type.MAIN);

        shooterThread = new Thread(() -> {
            while( shooterThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    Shooter.update();
                    Thread.sleep(3);
                } catch (InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        turretThread = new Thread(() -> {
            while( turretThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    Turret.update();
                    Thread.sleep(3);
                } catch( InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        driveThread = new Thread(() -> {
            while( driveThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    Drivetrain.betterDrive(gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X) , -gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y) , gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));
                    Thread.sleep(3);
                } catch ( InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void on_start() {
        turretThreadRunning = shooterThreadRunning = driveThreadRunning = true;
        updateTurret = Shooter.shooterEnabled = false;

        turretThread.start();
        shooterThread.start();
        driveThread.start();

    }

    long now, last, dt;
    double pos;
    //volatile double slow;
    @Override
    public void active_loop() {
       // slow= gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER);
        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;
       // Shooter.servo.setPosition(Range.clip(Shooter.servo.getPosition() + 0.0025 *(gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)), Shooter.minHoodpos, Shooter.maxHoodPos));

        telemetry.addData("playstyle", Robot.playStyle);
        telemetry.addData("robot centric", Drivetrain.robotCentric);
        telemetry.addLine( "------------------");
        telemetry.addData("x", Robot.robotPose.getX());
        telemetry.addData("y", Robot.robotPose.getY());
        telemetry.addData("heading", Robot.robotPose.getHeading());
        telemetry.addData("dist", Turret.dist);
        telemetry.addData("bm up" , Storage.bUp.getBeamState());
        telemetry.addData("bm down" , Storage.bDown.getBeamState());
        telemetry.addLine("-------------------");
        telemetry.addData("moving", Turret.sotm);
        telemetry.addData("velocity", Shooter.velocity);
        telemetry.addData("curr vel", Shooter.motor1.getVelocity());
        telemetry.addData("shooter enb", Shooter.shooterEnabled);
        telemetry.addData("hood pos", Shooter.servo.getPosition());
        telemetry.addData("count" , Storage.count);
        telemetry.addData("TOF", Turret.TOF);
        telemetry.addData("blocker blocked?", Blocker.blocked);

        Intake.update();
        Storage.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void end() {
        turretThreadRunning = shooterThreadRunning = driveThreadRunning = false;
    }
}