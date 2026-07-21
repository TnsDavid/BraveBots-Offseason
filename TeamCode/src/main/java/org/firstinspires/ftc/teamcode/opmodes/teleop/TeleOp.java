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
    MultipleTelemetry telemetry;
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
                .whenPressed(() -> {
                    robot.resetPinPoint();
                    Turret.offset=0;
                }, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                .whenPressed(()->{
                    Intake.start();
                    Intake.status= Intake.IntakeStatus.FORCEDINTAKE;
                },BetterGamepad.Type.PARALLEL);//intake fortat
        gamepadEx2.getButton(BetterGamepad.Buttons.TRIANGLE)
                .whenPressed(Blocker::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.CIRCLE)
                .whenPressed(Spit::new, BetterGamepad.Type.PARALLEL);
        gamepadEx1.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(Drivetrain::toggleControlMethod, BetterGamepad.Type.PARALLEL);

        gamepadEx2.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(() -> Shooter.shooterEnabled = !Shooter.shooterEnabled, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Turret::toggleStatus,  BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_RIGHT)
                .whenPressed(() -> Turret.offset -= .5, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_LEFT)
                .whenPressed(() -> Turret.offset += .5, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(() -> Turret.offset += 2, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                .whenPressed(() -> Turret.offset -= 2, BetterGamepad.Type.PARALLEL);

        gamepadEx2.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(Robot::switchPlayStyle, BetterGamepad.Type.MAIN);

        gamepadEx2.getButton(BetterGamepad.Buttons.SQUARE)
                .whenPressed(()->Shooter.offset += 30 , BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.CIRCLE)
                .whenPressed(()->Shooter.offset -= 30 , BetterGamepad.Type.PARALLEL);

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
        Storage.status = Storage.Status.EMPTY;
        Shooter.hoodServo.setPosition(0.75);
    }

    @Override
    public void initialize_loop() {
        Blocker.block();
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
    volatile double slow;

    boolean rumbleIN = false, rumbleOUT = false;
    @Override
    public void active_loop() {
        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;

       // Shooter.hoodServo.setPosition(Shooter.hoodServo.getPosition() + 0.0025 * gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER));
        telemetry.addData("playstyle", Robot.playStyle);
        telemetry.addData("x", Robot.robotPose.getX());
        telemetry.addData("y", Robot.robotPose.getY());
        telemetry.addData("heading", Robot.robotPose.getHeading());
        telemetry.addData("dist", Turret.virtualDist);
        telemetry.addData(" target velocity", Shooter.velocity);
        telemetry.addData("curr vel", Shooter.motor1.getVelocity());
        telemetry.addData("hood pos", Shooter.hoodServo.getPosition());
        telemetry.addData("robotCentric", Drivetrain.robotCentric);
        telemetry.addData("blocker", Blocker.blocked);
        telemetry.addData("blocker at pos", Blocker.atPosition());
        telemetry.speak("Hussam esti un cacat" ,  "ISO 639-2" , "+40");

        if(Storage.status == Storage.Status.STORAGEFULL && !rumbleIN){
            gamepad1.rumble(300);
            rumbleIN = true;
            rumbleOUT = false;
        }
        if(Storage.status == Storage.Status.EMPTY && !rumbleOUT){
            gamepad1.rumble(300);
            rumbleIN = false;
            rumbleOUT = true;
        }

        if(!Shooter.readyForShoot) Blocker.block();

        Intake.update();
        Storage.update2();
        robot.update();
        telemetry.update();
    }

    @Override
    public void end() {
        turretThreadRunning = shooterThreadRunning = driveThreadRunning = false;
    }
}