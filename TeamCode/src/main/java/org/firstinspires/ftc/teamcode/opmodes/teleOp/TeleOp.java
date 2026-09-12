package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Globals;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BestGamepad;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BestOpMode;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BetterTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.TaskType;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.test_commands.TestShoot;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.test_commands.TestSpit;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso.BestGamepad;

import java.util.LinkedList;

@Configurable
public abstract class TeleOp extends BestOpMode {

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
        TestShoot.createCommandGroup();
        TestSpit.createCommandGroup();
//        scheduler.schedule(BetterTask.createInfinite(() -> {
//
//        }));

        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.setMsTransmissionInterval(500);
        robot = new Robot(hardwareMap, this.telemetry, this.a)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .setPlayStyle(Robot.PlayStyle.FAR)
                .initialize();

        gamepadEx1.getButton(BestGamepad.Buttons.LEFT_BUMPER)
                .whenPressed(BetterTask.createSimpleTask(Intake::start, TaskType.PARALLEL), false);
        gamepadEx1.getButton(BestGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(TestShoot.getCommand());
        gamepadEx1.getButton(BestGamepad.Buttons.OPTIONS)
                .whenPressed(BetterTask.createSimpleTask(() -> {
                    robot.resetPinPoint();
                     Turret.offset = -1.5;
                    gamepad1.rumble(200);
                }, TaskType.SEQUENTIAL), false);
        gamepadEx1.getButton(BestGamepad.Buttons.DPAD_DOWN)
                .whenPressed(BetterTask.createSimpleTask(()->{
                    Intake.start();
                    Intake.status= Intake.IntakeStatus.FORCEDINTAKE;
                }, TaskType.PARALLEL), false);
        gamepadEx1.getButton(BestGamepad.Buttons.CIRCLE)
                        .whenPressed(TestSpit.getCommand());
        gamepadEx1.getButton(BestGamepad.Buttons.DPAD_UP)
                .whenPressed(BetterTask.createSimpleTask(Drivetrain::toggleControlMethod, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.TRIANGLE)
                .whenPressed(BetterTask.createSimpleTask(Blocker::block, TaskType.PARALLEL), true);
        gamepadEx2.getButton(BestGamepad.Buttons.OPTIONS)
                .whenPressed(BetterTask.createSimpleTask(Robot::switchPlayStyle, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.CROSS)
                .whenPressed(BetterTask.createSimpleTask(() -> Shooter.shooterEnabled = !Shooter.shooterEnabled, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(BetterTask.createSimpleTask(Turret::toggleStatus, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.LEFT_BUMPER)
                .whenPressed(BetterTask.createSimpleTask(() -> Globals.playingDefense = !Globals.playingDefense, TaskType.PARALLEL), true);
        gamepadEx2.getButton(BestGamepad.Buttons.DPAD_RIGHT)
                .whenPressed(BetterTask.createSimpleTask(() -> Turret.offset -= .5, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.DPAD_LEFT)
                .whenPressed(BetterTask.createSimpleTask(() -> Turret.offset += .5, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.DPAD_UP)
                .whenPressed(BetterTask.createSimpleTask(() -> Turret.offset += 2, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.DPAD_DOWN)
                .whenPressed(BetterTask.createSimpleTask(() -> Turret.offset -= 2, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.SQUARE)
                .whenPressed(BetterTask.createSimpleTask(()->Shooter.offset += 10, TaskType.PARALLEL), false);
        gamepadEx2.getButton(BestGamepad.Buttons.CIRCLE)
                .whenPressed(BetterTask.createSimpleTask(()->Shooter.offset -= 10, TaskType.PARALLEL), false);

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
                    Storage.update2();
                    Thread.sleep(3);
                } catch( InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        driveThread = new Thread(() -> {
            while( driveThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    Drivetrain.betterDrive(gamepadEx1.getDouble(BestGamepad.Trigger.LEFT_X) , -gamepadEx1.getDouble(BestGamepad.Trigger.LEFT_Y) , gamepadEx1.getDouble(BestGamepad.Trigger.RIGHT_X));
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

        Blocker.block();
        if( Blocker.atPosition() )
            gamepad1.rumble(200);

    }


    long now, last, dt;

    boolean rumbleIN = false, rumbleOUT = false;
    LinkedList<Double> volt, amps;
    @Override
    public void active_loop() {
        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;

        telemetry.addData("playstyle", Robot.playStyle);
//        telemetry.addData("x", Robot.robotPose.getX());
//        telemetry.addData("y", Robot.robotPose.getY());
//        telemetry.addData("heading", Robot.robotPose.getHeading());
//        telemetry.addData("dist", Turret.virtualDist);
//        telemetry.addData("turret offset", Turret.offset);
//        telemetry.addData("shooter offset", Shooter.offset);
//        telemetry.addData("cur vel", Shooter.motor2.getVelocity());
//        telemetry.addData("target vel", Shooter.velocity);
//        telemetry.addData("isRobotCentric", Drivetrain.robotCentric);
//        for(BetterTask t : scheduler.infiniteList) telemetry.addData("task", t);
        long start = System.nanoTime();

        scheduler.update();

        long elapsed = System.nanoTime() - start;

        telemetry.addData("Scheduler ms", elapsed / 1_000_000.0);

        if(Storage.status == Storage.Status.STORAGEFULL && !rumbleIN){
            gamepad1.rumble(700);
            rumbleIN = true;
            rumbleOUT = false;
        }
        if(Storage.status == Storage.Status.EMPTY && !rumbleOUT){
            gamepad1.rumble(700);
            rumbleIN = false;
            rumbleOUT = true;
        }
        if(!Blocker.blocked && !Shooter.readyForShoot){
            gamepad2.rumble(300);
        }
        Intake.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void end() {
        turretThreadRunning = shooterThreadRunning = driveThreadRunning = false;
    }
}
//ceva