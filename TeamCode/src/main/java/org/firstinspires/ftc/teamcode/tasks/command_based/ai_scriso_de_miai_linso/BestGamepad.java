package org.firstinspires.ftc.teamcode.tasks.command_based.ai_scriso_de_miai_linso;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.LinkedList;
import java.util.function.BooleanSupplier;

public class BestGamepad {
    Gamepad gp;
    BetterScheduler scheduler;
    int numberOfButtons= 15;
    Button[] buttons= new Button[numberOfButtons];

    public BestGamepad(Gamepad gp, BetterScheduler scheduler){
        this.gp= gp;
        this.scheduler = scheduler;
        int i=0;
        for(Buttons button: Buttons.values())
        {
            buttons[i]= new Button(button);
            i++;
        }

    }

    public enum Buttons{
        CROSS, CIRCLE, TRIANGLE, SQUARE, RIGHT_BUMPER, LEFT_BUMPER, DPAD_UP, DPAD_LEFT
        ,DPAD_DOWN, DPAD_RIGHT, LEFT_STICK, RIGHT_STICK, TOUCHPAD, OPTIONS, SHARE
    }

    public class Button{
        Buttons button;
        BooleanSupplier pressed;
        BetterTask task;
        LinkedList<BetterTask> taskList;
        public boolean now = true;
        public Button(Buttons button){
            this.button= button;
            switch (button){
                case CROSS:
                    pressed= ()-> gp.crossWasPressed();
                    break;
                case CIRCLE:
                    pressed= ()-> gp.circleWasPressed();
                    break;
                case SQUARE:
                    pressed= ()-> gp.squareWasPressed();
                    break;
                case TRIANGLE:
                    pressed= ()-> gp.triangleWasPressed();
                    break;
                case TOUCHPAD:
                    pressed= ()-> gp.touchpadWasPressed();
                    break;
                case DPAD_UP:
                    pressed= ()-> gp.dpadUpWasPressed();
                    break;
                case DPAD_DOWN:
                    pressed= ()-> gp.dpadDownWasPressed();
                    break;
                case DPAD_LEFT:
                    pressed= ()-> gp.dpadLeftWasPressed();
                    break;
                case DPAD_RIGHT:
                    pressed= ()-> gp.dpadRightWasPressed();
                    break;
                case LEFT_STICK:
                    pressed= ()-> gp.leftStickButtonWasPressed();
                    break;
                case RIGHT_STICK:
                    pressed= ()-> gp.rightStickButtonWasPressed();
                    break;
                case LEFT_BUMPER:
                    pressed= ()-> gp.leftBumperWasPressed();
                    break;
                case RIGHT_BUMPER:
                    pressed= ()-> gp.rightBumperWasPressed();
                    break;
                case OPTIONS:
                    pressed = ()->gp.optionsWasPressed();
                    break;
                case SHARE:
                    pressed = () -> gp.shareWasPressed();
                    break;
            }
        }
        public void whenPressed(BetterTask task, boolean now) {
            this.task = task;
            this.now = now;
        }
        public void whenPressed(LinkedList<BetterTask> tasks){
            taskList = new LinkedList<>();
            for(BetterTask task : tasks)
                taskList.addLast(task);
            this.now = false;
        }
        public boolean wasPressed(){
            return pressed.getAsBoolean();
        }
        public Buttons getButton(){
            return this.button;
        }

    }

    public Button getButton(Buttons button){
        for(int i=0; i< numberOfButtons; i++)
            if(buttons[i].getButton().equals(button))
                return buttons[i];
        throw new IllegalArgumentException("could not find button");

    }

    public void update(){
        for(Button button: buttons){
            if(button.wasPressed()) {
                if (button.task != null) {
                    if (button.now) scheduler.now(button.task);
                    else scheduler.schedule(button.task);
                }
                if (button.taskList != null) {
                    for(BetterTask t : button.taskList)
                        scheduler.schedule(t);
                }
            }
        }

    }
    public enum Trigger{
        LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y, LEFT_TRIGGER, RIGHT_TRIGGER
    }
    public double getDouble(Trigger trigger){
        switch (trigger){
            case LEFT_X:
                return gp.left_stick_x;
            case LEFT_Y:
                return gp.left_stick_y;
            case RIGHT_X:
                return gp.right_stick_x;
            case RIGHT_Y:
                return gp.right_stick_y;
            case LEFT_TRIGGER:
                return gp.left_trigger;
            case RIGHT_TRIGGER:
                return gp.right_trigger;
            default:
                throw new IllegalArgumentException("could not find button");

        }
    }
}
