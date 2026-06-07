package org.firstinspires.ftc.teamcode.util.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tasks.command_based.core.InstantTask;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Command;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Scheduler;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Task;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class BetterGamepad {
    Gamepad gp;
    int numberOfButtons= 14;
    Button[] buttons= new Button[numberOfButtons];
    Command command;
    public void killScheduler(){
        command.removeAllTasks();
    }


    public BetterGamepad(Gamepad gp){
        this.gp= gp;
        int i=0;
        for(Buttons button: Buttons.values())
        {
            buttons[i]= new Button(button);
            i++;
        }

    }
    public enum Buttons{
        CROSS, CIRCLE, TRIANGLE, SQUARE, RIGHT_BUMPER, LEFT_BUMPER, DPAD_UP, DPAD_LEFT
        ,DPAD_DOWN, DPAD_RIGHT, LEFT_STICK, RIGHT_STICK, TOUCHPAD, OPTIONS


    }
    public enum Type{
        MAIN, PARALLEL
    }
    public class Button{
        Buttons button;
        BooleanSupplier pressed;
        Supplier<Task> s;
        boolean schedule;
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
            }
        }
        public boolean now= true;
        Type type= Type.MAIN;
        public void whenPressed(Supplier<Task> s, Type type){
            this.s= s;
            this.type= type;
            this.now= true;
        }

        public void whenPressed(Supplier<Task> s, Type type, boolean now){
            this.s= s;
            this.now    = now;
            this.type= type;
        }



        public void whenPressed(InstantTask t, Type type){
            this.s= ()->()-> {t.run(); return true;};
            this.type= type;
            this.now= true;
        }
        public void whenPressed(InstantTask t, Type type, boolean now){
            this.s= ()->()-> {t.run(); return true;};
            this.now= now;
            this.type= type;
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
            if(button.s!= null){
                if(button.wasPressed()) {
                    if(button.type.equals(Type.MAIN)) {
                        if(button.now)
                            Scheduler.now(button.s);
                        else
                            Scheduler.schedule(button.s);
                    }
                    else{
                        Scheduler.inParallel(button.s);
                    }
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
