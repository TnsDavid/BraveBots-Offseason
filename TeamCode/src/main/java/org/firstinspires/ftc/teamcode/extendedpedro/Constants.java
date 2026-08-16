package org.firstinspires.ftc.teamcode.extendedpedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

public class Constants {
    public PIDFController forward = new PIDFController(new PIDFCoefficients(0 , 0 , 0 ,0));
    public PIDFController strafe = new PIDFController(new PIDFCoefficients(0 , 0 , 0 ,0));
    public PIDFController heading = new PIDFController(new PIDFCoefficients(1.2, 0, 0.06, 0.01));
}
