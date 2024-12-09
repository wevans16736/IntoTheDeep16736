package org.firstinspires.ftc.teamcode.Autonomus;

public class Configuration{
    //drive default movement
    public static double maxWheelVel = (50); //default is 50
    public static double minProfileAccel =  (-30); //default is -30
    public static double maxProfileAccel = (150); //default is 50
    public static double maxAngVel = Math.PI; //default is Math.PI
    public static double maxAngAccel = Math.PI; //default is Math.PI


    //vertical slide position
    public static int highBar = -480;
    public static int bottom = 0;
    //todo is this right?
    public static int lowBar = -55;
    public static int topBasket = -830;

    //this is vertical wrist servo position
    //this is a position to place it on the basket
    public static double forwardUp = 0.4;
    //this is a position to grab the butter from the wall or set it on the lower basket or either rung
    public static double forwardDown = 0.24;
    //this is the position to grab the butter from the intake
    public static double backwardPos = 0.75;


    //vertical grabber servo position
    public static double open = 0.3;
    public static double close = 0.5;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double backwardPosIn = 0.9;
    //servo position to have grabber slightly above the ground
    public static double backwardPosOut = 0.45;
    //servo position to have grabber grab the butter
    public static double forwardPosOut = 0.2;


    //horizontal grabber servo position
    public static double floorClose = 0;
    public static double floorOpen = .4;

    //horizontal roll servo position
    public static double flat = 0.075;
    public static double sideway = .38;


    //horizontal slide motor position
    public static int retractSlide = 0;
    public static int extend = 1700;
    public static double extendVelocity = 1800;
}