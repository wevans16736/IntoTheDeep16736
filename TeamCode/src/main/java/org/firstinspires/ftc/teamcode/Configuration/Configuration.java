package org.firstinspires.ftc.teamcode.Configuration;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Configuration{
    //drive default movement
    public static double driveVelocity = 50; //default is 50
    public static double driveMinAccel =  -30; //default is -30
    public static double driveMaxAccel = 50; //default is 50
    public static double driveMinAngle = Math.PI; //default is Math.PI
    public static double driveMaxAngle = Math.PI; //default is Math.PI


    //vertical slide position
    public static int highBar = -480;
    public static int bottom = 0;
    //todo is this right?
    public static int lowBar = -55;
    public static int topBasket = -830;

    //this is vertical wrist servo position
    //this is a position to place it on the basket
    public static double verticalWristBasket = 0.45;
    //this is a position to grab the butter from the wall or set it on the lower basket or either rung
    public static double verticalWristWall = 0.24;
    //this is the position to grab the butter from the intake
    public static double verticalWristIntake = 0.745;


    //vertical grabber servo position
    public static double verticalOpen = 0.3;
    public static double verticalClose = 0.5;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double horizontalWristTransfer = 0.99;
    //servo position to have grabber slightly above the ground
    public static double horizontalWristHover = 0.60;
    //servo position to have grabber grab the butter
    public static double horizontalWristIntake = 0.2;


    //horizontal grabber servo position
    public static double horizontalGrabberClose = 0;
    public static double horizontalGrabberOpen = .6;

    //horizontal roll servo position
    public static double flat = 0.075;
    public static double sideway = .34;


    //horizontal slide motor position
    public static int horizontalSlideRetract = 0;
    public static int extend = 1000;
    public static double extendVelocity = 1800;
}