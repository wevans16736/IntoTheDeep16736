package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigurationSecondRobot {
    public static double adriveMultiplyer = 1.5;
    //drive default movement
    public static double driveVelocity = 50* adriveMultiplyer; //default is 50
    public static double driveMinAccel =  -30* adriveMultiplyer; //default is -30
    public static double driveMaxAccel = 50* adriveMultiplyer; //default is 50
    public static double driveMinAngle = Math.PI* adriveMultiplyer; //default is Math.PI
    public static double driveMaxAngle = Math.PI* adriveMultiplyer; //default is Math.PI


    //vertical slide position
    public static int highBar = 1000;
    public static int bottom = 0;
    public static int lowBar = 0;
    public static int topBasket = 2200;

    //this is vertical wrist servo position
    //this is a position to place it on the basket
    public static double verticalWristBasket = 0.5;
    //this is a position to grab the butter from the wall or set it on the lower basket or either rung
    public static double verticalWristWall = 0.275;
    //this is the position to grab the butter from the intake
    public static double verticalWristIntake = 0.94;
    //this is the position to hang the butter to the bar
    public static double verticalWristBar = .35;


    //vertical grabber servo position
    public static double verticalOpen = 0.5;
    public static double verticalClose = 0.41;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double horizontalWristTransfer = 0.05;
    //servo position to have grabber slightly above the ground
    public static double horizontalWristHover = 0.60;
    //servo position to have grabber grab the butter
    public static double horizontalWristIntake = 0.99;


    //horizontal grabber servo position
    public static double horizontalGrabberClose = .2;
    public static double horizontalGrabberOpen = 0;

    //horizontal roll servo position
    public static double flat = 0.125;
    public static double sideway = .34;


    //horizontal slide motor position
    public static int horizontalSlideRetract = 0;
    public static int horizontalSlideExtend = 550;
    public static double extendVelocity = 1800;
}