package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigurationSecondRobot {
    public static double adriveMultiplyer = 2;
    //drive default movement
    public static double driveVelocity = (50* adriveMultiplyer); //default is 50
    public static double driveMinAccel =  (-30* adriveMultiplyer); //default is -30
    public static double driveMaxAccel = (50* adriveMultiplyer); //default is 50
    public static double driveMinAngle = (Math.PI* adriveMultiplyer); //default is Math.PI
    public static double driveMaxAngle = (Math.PI* adriveMultiplyer); //default is Math.PI


    //vertical slide position
    public static int highBar = 750;
    public static int autoHighBar = 900;
    public static int bottom = 0;
    public static int lowBar = 0;
    public static int topBasket = 2360;
    //added 280 on 1/3/2024

    //this is vertical wrist servo position
    //this is a position to place it on the basket
    public static double verticalWristBasket = 0.43;
    //this is a position to grab the butter from the wall or set it on the lower basket or either rung
    public static double verticalWristWall = 0.275;
    //this is the position to grab the butter from the intake
    public static double verticalWristIntake = 0.935;
    //this is the position to hang the butter to the bar
    public static double verticalWristBar = .43;

    //Time to move vertical wrist, MS
    public static double verticalWristWalltoIntake = 440;


    //vertical grabber servo position
    public static double verticalOpen = 0.55;
    public static double verticalClose = 0.41;

    //Time to move vertical grabber, MS
    public static double verticalOpenTime = 350;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double horizontalWristTransfer = 0.075;
    //servo position to have grabber slightly above the ground
    //TODO: axon servo change here
    public static double horizontalWristHover = 0.60;
    //servo position to have grabber grab the butter
    public static double horizontalWristIntake = 0.99;

    //horizontal wrist moving times, MS
    public static double horizontalWristtoMiddleTime = 750;
    public static double horizontalWristDowntoUpTime = 1340;


    //horizontal grabber servo position
    public static double horizontalGrabberClose = 0.6;
    public static double horizontalGrabberSoftClose = horizontalGrabberClose - 0.025;
    public static double horizontalGrabberOpen = horizontalGrabberClose - 0.35;

    //horizontal roll servo position
    public static double flat = 0.125;
    public static double sideway = .34;


    //horizontal slide motor position
    public static int horizontalSlideRetract = 0;
    public static int horizontalSlideExtend = 550;
    public static double extendVelocity = 1800;
}