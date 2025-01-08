package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigurationSecondRobot {
    public static double adriveMultiple = 8;
    //drive default movement
    public static double driveVelocity = (50* adriveMultiple); //default is 50
    public static double driveMinAccel =  (-30* adriveMultiple); //default is -30
    public static double driveMaxAccel = (50* adriveMultiple); //default is 50
    public static double driveMinAngle = (Math.PI* adriveMultiple); //default is Math.PI
    public static double driveMaxAngle = (Math.PI* adriveMultiple); //default is Math.PI


    //vertical slide position
    public static int highBar = 700;
    public static int autoHighBar = 900;
    public static int bottom = 0;
    public static int lowBar = 0;
    public static int topBasket = 2360;
    //added 280 on 1/3/2024

    public static double verticalSlideBottomToHighBar = 3000;

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
    public static double verticalWristIntaketoBar = 500;

    //vertical grabber servo position
    public static double verticalOpen = 0.53;
    public static double verticalClose = 0.41;

    //Time to move vertical grabber, MS
    public static double verticalOpenTime = 200;
    public static double verticalCloseTime = 200;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double horizontalWristTransfer = 0.91;
    //servo position to have grabber slightly above the ground
    public static double horizontalWristHover = .4;
    //servo position to have grabber grab the butter
    public static double horizontalWristIntake = 0.175;

    //horizontal wrist moving times, MS
    public static double horizontalWristtoMiddleTime = 750;
    public static double horizontalWristDowntoUpTime = 1340;
    public static double horizontalWristIntaketoTransfer = 800;


    //horizontal grabber servo position
    public static double horizontalGrabberClose = .41;
    public static double horizontalGrabberSoftClose = horizontalGrabberClose - 0.025;
    public static double horizontalGrabberOpen = horizontalGrabberClose - 0.225;

    //todo horizontal grabber close time, MS
    public static double horizontalGrabberCloseTime = 150; //Maybe this right?

    //horizontal roll servo position
    public static double flat = 0.15;
    public static double sideway = .34;

    //horizontal roll time to flip, MS
    public static double flatToSidewaysTime = 600;

    //horizontal slide motor position
    public static int horizontalSlideRetract = 0;
    public static int horizontalSlideExtend = 550;
    public static double extendVelocity = 1800;

    public static double horizontalSlideTime = 0;
}
