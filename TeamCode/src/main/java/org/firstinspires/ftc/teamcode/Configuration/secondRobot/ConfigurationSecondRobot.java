package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigurationSecondRobot {
    public static double adriveMultiple = 50; //drive default movement
    public static double driveVelocity = (50* adriveMultiple); //default is 50
    public static double driveMinAccel =  -30; //default is -30
    public static double driveMaxAccel = (50* adriveMultiple); //default is 50
    public static double driveMinAngle = Math.PI * adriveMultiple; //default is Math.PI
    public static double driveMaxAngle = Math.PI * adriveMultiple; //default is Math.PI


    //vertical slide position
    public static int highBar = 650;
    public static int bottom = 0;
    public static int lowBar = 2000;
    public static int topBasket = 2370;
    //added 280 on 1/3/2024

    public static double verticalSlideBottomToHighBar = 1500;

    //this is vertical wrist servo position
    //this is a position to place it on the basket
    public static double verticalWristBasket = .45;
    //this is a position to grab the butter from the wall or set it on the lower basket or either rung
    public static double verticalWristWall = 0.275;
    //this is the position to grab the butter from the intake
    public static double verticalWristIntake = 0.93;
    //this is the position to hang the butter to the bar
    public static double verticalWristBar = .43;
    public static double verticalWristUp = .7;

    //Time to move vertical wrist, MS
    public static double verticalWristWalltoIntake = 650;
    public static double verticalWristIntaketoBar = 500;

    //vertical grabber servo position
    public static double verticalClose = .47;
    public static double verticalCloseHard = .5;
    public static double verticalOpen = .2;
    public static double verticalOpenWide = .175;

    //Time to move vertical grabber, MS
    public static double verticalOpenTime = 275;
    public static double verticalCloseTime = 250;
    public static double verticalCloseWideTime = 325;

    //this is the position to hang the robot
    public static double verticalHangIn = 0.9;
    public static double verticalHangOut = 0.0;


    //horizontal wrist servo position
    //servo position to have the wrist to be at vertical grabber position
    public static double horizontalWristTransfer = 0.91;
    //servo position to have grabber slightly above the ground
    public static double horizontalWristHover = .32;
    //servo position to have grabber grab the butter
    public static double horizontalWristIntake = 0.17;

    //horizontal wrist moving times, MS
    public static double horizontalWristtoMiddleTime = 300;
    public static double horizontalWristDowntoUpTime = 1340;
    public static double horizontalWristIntaketoTransfer = 450;

    //horizontal grabber servo position
    public static double horizontalGrabberClose = .15;

    public static double horizontalGrabberOpen = .3;
    public static double horizontalGrabberWide = .37;

    //todo horizontal grabber close time, MS
    public static double horizontalGrabberCloseTime = 150; //Maybe this right?
    public static double horizontalGrabberWideTime = 150;

    //horizontal roll servo position
    public static double flat = 0.175;
    public static double slant = .3;
    public static double sideway = 0.52;

    //horizontal roll time to flip, MS
    public static double flatToSidewaysTime = 600;

    //horizontal slide motor position
    public static int horizontalSlideRetract = -5;
    public static int horizontalSlideExtend = 640;
    public static double extendVelocity = 1800;

    public static double horizontalSlideTime = 640;
}
