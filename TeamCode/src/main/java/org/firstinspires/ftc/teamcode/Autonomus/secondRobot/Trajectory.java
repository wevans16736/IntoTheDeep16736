package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Trajectory{
    HardwareMap hardwareMap;
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    public Trajectory(PinpointDrive drive, Pose2d pose, HardwareMap hardwareMap){
        this.drive = drive;
        this.pose = pose;
        this.hardwareMap = hardwareMap;
    }

    //all of these class is under Configuration.secondRobot
    VerticalSlideRR verticalSlideRR = new VerticalSlideRR(hardwareMap);
    VerticalWristRR verticalWristRR = new VerticalWristRR(hardwareMap);
    VerticalGrabberRR verticalGrabberRR = new VerticalGrabberRR(hardwareMap);

    HorizontalSlideRR horizontalSlideRR = new HorizontalSlideRR(hardwareMap);
    HorizontalRollRR horizontalRollRR = new HorizontalRollRR(hardwareMap);
    HorizontalGrabberRR horizontalGrabberRR = new HorizontalGrabberRR(hardwareMap);
    HorizontalWristRR horizontalWristRR = new HorizontalWristRR(hardwareMap);

    public void setCurrentPose(TrajectoryActionBuilder currentTrajectory){
        this.currentTrajectory = currentTrajectory;
    }

    TrajectoryActionBuilder initialize = drive.actionBuilder(pose)
            .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
            .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))

            .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
            .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
            .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake));

    TrajectoryActionBuilder HangTrajectory = drive.actionBuilder(pose)
            .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
            .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
            .strafeTo(new Vector2d(-15, 30))
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
            .strafeTo(new Vector2d(-15, 15))
            .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
            .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake));

    TrajectoryActionBuilder ButterPickupTrajectory = drive.actionBuilder(pose)
            .strafeToSplineHeading(new Vector2d(36, 27.45), Math.toRadians(-90));

    TrajectoryActionBuilder ButterPickUpAttachment = drive.actionBuilder(pose)
            //priming the horizontal to grab the butter
            .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
            .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
            .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
            .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
            .waitSeconds(.5)
            //close the horizontal grabber
            .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
            //retract the horizontal
            .waitSeconds(.5)
            .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
            .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
            .waitSeconds(.5)
            //horizontal grabber let go and vertical grabber close
            .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
            .waitSeconds(.25)
            .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
            .waitSeconds(.25)
            //vertical wrist move to the other side
            .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
            .waitSeconds(.5);

    TrajectoryActionBuilder ButterPickUp = drive.actionBuilder(pose)
            .strafeTo(new Vector2d(46, 27.45));

    public Action runButterLocation(){
        return currentTrajectory.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(36, 27.45), Math.toRadians(-90))
                .build();
    }

    public Action runButterPickUp(){
        return currentTrajectory.endTrajectory().fresh()
                .waitSeconds(.5)
                .strafeTo(new Vector2d(46, 27.45))
                .waitSeconds(1)
                .build();
    }
}
