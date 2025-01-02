package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;
@Config
public class Trajectory{
    public static int YAxis = 10;
    public static int XAxis = 36;
    VerticalSlideRR verticalSlideRR;
    VerticalWristRR verticalWristRR;
    VerticalGrabberRR verticalGrabberRR;

    HorizontalSlideRR horizontalSlideRR;
    HorizontalRollRR horizontalRollRR;
    HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR;
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    public void setTrajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR, VerticalWristRR verticalWristRR,
                      VerticalGrabberRR verticalGrabberRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR) {
        this.drive = drive;
        this.pose = pose;
        this.verticalSlideRR = verticalSlideRR;
        this.verticalWristRR = verticalWristRR;
        this.verticalGrabberRR = verticalGrabberRR;
        this.horizontalSlideRR = horizontalSlideRR;
        this.horizontalRollRR = horizontalRollRR;
        this.horizontalGrabberRR = horizontalGrabberRR;
        this.horizontalWristRR = horizontalWristRR;
    }

    public void setCurrentTrajectory(TrajectoryActionBuilder startPosition){
        currentTrajectory = startPosition;
    }

    public void setStartTrajectory(){
        currentTrajectory = drive.actionBuilder(pose);
    }
    public TrajectoryActionBuilder getHangTrajectory() {
        TrajectoryActionBuilder HangTrajectory = currentTrajectory
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .strafeTo(new Vector2d(-10, 23))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .strafeTo(new Vector2d(-10, YAxis))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake));
        currentTrajectory = HangTrajectory.endTrajectory().fresh();
        return HangTrajectory;
    }
    public TrajectoryActionBuilder getButterPickUpTrajectory(){
        TrajectoryActionBuilder ButterPickUpTrajectory = currentTrajectory
                .strafeToSplineHeading(new Vector2d(XAxis, YAxis), Math.toRadians(-90));
        currentTrajectory = ButterPickUpTrajectory.endTrajectory().fresh();
        return ButterPickUpTrajectory;
    }

    public TrajectoryActionBuilder getButterPickUpAttachment(){
        TrajectoryActionBuilder ButterPickUpAttachment = currentTrajectory
                //priming the horizontal to grab the butter
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .waitSeconds(1)
                //close the horizontal grabber
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                //retract the horizontal
                .waitSeconds(.5)
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(1)
                //horizontal grabber let go and vertical grabber close
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(.25)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .waitSeconds(.25)
                //vertical wrist move to the other side
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(.5);
        return ButterPickUpAttachment;
    }
}
