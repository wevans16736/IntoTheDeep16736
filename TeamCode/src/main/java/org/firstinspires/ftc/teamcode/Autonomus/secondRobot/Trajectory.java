package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

@Config
public class Trajectory{
    VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(35),
            new AngularVelConstraint(Math.PI*.9)
    ));

    VelConstraint speed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(120),
            new AngularVelConstraint(Math.PI)
    ));
    public static double YAxis = 16.25;
    public static double XAxis = 37;
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

    public TrajectoryActionBuilder setSplineTestTrajectory(){
        return drive.actionBuilder(pose)
                .splineTo(new Vector2d(30, 30), Math.PI / 2)
                .splineTo(new Vector2d(0, 60), Math.PI)
                ;
    }

    public void setCurrentTrajectory(TrajectoryActionBuilder startPosition){
        currentTrajectory = startPosition;
    }

    public void setStartTrajectory(){
        currentTrajectory = drive.actionBuilder(pose);
    }
    public TrajectoryActionBuilder getHangTrajectory(boolean side) {
        double xAxis = -12;
        if(side){
            xAxis = xAxis*-1;
        }
        TrajectoryActionBuilder HangTrajectory = currentTrajectory
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .strafeToSplineHeading(new Vector2d(xAxis, 27.5),Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .strafeTo(new Vector2d(xAxis, YAxis))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom+5))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake));
        currentTrajectory = HangTrajectory.endTrajectory().fresh();
        return HangTrajectory;
    }
    public TrajectoryActionBuilder getHangTrajectory2() {
        TrajectoryActionBuilder HangTrajectory = currentTrajectory
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar-20))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .strafeToSplineHeading(new Vector2d(-9, 24),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-9, 29.5),Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .strafeTo(new Vector2d(-10, YAxis))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom+5))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake));
        currentTrajectory = HangTrajectory.endTrajectory().fresh();
        return HangTrajectory;
    }
    public TrajectoryActionBuilder getHangTrajectory3() {
        TrajectoryActionBuilder HangTrajectory = currentTrajectory
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar-20))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .strafeToSplineHeading(new Vector2d(-6, 24),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-6, 29.5),Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .strafeTo(new Vector2d(-10, YAxis))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom+5))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake));
        currentTrajectory = HangTrajectory.endTrajectory().fresh();
        return HangTrajectory;
    }
    public TrajectoryActionBuilder getButterPickUpTrajectory(){
        TrajectoryActionBuilder ButterPickUpTrajectory = currentTrajectory
                .strafeToSplineHeading(new Vector2d(XAxis, YAxis), Math.toRadians(-90), baseVel);
        currentTrajectory = ButterPickUpTrajectory.endTrajectory().fresh();
        return ButterPickUpTrajectory;
    }

    public TrajectoryActionBuilder getSecondButterPickUpTrajectory(){
        TrajectoryActionBuilder SecondButterPickUpTrajectory = currentTrajectory
                .waitSeconds(1)
                .strafeTo(new Vector2d(49, YAxis));
        currentTrajectory = SecondButterPickUpTrajectory.endTrajectory().fresh();
        return SecondButterPickUpTrajectory;
    }

    public TrajectoryActionBuilder getButterPickUpAttachment(){
        return currentTrajectory
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
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberSoftClose))
                .waitSeconds(1.5)
                //horizontal grabber let go and vertical grabber close
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(.25)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .waitSeconds(.25)
                //vertical wrist move to the other side
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(.5)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(.25);
    }

    public TrajectoryActionBuilder getPostHangLocationTrajectory(){
        TrajectoryActionBuilder PostHangLocationTrajectory = currentTrajectory
                //strafe to human pick up
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .strafeToSplineHeading(new Vector2d(49, 12), Math.toRadians(-90))
                .strafeToSplineHeading(new Vector2d(49, 8.25), Math.toRadians(-90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(.5);
        currentTrajectory = PostHangLocationTrajectory.endTrajectory().fresh();
        return  PostHangLocationTrajectory;
    }

    public TrajectoryActionBuilder getPark(boolean side){
        double xAxis3 = XAxis+12;
        if(side){
            xAxis3 = xAxis3*-1;
        }
        return currentTrajectory
                .strafeTo(new Vector2d(xAxis3, 4), speed);
    }

    public TrajectoryActionBuilder getPostHangAttachment(){
        return currentTrajectory
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(.5)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                ;
    }
}
