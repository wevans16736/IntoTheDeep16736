package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

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
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

public class TrajectoryLeft {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR;
    VerticalHangerRR verticalHangerRR; HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR;
    HorizontalGrabberRR horizontalGrabberRR; HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;

    public TrajectoryLeft(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR) {
        this.drive = drive;
        this.pose = pose;
        this.verticalSlideRR = verticalSlideRR;
        this.verticalWristRR = verticalWristRR;
        this.verticalGrabberRR = verticalGrabberRR;
        this.verticalHangerRR = verticalHangerRR;
        this.horizontalSlideRR = horizontalSlideRR;
        this.horizontalRollRR = horizontalRollRR;
        this.horizontalGrabberRR = horizontalGrabberRR;
        this.horizontalWristRR = horizontalWristRR;

        currentTrajectory = drive.actionBuilder(pose);
    }
    double BasketX = -55;
    double BasketY = -55;
    public Action getAllTrajectory(){
        return currentTrajectory
                //get to the basket
                .strafeTo(new Vector2d(-15, -60))
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, -35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the second butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57,-35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to third butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57, -25, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to parking
                .setReversed(true)
                .splineTo(new Vector2d(-35, -10), Math.toRadians(0))
                .build();
    }
    public Action getInitialBasket(){
        TrajectoryActionBuilder InitialBasket = currentTrajectory
                .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .afterTime(0, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .afterTime(0, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .strafeTo(new Vector2d(-15, -60))
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen));
        currentTrajectory = InitialBasket.endTrajectory().fresh();
        return InitialBasket.build();
    }
    public Action getFirstButter(){
        TrajectoryActionBuilder FirstButter = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, -35, Math.toRadians(-90)), Math.toRadians(90));
        currentTrajectory = FirstButter.endTrajectory().fresh();
        return FirstButter.build();
    }
    public Action getBasket(){
        TrajectoryActionBuilder Basket = currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .splineToLinearHeading(new Pose2d(BasketX, BasketY,Math.toRadians(225)), Math.toRadians(225));
        currentTrajectory = Basket.endTrajectory().fresh();
        return Basket.build();
    }
    public Action getSecondButter(){
        TrajectoryActionBuilder SecondButter = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57,-35, Math.toRadians(-90)), Math.toRadians(90));
        currentTrajectory = SecondButter.endTrajectory().fresh();
        return SecondButter.build();
    }
    public Action getThirdButter(){
        TrajectoryActionBuilder ThirdButter = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57, -25, Math.toRadians(0)), Math.toRadians(90));
        currentTrajectory = ThirdButter.endTrajectory().fresh();
        return ThirdButter.build();
    }
    public Action getButterAttachment(){
        return currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                .build();
    }
    public Action getPark(){
        TrajectoryActionBuilder Park = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .setReversed(true)
                .splineTo(new Vector2d(-35, -10), Math.toRadians(0));
        currentTrajectory = Park.endTrajectory().fresh();
        return Park.build();
    }
}
