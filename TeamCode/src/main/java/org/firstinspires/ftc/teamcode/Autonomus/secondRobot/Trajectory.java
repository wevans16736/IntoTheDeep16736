package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.*;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR){
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
    VelConstraint slowerVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(Math.PI*.75)
    ));

    public Action getHangTrajectory(){
        TrajectoryActionBuilder hangTrajectory;
        hangTrajectory = currentTrajectory
                .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .splineTo(new Vector2d(0, 26), Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
        ;
        currentTrajectory = hangTrajectory.endTrajectory().fresh();
        return hangTrajectory.build();
    }
    public Action getFirstButterTrajectory(){
        TrajectoryActionBuilder firstButterTrajectory;
        firstButterTrajectory = currentTrajectory
                .afterDisp(5, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .afterDisp(5, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(22, 10, Math.toRadians(180)), Math.toRadians(0), slowerVel)
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .splineToLinearHeading(new Pose2d(38, 17, Math.toRadians(-90)), Math.toRadians(0), slowerVel)
        ;
        currentTrajectory = firstButterTrajectory.endTrajectory().fresh();
        return firstButterTrajectory.build();
    }
    public Action getSecondButterTrajectory(){
        TrajectoryActionBuilder secondButterTrajectory;
        secondButterTrajectory = currentTrajectory
                //todo
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .splineTo(new Vector2d(47.5, 17), Math.toRadians(0));
        currentTrajectory = secondButterTrajectory.endTrajectory().fresh();
        return secondButterTrajectory.build();
    }
    public Action getThirdButterTrajectory(){
        TrajectoryActionBuilder thirdButterTrajectory;
        thirdButterTrajectory = currentTrajectory
                .waitSeconds(3)
                .splineToSplineHeading(new Pose2d(42, 40, Math.toRadians(180)), Math.toRadians(-90), slowerVel);
        currentTrajectory = thirdButterTrajectory.endTrajectory().fresh();
        return thirdButterTrajectory.build();
    }
    public Action getButterAttachement(){
        TrajectoryActionBuilder butterAttachement;
        butterAttachement = currentTrajectory
                //grab the butter from the ground
                .waitSeconds(.1)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                //retract the horizontal slide and rotate the wrist
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                //close the vertical grabber
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                //open the horizontal grabber
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                //vertical wrist on the other side while horizontal slide extend for the second butter
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                //open the vertical grabber
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
        ;
        return butterAttachement.build();
    }
    public Action getThirdButterAttachment(){
        TrajectoryActionBuilder thirdButterAttachment;
        thirdButterAttachment = currentTrajectory
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                .waitSeconds(ConfigurationSecondRobot.flatToSidewaysTime/1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen));
        return  thirdButterAttachment.build();
    }
    public Action getPostHangLocation(){
        TrajectoryActionBuilder postHangLocation;
        postHangLocation = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000 + 1)
                .splineToLinearHeading(new Pose2d(40.0, 9.5, Math.toRadians(-90.0)), Math.toRadians(180.0))
        ;
        currentTrajectory = postHangLocation.endTrajectory().fresh();
        return postHangLocation.build();
    }
    public Action getPostHangTrajectory(){
        TrajectoryActionBuilder postHangTrajectory;
        postHangTrajectory = currentTrajectory
                .waitSeconds(.05)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .splineToLinearHeading(new Pose2d(17, 10, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .splineToLinearHeading(new Pose2d(-15, 26, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(18.0, 25.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .splineToLinearHeading(new Pose2d(35.0, 15, Math.toRadians(-90.0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(35, 9), Math.toRadians(-90));
        currentTrajectory = postHangTrajectory.endTrajectory().fresh();
        return postHangTrajectory.build();
    }
}
