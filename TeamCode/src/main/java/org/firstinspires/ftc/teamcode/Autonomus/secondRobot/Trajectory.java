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
            new TranslationalVelConstraint(15),
            new AngularVelConstraint(Math.PI)
    ));

    VelConstraint speed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(1000),
            new AngularVelConstraint(Math.PI*25)
    ));

    VelConstraint SuperSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(10000000),
            new AngularVelConstraint(Math.PI*25000)
    ));
    double hangX = 0;
    double hangY = -34;
    double firstButterX = 48.5;
    double firstButterY = -34.8;
    double secondButterX = 59;
    double secondButterY = -34.8;
    double thirdButterX = 60;
    double thirdButterY = -27;
    double postHumanY = -43.25;
    double humanX = 55;
    double postHumanPickUpY = -54.25;
    public Action getAlltrajectory() {
        return currentTrajectory
                //hang
                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                //move to first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.00000001)), Math.toRadians(90))
                //move to human player
                .waitSeconds(.5)
                .splineToLinearHeading(new Pose2d(49, -47, Math.toRadians(-90)), Math.toRadians(-90))
                //move to second butter
                .waitSeconds(.5)
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0))
                //move to human player
                .waitSeconds(.5)
                .splineToLinearHeading(new Pose2d(59, -47, Math.toRadians(-90)), Math.toRadians(-90))
                //move to third butter
                .waitSeconds(.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(90))
                //move to human player
                .waitSeconds(.5)
//               .splineToLinearHeading(new Pose2d(43, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(humanX, -60, Math.toRadians(-.00000000001)), Math.toRadians(0))
                //move to hang
                .waitSeconds(.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-5, -33, Math.toRadians(90.0000001)), Math.toRadians(90))
                //move back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(43, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .build();
    }
    public Action testTransfer(){
        return currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .build();
    }

    public Action getHang(){
        TrajectoryActionBuilder Hang = currentTrajectory
                .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .afterTime(0, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90));
        currentTrajectory = Hang.endTrajectory().fresh();
        return Hang.build();
    }
    public Action getFirstButter(){
        TrajectoryActionBuilder FirstButter = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .afterTime(2, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.00000001)), Math.toRadians(90));
        currentTrajectory = FirstButter.endTrajectory().fresh();
        return  FirstButter.build();
    }
    public Action getFirstButterDropOff(){
        TrajectoryActionBuilder FirstButterDropOff = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .splineToLinearHeading(new Pose2d(firstButterX,-40,Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(secondButterX, -48, Math.toRadians(-90)), Math.toRadians(-90));
        currentTrajectory = FirstButterDropOff.endTrajectory().fresh();
        return FirstButterDropOff.build();
    }
    public Action getSecondButter(){
        TrajectoryActionBuilder SecondButter = currentTrajectory
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0));
        currentTrajectory = SecondButter.endTrajectory().fresh();
        return SecondButter.build();
    }
    public Action getSecondButterDropOff(){
        TrajectoryActionBuilder SecondButterDropOff = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(secondButterX, postHumanPickUpY, Math.toRadians(-90)), Math.toRadians(-90), slowerVel)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide));
        currentTrajectory = SecondButterDropOff.endTrajectory().fresh();
        return SecondButterDropOff.build();
    }
    public Action getThirdButter(){
        TrajectoryActionBuilder ThirdButter = currentTrajectory
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(90))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose));
        currentTrajectory = ThirdButter.endTrajectory().fresh();
        return ThirdButter.build();
    }
    public Action getThirdButterDropOff(){
        TrajectoryActionBuilder ThirdButterDropOff = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .splineToLinearHeading(new Pose2d(humanX, -62, Math.toRadians(-.00000000001)), Math.toRadians(0))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen));
        currentTrajectory = ThirdButterDropOff.endTrajectory().fresh();
        return  ThirdButterDropOff.build();
    }
    double barX = -3;
    public Action getPostHang(){
        TrajectoryActionBuilder PostHang = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                //move to semi circle trajectory
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90), speed)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(barX, -33, Math.toRadians(90.0000001)), Math.toRadians(90), speed)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                .splineToLinearHeading(new Pose2d(barX,-35, Math.toRadians(90)), Math.toRadians(-90)) //todo add this
                //move back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90), speed)
                .splineToLinearHeading(new Pose2d(43, postHumanPickUpY, Math.toRadians(-90)), Math.toRadians(-90));
        barX +=2;
        currentTrajectory = PostHang.endTrajectory().fresh();
        return PostHang.build();
    }
    public Action getPark(){
        TrajectoryActionBuilder Park = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                //move to semi circle trajectory
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90), speed)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.highBar))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(barX, -33, Math.toRadians(90.0000001)), Math.toRadians(90), speed)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
//                .splineToLinearHeading(new Pose2d(barX,-35, Math.toRadians(90)), Math.toRadians(-90)) //todo add this
                //move back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(45, postHumanPickUpY, Math.toRadians(-90)), Math.toRadians(-90));
                ;
        currentTrajectory.endTrajectory().fresh();
        return Park.build();
    }
    public Action ButterTransferAttachment(){
        TrajectoryActionBuilder ButterTransferAttachment = currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake/1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide));
        return ButterTransferAttachment.build();
    }

}
