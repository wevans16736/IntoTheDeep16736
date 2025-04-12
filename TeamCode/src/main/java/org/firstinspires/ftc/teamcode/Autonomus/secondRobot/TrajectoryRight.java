package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Pose;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Timing;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;

public class TrajectoryRight {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    RobotSensor robotSensor;
    public TrajectoryRight(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
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
    double hangX = -1; double hangY = -34.35; int hang = 0; int attachment = 0;
    double firstButterX = 48.75; double firstButterY = -36.25; double butterCounter = 0;
    double secondButterX = 58; double secondButterY =  -47.2;
    double thirdButterX = 61; double thirdButterY = -28.5;
    double humanX = 43; double humanY = -52; int human = 0;

    public Action getHang(){
        if(hang == 0){
            TrajectoryActionBuilder firstHang = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(Pose.verticalSlideHighBar))
                    .afterTime(0, verticalWristRR.verticalWristAction(Pose.verticalWristBar))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalOpenTime/1000);
            hangX = 1;
            hang++;
            currentTrajectory = firstHang.endTrajectory().fresh();
            return firstHang.build();
        } if(hang == 1){
            TrajectoryActionBuilder secondHang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalClose))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberOpen))
                    .waitSeconds(Timing.verticalCloseTime/1000 + .1)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideHighBar))
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristBar))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(200))
                    .splineToLinearHeading(new Pose2d(hangX, hangY + .75, Math.toRadians(90)), Math.toRadians(115))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
//                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristUp))
                    .waitSeconds(Timing.verticalCloseTime/1000);
            hangX += 1.5;
            hang++;
            currentTrajectory = secondHang.endTrajectory().fresh();
            return secondHang.build();

        } else {
            TrajectoryActionBuilder thirdHang = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalClose))
                    .waitSeconds(Timing.verticalCloseTime/1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristUp))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideHighBar))
                    .afterTime(1, verticalWristRR.verticalWristAction(Pose.verticalWristBar))
                    .strafeToLinearHeading(new Vector2d( humanX - 15, humanY), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(hangX, hangY + 1), Math.toRadians(90))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds((Timing.verticalCloseTime/1000))
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristWall))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen));
            hangX += 1.5;
            currentTrajectory = thirdHang.endTrajectory().fresh();
            return thirdHang.build();
        }
    }
    public Action getButter(){
        if(butterCounter == 0){
            TrajectoryActionBuilder firstButter = currentTrajectory
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideRetract))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.0001)), Math.toRadians(90));
            butterCounter++;
            currentTrajectory = firstButter.endTrajectory().fresh();
            return firstButter.build();
        } if(butterCounter == 1){
            TrajectoryActionBuilder secondButter = currentTrajectory
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0));
            butterCounter++;
            currentTrajectory = secondButter.endTrajectory().fresh();
            return secondButter.build();
        } else {
            TrajectoryActionBuilder thirdButter = currentTrajectory
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(350))
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(Pose.horizontalRollSideway))
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(0))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberClose))
                    .waitSeconds(Timing.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(Pose.horizontalRollFlat))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(55.5, -61.25, Math.toRadians(-.00000000001)), Math.toRadians(0))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideRetract))
                    .waitSeconds(Timing.horizontalGrabberCloseTime/1000);
            currentTrajectory = thirdButter.endTrajectory().fresh();
            return thirdButter.build();
        }
    }
    public Action getAttachment(){
        if(attachment == 0){
            TrajectoryActionBuilder firstAttachment = currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberClose))
                    .waitSeconds(Timing.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristTransfer))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideRetract))
                    .waitSeconds(Timing.horizontalWristIntaketoTransfer/1000+.1)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalClose))
                    .waitSeconds(Timing.verticalOpenTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberOpen))
                    .waitSeconds(Timing.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .waitSeconds(Timing.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalCloseTime/1000);
            attachment++;
            currentTrajectory = firstAttachment.endTrajectory().fresh();
            return  firstAttachment.build();
        } else {
            TrajectoryActionBuilder secondAttachment = currentTrajectory
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberClose))
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                    .waitSeconds(Timing.horizontalGrabberWideTime/1000)
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristTransfer))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideRetract))
                    .waitSeconds(Timing.horizontalWristIntaketoTransfer/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalClose))
                    .waitSeconds(Timing.verticalOpenTime/1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .waitSeconds(Timing.horizontalGrabberCloseTime/1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristWall))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .waitSeconds(Timing.verticalWristWalltoIntake/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalCloseTime/1000);
            currentTrajectory = secondAttachment.endTrajectory().fresh();
            return secondAttachment.build();
        }
    }
    public Action getHuman(){
            TrajectoryActionBuilder Human = currentTrajectory
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristUp))
                    .afterTime(1, verticalWristRR.verticalWristAction(Pose.verticalWristWall))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-89.9999999999)), Math.toRadians(-90));
            human++;
            currentTrajectory = Human.endTrajectory().fresh();
            return Human.build();
    }
}