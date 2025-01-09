package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
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

public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    boolean side = false;
    VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25),
            new AngularVelConstraint(Math.PI)
    ));
    VelConstraint postHangVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(400),
            new AngularVelConstraint(Math.PI*.75)
    ));
    VelConstraint slowVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(25),
            new AngularVelConstraint(Math.PI*.25)
    ));
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                              VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                              HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                              HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR,
                              boolean side) {
        this.drive = drive;
        this.pose = pose;
        this.verticalSlideRR = verticalSlideRR;
        this.verticalWristRR = verticalWristRR;
        this.verticalGrabberRR = verticalGrabberRR;
        this.horizontalSlideRR = horizontalSlideRR;
        this.horizontalRollRR = horizontalRollRR;
        this.horizontalGrabberRR = horizontalGrabberRR;
        this.horizontalWristRR = horizontalWristRR;
        this.side = side;
        if(side){
            hangX = hangX*-1;
            butterX = butterX*-1;
            parkX = parkX*-1;
        }
        currentTrajectory = drive.actionBuilder(pose);
    }
    public static double hangX = -15;
    public static double butterX = 37;
    public static double parkX = 49;
    public static double butterY = 17.25;
    public static double postHangX = 49;
    public static double postHangY = 9.5;
    public static double basketX = -37;
    public static double basketY = 7.5;
    public static double basketHeading = 220;
    public Action getHangTrajectory() {
        TrajectoryActionBuilder hangTrajectory;
        if(!side) {
             hangTrajectory = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction((ConfigurationSecondRobot.highBar)))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .afterTime(0, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .strafeTo(new Vector2d(hangX, 0))
                    .strafeTo(new Vector2d(hangX, 29.75))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .strafeTo(new Vector2d(hangX, butterY))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom + 5))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket));
        }else{
            hangTrajectory = currentTrajectory
                    .afterTime(.25, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                    .afterTime(.25, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .strafeToSplineHeading(new Vector2d(basketX, basketY),Math.toRadians(basketHeading),baseVel)
                    .waitSeconds(.2)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom));
        }
        currentTrajectory = hangTrajectory.endTrajectory().fresh();
        return hangTrajectory.build();
    }
    public Action getButterLocationTrajectory() {
        TrajectoryActionBuilder butterLocationTrajectory;
        if(!side) {
            butterLocationTrajectory = currentTrajectory
                    .afterDisp(40, horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .afterDisp(40, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .afterDisp(40, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .strafeToSplineHeading(new Vector2d(butterX, butterY), Math.toRadians(-90), baseVel);
        }else{
            butterLocationTrajectory = currentTrajectory
                    .afterDisp(10,horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .afterDisp(10, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .afterDisp(10, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .strafeToSplineHeading(new Vector2d(butterX, butterY), Math.toRadians(-90),slowVel);
        }
        currentTrajectory = butterLocationTrajectory.endTrajectory().fresh();
        return butterLocationTrajectory.build();
    }
    public Action getButterPickUpTrajectory(){
        TrajectoryActionBuilder butterPickUpTrajectory;
        if(!side){
            butterPickUpTrajectory = currentTrajectory
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .strafeTo(new Vector2d(49, butterY-.5));
        }
        else{
            butterPickUpTrajectory = currentTrajectory
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime)
                    .strafeToSplineHeading(new Vector2d(basketX, basketY), Math.toRadians(basketHeading))
                    .waitSeconds(3)
                    .strafeTo(new Vector2d(49, butterY))
                    .strafeToSplineHeading(new Vector2d(basketX, basketY), Math.toRadians(basketHeading))
            ;
        }
        currentTrajectory = butterPickUpTrajectory.endTrajectory().fresh();
        return butterPickUpTrajectory.build();
    }
    public Action getButterPickUpAttachment(){
        TrajectoryActionBuilder butterPickUpAttachement;
        if(!side) {
            butterPickUpAttachement = currentTrajectory
                    //close the horizontal grabber and put the vertical wrist and grabber to position
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //butter now in transfer system
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    //vertical grabber close
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    //horizontal grabber open
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //vertical now go to other side while priming the horizontal slide
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .waitSeconds((ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)+.25)
                    //open the vertical grabber and close the horizontal grabber
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //retract the horizontal and flip the vertical back to transfer
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    //close the vertical grabber
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .waitSeconds(ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000);
        }else{
            butterPickUpAttachement = currentTrajectory
                    //close the horizontal grabber and put the vertical wrist and grabber to position
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //butter now in transfer system
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    //vertical grabber close
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    //horizontal grabber open
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //vertical slide extend and wrist to basket
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                    .waitSeconds(ConfigurationSecondRobot.verticalSlideBottomToHighBar/1000)
                    //vertical grabber open
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    //retract the slide and extend the horizontal slide
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer/1000)
                    //todo repeat
                    //close the horizontal grabber and put the vertical wrist and grabber to position
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //butter now in transfer system
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds((ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)+.25)
                    //vertical grabber close
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    //horizontal grabber open
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //vertical slide extend and wrist to basket
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                    .waitSeconds(ConfigurationSecondRobot.verticalSlideBottomToHighBar/1000)
                    //vertical grabber open
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
            ;
        }
        return butterPickUpAttachement.build();
    }
    public Action getHumanPickUpTrajectory(){
        TrajectoryActionBuilder humanPickUpTrajectory;
        if(!side){
            hangX += 2;
            humanPickUpTrajectory = currentTrajectory
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .strafeToSplineHeading(new Vector2d(49, butterY),Math.toRadians(-90),baseVel)
                    .strafeTo(new Vector2d(postHangX, postHangY))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .strafeToSplineHeading(new Vector2d(hangX, postHangY), Math.toRadians(90),postHangVel);
            postHangX = 33;
        }else{

            humanPickUpTrajectory = currentTrajectory;
        }
        currentTrajectory = humanPickUpTrajectory.endTrajectory().fresh();
        return humanPickUpTrajectory.build();
    }
    public Action getParkTrajectory(){
        TrajectoryActionBuilder parkTrajectory;
        if(!side){
            parkTrajectory = currentTrajectory
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .strafeTo(new Vector2d(49, 9.75));
        }else{
            parkTrajectory = currentTrajectory
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .strafeTo(new Vector2d(-49, 9.75));
        }
        currentTrajectory = parkTrajectory.endTrajectory().fresh();
        return parkTrajectory.build();
    }
}
