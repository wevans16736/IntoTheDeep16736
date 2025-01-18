package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Decapricated;

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

@Deprecated
public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    boolean side = false;
    VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(75),
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
            hangX = Math.abs(hangX)*-1;
            butterX = Math.abs(butterX)*-1;
            parkX = Math.abs(parkX)*-1;
        }
        currentTrajectory = drive.actionBuilder(pose);
    }
    public static double hangX = 0;
    public static double butterX = 38;
    public static double parkX = 49;
    public static double butterY = 16.125;
    public static double postHangX = 49;
    public static double postHangY = 8;
    public static double basketX = -37.5;
    public static double basketY = 7.75;
    public static double basketHeading = 220;
    int count = 0;
    public Action getHangTrajectory() {
        if(count==1){
            hangX = -14;
        }
        if(count >= 2){
            hangX += 3;
        }
        TrajectoryActionBuilder hangTrajectory;
        if(!side) {
             hangTrajectory = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction((ConfigurationSecondRobot.highBar)))
                    .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                    .afterTime(0, verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .strafeTo(new Vector2d(hangX, postHangY))
                    .strafeTo(new Vector2d(hangX, 29.5))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .strafeTo(new Vector2d(hangX, butterY))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom + 5))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket));
        }else{
            hangTrajectory = currentTrajectory
                    .afterTime(.25, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                    .afterTime(.25, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .strafeTo(new Vector2d(0, basketY), baseVel)
                    .strafeToSplineHeading(new Vector2d(basketX, basketY),Math.toRadians(basketHeading),baseVel)
                    .waitSeconds(.2)
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime/1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom));
        }
        currentTrajectory = hangTrajectory.endTrajectory().fresh();
        count++;
        return hangTrajectory.build();
    }
    public Action getButterLocationTrajectory() {
        TrajectoryActionBuilder butterLocationTrajectory;
        if(!side) {
            butterLocationTrajectory = currentTrajectory
                    .afterDisp(40, horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .afterDisp(40, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .afterDisp(40, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .strafeToSplineHeading(new Vector2d(butterX, 16.125), Math.toRadians(-90), baseVel)
                    .waitSeconds(.25)
                    ;
        }else{
            butterLocationTrajectory = currentTrajectory
                    .afterDisp(0,horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .afterDisp(0, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .afterDisp(0, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                    .strafeToSplineHeading(new Vector2d(butterX+4.25, butterY+.5), Math.toRadians(-90),slowVel)
                    .waitSeconds(.1);
            butterX -= 12;
        }
        currentTrajectory = butterLocationTrajectory.endTrajectory().fresh();
        return butterLocationTrajectory.build();
    }
    public Action getButterPickUpTrajectory(){
        TrajectoryActionBuilder butterPickUpTrajectory;
        if(!side){
            butterPickUpTrajectory = currentTrajectory
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime/1000)
                    .waitSeconds(1)
                    .strafeTo(new Vector2d(49, 16.126));
        } else{
            butterPickUpTrajectory = currentTrajectory
                    .waitSeconds(2)
                    .strafeToSplineHeading(new Vector2d(basketX-3, basketY-3), Math.toRadians(basketHeading),slowVel)
                    .waitSeconds(.5)
            ;
            basketX -=.5;
            basketY -= .5;
        }
        currentTrajectory = butterPickUpTrajectory.endTrajectory().fresh();
        return butterPickUpTrajectory.build();
    }
    int counter = 0;
    int secondDelay = 1;

    public Action getButterPickUpAttachment(){
        TrajectoryActionBuilder butterPickUpAttachement;
        counter += 1;
        if(counter == 3){
            secondDelay = 3;
        }
        if(!side) {
            butterPickUpAttachement = currentTrajectory
                    //close the horizontal grabber and put the vertical wrist and grabber to position
                    .waitSeconds(.1)
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
                    .waitSeconds((ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)+.1)
                    //vertical now go to other side while priming the horizontal slide
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                    .waitSeconds((ConfigurationSecondRobot.verticalWristWalltoIntake / 1000)+.1)
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
        }else {
            butterPickUpAttachement = currentTrajectory
                    //close the horizontal grabber
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //retract the horizontal slide and prime the vertical grabber
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                    .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                    //close the vertical grabber
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    //open the horizontal grabber
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberOpen))
                    .waitSeconds(ConfigurationSecondRobot.horizontalGrabberCloseTime / 1000)
                    //extend the vertical slide
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                    .waitSeconds((ConfigurationSecondRobot.verticalSlideBottomToHighBar / 1000) + secondDelay)
                    //open the grabber
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    //retract the vertical slide
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .waitSeconds(ConfigurationSecondRobot.verticalSlideBottomToHighBar / 1000)
            ;
        }
        return butterPickUpAttachement.build();
    }
    public Action getThirdButterTrajectory(){
        TrajectoryActionBuilder thirdButterTrajectory;
        thirdButterTrajectory = currentTrajectory
                .splineToLinearHeading(new Pose2d(-30, butterY+20, Math.toRadians(0)), Math.toRadians(0), baseVel)
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .strafeTo(new Vector2d(-38.25, butterY+22), baseVel)
                        ;
        currentTrajectory = thirdButterTrajectory.endTrajectory().fresh();
        return thirdButterTrajectory.build();
    }
    public Action getHumanPickUpTrajectory(){
        TrajectoryActionBuilder humanPickUpTrajectory;
        if(!side){
            hangX += 2;
            humanPickUpTrajectory = currentTrajectory
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpenWide))
                    .strafeToSplineHeading(new Vector2d(postHangX, butterY),Math.toRadians(-90),baseVel)
                    .strafeTo(new Vector2d(postHangX, postHangY))
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                    .waitSeconds((ConfigurationSecondRobot.verticalCloseTime/1000))
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
                    .strafeTo(new Vector2d(-49, 11));
        }
        currentTrajectory = parkTrajectory.endTrajectory().fresh();
        return parkTrajectory.build();
    }
}
