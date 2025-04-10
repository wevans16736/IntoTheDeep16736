package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Championship;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Pose;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Timing;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.RR.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.LimeSweet;

import java.util.ArrayList;
import java.util.Arrays;

public class TrajectoryLeftChampionship {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    LimeSweet lime; ArrayList<Double> point; double grabX, grabY, angle; StrafeAction strafeAction;
    public TrajectoryLeftChampionship(PinpointDrive drive, Pose2d pose, LimeSweet lime,StrafeAction strafeAction, HorizontalGrabberRR horizontalGrabberRR,
                                      HorizontalRollRR horizontalRollRR, HorizontalSlideRR horizontalSlideRR,
                                      HorizontalWristRR horizontalWristRR, VerticalGrabberRR verticalGrabberRR,
                                      VerticalHangerRR verticalHangerRR, VerticalSlideRR verticalSlideRR,
                                      VerticalWristRR verticalWristRR){
        this.drive = drive;
        this.pose = pose;
        this.lime = lime;
        this.strafeAction = strafeAction;
        this.point = new ArrayList<>();
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
    VelConstraint subVel = new TranslationalVelConstraint(5);
    AccelConstraint subAccel = new ProfileAccelConstraint(-15, 10);

    int butter = 0; int attachment = 0; int park = 0; int sub = 0;
    double BX = -55; double BY = -54; double BH = Math.toRadians(220); double BT = Math.toRadians(220); int basket = 0;
    double FBX = -48.25; double FBY = -47.5; double FBH = Math.toRadians(-90); double FBT = Math.toRadians(90);
    double SBX = -58.75; double SBY = -45.5; double SBH = Math.toRadians(-90); double SBT = Math.toRadians(90);
    double TBX = -52; double TBY = -23.25; double TBH = Math.toRadians(-.000001); double TBT = Math.toRadians(180);
    double SX = -10; double SY = -5; double SH = Math.toRadians(180); double ST = Math.toRadians(0);
//    double humanX = 20.5, humanY = -63;

    public Action getBasket(){
        if(basket == 0){
            TrajectoryActionBuilder firstBasket = currentTrajectory
                    .afterTime(0, verticalSlideRR.verticalSlideAction(Pose.verticalSlideHighBasket))
                    .afterTime(0, verticalWristRR.verticalWristAction(Pose.verticalWristBasket))
                    .setTangent(Math.toRadians(Math.toRadians(135)))
                    .splineToLinearHeading(new Pose2d(BX, BY, BH), BT);
            basket++;
            currentTrajectory = firstBasket.fresh();
            return firstBasket.build();
        } if (basket == 1){
            TrajectoryActionBuilder secondBasket = currentTrajectory
                    .waitSeconds(Timing.horizontalGrabberCloseTime / 1000)
                    .waitSeconds(1.4)
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(BX, BY, BH), BT);
            basket++;
            currentTrajectory = secondBasket.fresh();
            return secondBasket.build();
        } if (basket == 2){
            TrajectoryActionBuilder thirdBasket = currentTrajectory
                    .waitSeconds(Timing.horizontalGrabberCloseTime / 1000)
                    .waitSeconds(1.4)
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(BX, BY, BH), BT);
            basket++;
            currentTrajectory = thirdBasket.fresh();
            return thirdBasket.build();
        } if (basket == 3) {
            TrajectoryActionBuilder forthBasket = currentTrajectory
                    .waitSeconds(Timing.horizontalGrabberCloseTime / 1000)
                    .setTangent(Math.toRadians(0)) //todo suckkkyyyy ahhhhhh :(((((((
                    .splineToLinearHeading(new Pose2d(-45, -35, Math.toRadians(-90)), Math.toRadians(-90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(BX, BY, BH), BT);
            basket++;
            currentTrajectory = forthBasket.fresh();
            return forthBasket.build();
        } else {
            TrajectoryActionBuilder restBasket = currentTrajectory
                    .waitSeconds(Timing.horizontalGrabberCloseTime)
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(BX, BY, BH), BT);
            basket++;
            currentTrajectory = restBasket.fresh();
            return restBasket.build();
        }
    }
    public Action getButter(){
        if(butter == 0){
            TrajectoryActionBuilder firstButter = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalOpenTime / 1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .setTangent(Math.toRadians(40))
                    .strafeToLinearHeading(new Vector2d(FBX, FBY), FBH);
            butter++;
            currentTrajectory = firstButter.fresh();
            return firstButter.build();
        } if(butter == 1){
            TrajectoryActionBuilder secondButter = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalOpenTime / 1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .setTangent(Math.toRadians(40))
                    .strafeToLinearHeading(new Vector2d(SBX, SBY), SBH);
            butter++;
            currentTrajectory = secondButter.fresh();
            return secondButter.build();
        } else {
            TrajectoryActionBuilder thirdButter = currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                    .waitSeconds(Timing.verticalOpenTime / 1000)
                    .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                    .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideExtend))
                    .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                    .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberWide))
                    .stopAndAdd(horizontalRollRR.horizontalRollAction(Pose.horizontalRollSideway))
                    .setTangent(Math.toRadians(40))
                    .splineToLinearHeading(new Pose2d(TBX, TBY, TBH), TBT);
            currentTrajectory = thirdButter.fresh();
            return thirdButter.build();
        }
    }
    public Action getTransfer(){
        TrajectoryActionBuilder transfer = currentTrajectory
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberClose))
                .waitSeconds(Timing.horizontalGrabberCloseTime / 1000)
                .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristTransfer))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(Pose.horizontalSlideRetract))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(Pose.horizontalRollFlat))
                .waitSeconds(Timing.horizontalWristIntaketoTransfer / 1000)
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalClose))
                .waitSeconds(Timing.verticalCloseTime / 1000)
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(Pose.horizontalGrabberOpen))
                .waitSeconds(Timing.horizontalGrabberCloseTime / 1000)
                .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideHighBasket))
                .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristBasket));
        currentTrajectory = transfer.fresh();
        return transfer.build();
    }
    public Action getSub(){
        TrajectoryActionBuilder sub = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(Pose.verticalOpen))
                .waitSeconds(Timing.verticalOpenTime / 1000)
                .stopAndAdd(verticalWristRR.verticalWristAction(Pose.verticalWristTransfer))
                .stopAndAdd(verticalSlideRR.verticalSlideAction(Pose.verticalSlideBottom))
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(new Pose2d(SX-20, SY, SH), ST)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(SX, SY, SH), ST, subVel, subAccel);
        currentTrajectory = sub.fresh();
        return sub.build();
    }
    public Action getSubButter() {
        TrajectoryActionBuilder subButter = currentTrajectory
                .stopAndAdd(horizontalSlideRR.horizontalSlideDistance(grabY, 3000))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(Pose.horizontalWristIntake))
                .stopAndAdd(horizontalRollRR.horizontalRollAction((angle / 90) * .3))
                .strafeToConstantHeading(new Vector2d(grabX + drive.getLastPinpointPose().position.y, drive.getLastPinpointPose().position.x));
        currentTrajectory = subButter.fresh();
        return subButter.build();
    }
    public Action getTest(){
        TrajectoryActionBuilder test = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(strafeAction.grabX + 1, 0));
        currentTrajectory = drive.actionBuilder(drive.getLastPinpointPose());
        return test.build();
    }
}
