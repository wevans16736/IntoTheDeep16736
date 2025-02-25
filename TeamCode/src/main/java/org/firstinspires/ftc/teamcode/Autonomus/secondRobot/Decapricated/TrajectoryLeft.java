package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Decapricated;

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
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class TrajectoryLeft {
    VerticalSlideRR verticalSlideRR;
    VerticalWristRR verticalWristRR;
    VerticalGrabberRR verticalGrabberRR;
    VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR;
    HorizontalRollRR horizontalRollRR;
    HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR;
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    double BasketX = -56.5; double BasketY = -55;
    double FirstButterX = -48.5; double FirstButterY = -35;
    double SecondButterX = -63; double SecondButterY = -34.5;
    double ThirdButterX = -60.5; double ThirdButterY = -24;
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

    public Action getAllTrajectory() {
        return currentTrajectory
                //get to the basket
//                .strafeTo(new Vector2d(-15, -60))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(FirstButterX, FirstButterY, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the second butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(SecondButterX, SecondButterY, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to third butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(ThirdButterX, ThirdButterY, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to parking
                .setReversed(true)
                .splineTo(new Vector2d(-35, -10), Math.toRadians(0))
                .build();
    }

    public Action getInitialBasket() {
        TrajectoryActionBuilder InitialBasket = currentTrajectory
                .afterTime(0, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                .afterTime(0, verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                .afterTime(0, horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .afterTime(0, horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen));
        currentTrajectory = InitialBasket.endTrajectory().fresh();
        return InitialBasket.build();
    }

    public Action getFirstButter() {
        TrajectoryActionBuilder FirstButter = currentTrajectory
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .afterTime(.5, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(FirstButterX, FirstButterY, Math.toRadians(-90)), Math.toRadians(90));
        currentTrajectory = FirstButter.endTrajectory().fresh();
        return FirstButter.build();
    }

    public Action getBasket(boolean submersible, boolean attempt) {
        if (attempt) {
            if (!submersible) {
                TrajectoryActionBuilder Basket = currentTrajectory
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                        .waitSeconds(2)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225));
                currentTrajectory = Basket.endTrajectory().fresh();
                return Basket.build();
            } else {
                TrajectoryActionBuilder Basket = currentTrajectory
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(BasketX + 15, 0, Math.toRadians(180)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(BasketX + 10, BasketY + 10, Math.toRadians(225)), Math.toRadians(225))
                        .splineToLinearHeading(new Pose2d(BasketX, BasketY, Math.toRadians(225)), Math.toRadians(225));
                currentTrajectory = Basket.endTrajectory().fresh();

                return Basket.build();
            }
        } else {
            return currentTrajectory.build();
        }
    }

    public Action getSecondButter() {
        TrajectoryActionBuilder SecondButter = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                .waitSeconds(.25)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .afterTime(.5, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(SecondButterX, SecondButterY, Math.toRadians(-90)), Math.toRadians(90));
        currentTrajectory = SecondButter.endTrajectory().fresh();
        return SecondButter.build();
    }

    public Action getThirdButter() {
        TrajectoryActionBuilder ThirdButter = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                .waitSeconds(.25)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                .afterTime(.5, verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(300))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(ThirdButterX, ThirdButterY, Math.toRadians(0)), Math.toRadians(90));
        currentTrajectory = ThirdButter.endTrajectory().fresh();
        return ThirdButter.build();
    }

    public Action getButterAttachment(boolean submersible, boolean attempt) {
        if (attempt) {
            if (!submersible) {
                return currentTrajectory
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                        .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                        .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                        .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                        .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                        .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                        .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                        .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                        .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                        .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                        .build();
            } else {
                return currentTrajectory
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                        .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                        .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract))
                        .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                        .waitSeconds(ConfigurationSecondRobot.horizontalWristIntaketoTransfer / 1000)
                        .waitSeconds(.5)
                        .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                        .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                        .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime / 1000)
//                        .waitSeconds(.5)
                        .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBasket))
                        .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.topBasket))
                        .build();
            }
        } else {
            return currentTrajectory.build();
        }
    }

    public Action getSubmersible(boolean sideway, boolean attempt) {
        if (attempt) {
            if (!sideway) {
                TrajectoryActionBuilder Submersible = currentTrajectory
                        .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                        .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                        .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                        .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(BasketX + 15, 0, Math.toRadians(180)), Math.toRadians(90))
                        .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristHover))
                        .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(180)), Math.toRadians(0))
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                        .waitSeconds(.5);
                currentTrajectory = Submersible.endTrajectory().fresh();
                return Submersible.build();
            } else {
                TrajectoryActionBuilder Submersible = currentTrajectory
                        .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                        .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                        .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                        .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(BasketX + 15, 0, Math.toRadians(180)), Math.toRadians(90))
                        .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideExtend))
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristHover))
                        .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(180)), Math.toRadians(0))
                        .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                        .waitSeconds(.25)
                        .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.sideway))
                        .waitSeconds(.25);
                currentTrajectory = Submersible.endTrajectory().fresh();
                return Submersible.build();
            }
        } else {
            return currentTrajectory
                    .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                    .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                    .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristIntake))
                    .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                    .build();
        }
    }

    public Action getPark() {
        TrajectoryActionBuilder Park = currentTrajectory
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .waitSeconds(ConfigurationSecondRobot.verticalCloseTime / 1000)
                .waitSeconds(.2)
                .stopAndAdd(verticalSlideRR.verticalSlideAction(ConfigurationSecondRobot.bottom))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristUp))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(BasketX, 0, Math.toRadians(0)), Math.toRadians(90))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalClose))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.1)
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristBar))
                .waitSeconds(1);
        currentTrajectory = Park.endTrajectory().fresh();
        return Park.build();
    }
}
