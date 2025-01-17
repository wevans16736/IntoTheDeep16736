package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(-15, -64, Math.toRadians(180));

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
                .strafeTo(new Vector2d(-15, -60))
                .splineToLinearHeading(new Pose2d(-55, -55,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-48, -35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(-55, -55,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get the second butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57,-35, Math.toRadians(-90)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(-55, -55,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to third butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57, -25, Math.toRadians(0)), Math.toRadians(90))
                .waitSeconds(.5)
                //get to basket
                .splineToLinearHeading(new Pose2d(-55, -55,Math.toRadians(225)), Math.toRadians(225))
                .waitSeconds(.5)
                //get to parking
                .setReversed(true)
                .splineTo(new Vector2d(-35, -10), Math.toRadians(0))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}