package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryRight {
    public static void main(String[] args) {
        double hangX = 9;
        double hangY = -33;
        double firstButterX = 49;
        double firstButterY = -35;
        double secondButterX = 59;
        double secondButterY = -35;
        double thirdButterX = 59;
        double thirdButterY = -25;
        double postHumanY = -45;
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(9, -64, Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
//                .splineToLinearHeading(new Pose2d(9, -33, Math.toRadians(90)), Math.toRadians(90))
//                //move to first butter
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(49, -35, Math.toRadians(-90.00000001)), Math.toRadians(90))
//                //move to human player
//                .waitSeconds(.5)
//                .splineToLinearHeading(new Pose2d(49, -47, Math.toRadians(-90)), Math.toRadians(-90))
//                //move to second butter
//                .waitSeconds(.5)
//                .splineToLinearHeading(new Pose2d(59, -35, Math.toRadians(-90)), Math.toRadians(0))
//                //move to human player
//                .waitSeconds(.5)
//                .splineToLinearHeading(new Pose2d(59, -47, Math.toRadians(-90)), Math.toRadians(-90))
//                //move to third butter
//                .waitSeconds(.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(59, -25, Math.toRadians(180)), Math.toRadians(90))
//                //move to human player
//                .waitSeconds(.5)
////               .splineToLinearHeading(new Pose2d(43, -55, Math.toRadians(-90)), Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-.00000000001)), Math.toRadians(0))
//                //move to hang
//                .waitSeconds(.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(43, -45, Math.toRadians(-90)), Math.toRadians(90))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-9, -33, Math.toRadians(90.0000001)), Math.toRadians(90))
//                //move back to human player
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(43, -45, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(43, -55, Math.toRadians(-90)), Math.toRadians(-90))

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
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-.00000000001)), Math.toRadians(0))
                //move to hang
                .waitSeconds(.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-9, -33, Math.toRadians(90.0000001)), Math.toRadians(90))
                //move back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(43, postHumanY, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(43, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}