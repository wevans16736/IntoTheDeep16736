package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryRight {
    public static void main(String[] args) {
        double hangX = 0;
        double hangY = -34;
        double firstButterX = 48.5;
        double firstButterY = -40;
        double secondButterX = 59;
        double secondButterY = -40;
        double thirdButterX = 60;
        double thirdButterY = -27;
        double postHumanY = -43.25;
        double postHumanPickUpY = -54.25;
        double barX = -3;
        double humanX = 40;
        double humanY = -60;
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(200, 200, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(9, -64, Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
                //hang
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(.5)
                //back up a little
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.01)), Math.toRadians(90))
                .waitSeconds(.5)
                //second butter
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0))
                .waitSeconds(.5)
                //third butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY , Math.toRadians(180)), Math.toRadians(90))
                .waitSeconds(.5)
                //go to human pickup
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(-.000001)), Math.toRadians(0))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-3, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-3, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go back to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .waitSeconds(.5)
                //go to hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, hangY, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(0, hangY-5, Math.toRadians(90)), Math.toRadians(-90))
                //go to human player
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90.00001)), Math.toRadians(-90))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}