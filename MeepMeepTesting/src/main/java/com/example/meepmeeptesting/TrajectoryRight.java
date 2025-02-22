package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TrajectoryRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(200, 200, Math.toRadians(1800000), Math.toRadians(18000000), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(0, 0, Math.toRadians(0));

        double hangX = 0; double hangY = -35; int hang = 0; int attachment = 0;
        double firstButterX = 49; double firstButterY = -37.25; double butterCounter = 0;
        double secondButterX = 58; double secondButterY =  -47.75;
        double thirdButterX = 61; double thirdButterY = -28.5;
        double humanX = 35; double humanY = -60; int human = 0;

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
//                //hang the first butter
//                .splineToLinearHeading(new Pose2d(0, -30, Math.toRadians(90)), Math.toRadians(90))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(-90))
//                //first butter
//                .splineToLinearHeading(new Pose2d(48, -35, Math.toRadians(-90)), Math.toRadians(90))
//                //second butter
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(58,-45, Math.toRadians(-90)) , Math.toRadians(-90))
//                //third butter
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(58,-25, Math.toRadians(180)), Math.toRadians(0))
//                //go to human player
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(61,-60, Math.toRadians(-0.1)), Math.toRadians(0))
//                //avoid the third butter
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(40,-40, Math.toRadians(179.9)), Math.toRadians(180))
//
//                //hang 2
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-10,-32, Math.toRadians(90)), Math.toRadians(90))
//                //human pick up 2
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(61,-58, Math.toRadians(0)), Math.toRadians(0))
//                //hang 2
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-8,-32, Math.toRadians(90)), Math.toRadians(90))
//                //human pick up 2
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(61,-58, Math.toRadians(0)), Math.toRadians(0))
//                //hang 2
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-6,-32, Math.toRadians(90)), Math.toRadians(90))
//                //human pick up 2
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(61,-58, Math.toRadians(0)), Math.toRadians(0))
//                //hang 2
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-4,-32, Math.toRadians(90)), Math.toRadians(90))
//                //human pick up 2
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(61,-58, Math.toRadians(0)), Math.toRadians(0))

//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.0001)), Math.toRadians(90))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(59, -58, Math.toRadians(-.0001)), Math.toRadians(0))
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
//                        .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))

                        .setTangent(-Math.PI/2)
//                        .splineToLinearHeading(new Pose2d(-6, 0, Math.toRadians(-180)), Math.toRadians(-180))
                        .turn(Math.toRadians(180))
//                        .waitSeconds(5)
                        .setTangent(-Math.PI/1.5)
                .lineToY(-48)

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}