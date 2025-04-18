package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ConceptRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(200, 200, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(9, -64, Math.toRadians(90));

        double hangX = -1; double hangY = -34.35; int hang = 0; int attachment = 0;
        double firstButterX = 48.75; double firstButterY = -35.5; double butterCounter = 0;
        double secondButterX = 58; double secondButterY =  -46;
        double thirdButterX = 64; double thirdButterY = -28.5;
        double humanX = 43; double humanY = -52.15; int human = 0;

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
                //hang
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(hangX, hangY, Math.toRadians(90)), Math.toRadians(90))
                //first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(firstButterX, firstButterY, Math.toRadians(-90.0001)), Math.toRadians(90))
                //second butter
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(secondButterX, secondButterY, Math.toRadians(-90)), Math.toRadians(0))
                //third butter
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(thirdButterX, thirdButterY, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(55, -61.25, Math.toRadians(-.00000000001)), Math.toRadians(0))
                //second hang
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(45, -45, Math.toRadians(179.999)), Math.toRadians(200))
                .splineToLinearHeading(new Pose2d(hangX, hangY + 1.5, Math.toRadians(90)), Math.toRadians(115))
                //human
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(28, -45, Math.toRadians(-179)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(humanX, humanY, Math.toRadians(-89.9999999999)), Math.toRadians(-90))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}