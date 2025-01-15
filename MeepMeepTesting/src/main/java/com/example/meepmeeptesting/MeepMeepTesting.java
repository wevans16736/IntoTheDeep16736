package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.prefs.PreferencesFactory;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(9, -64, Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
                .splineTo(new Vector2d(9,-35), Math.toRadians(90))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(48, -40, Math.toRadians(-90)), Math.toRadians(0))
                .splineTo(new Vector2d(58, -40), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -25, Math.toRadians(180)), Math.toRadians(90))

                .splineToLinearHeading(new Pose2d(40, -55, Math.toRadians(-90)), Math.toRadians(-90))

                .splineToLinearHeading(new Pose2d(-13, -35, Math.toRadians(90)), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(40, -50, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineTo(new Vector2d(40, -55), Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}