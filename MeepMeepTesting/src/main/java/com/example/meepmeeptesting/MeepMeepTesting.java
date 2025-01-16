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
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 14.257357477632226)
                .build();

        Pose2d drive = new Pose2d(9, -64, Math.toRadians(90));

        myBot.runAction(myBot.getDrive().actionBuilder(drive)
                .splineToLinearHeading(new Pose2d(9, -33, Math.toRadians(90)), Math.toRadians(90))
                //move to first butter
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(47, -35, Math.toRadians(-90.00000001)), Math.toRadians(90))
                //move to second butter
                .waitSeconds(.5)
                .splineToLinearHeading(new Pose2d(57, -35, Math.toRadians(-90)), Math.toRadians(0))
                //move to third butter
                .waitSeconds(.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(59, -25, Math.toRadians(180)), Math.toRadians(90))

                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}