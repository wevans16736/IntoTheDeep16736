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

        Pose2d pose = new Pose2d(-39.5, -62.25, Math.toRadians(180));
        int butter = 0; int transferCount = 0;
        double BX = -55; double BY = -54.25; double BH = Math.toRadians(220); double BT = Math.toRadians(220); int basket = 0;
        double FBX = -48.75; double FBY = -43.5; double FBH = Math.toRadians(-90); double FBT = Math.toRadians(90);
        double SBX = -58.75; double SBY = -41.5; double SBH = Math.toRadians(-90); double SBT = Math.toRadians(90);
        double TBX = -56; double TBY = -23.25; double TBH = Math.toRadians(-.000001); double TBT = Math.toRadians(180);
        double SX = -25; double SY = 0; double SH = Math.toRadians(180); double ST = Math.toRadians(0);
        double MX = -45; double MY = -20;
        double sec = 1.2;

        myBot.runAction(myBot.getDrive().actionBuilder(pose)
                .setTangent(Math.toRadians(Math.toRadians(135)))
                .splineToLinearHeading(new Pose2d(BX, BY-.5, BH), BT)
                .setTangent(Math.toRadians(40))
                .strafeToLinearHeading(new Vector2d(FBX, FBY), FBH)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BX, BY, BH), BT)
                .setTangent(Math.toRadians(40))
                .strafeToLinearHeading(new Vector2d(SBX, SBY), SBH)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BX, BY, BH), BT)
                .setTangent(Math.toRadians(40))
                .splineToLinearHeading(new Pose2d(TBX, TBY, TBH), TBT)
                .waitSeconds(150 / 1000)
                .waitSeconds(.5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(BX, BY, BH), BT)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(MX, MY, Math.toRadians(-90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SX, SY, SH), ST)

                //returning back to Basket
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(MX, MY, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(BX, BY, BH), BT)
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}