package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Vision;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Trajectory {
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; PinpointDrive drive; Pose2d pose; TrajectoryActionBuilder currentTrajectory;
    Telemetry telemetry; double x, y = 0;
    public Trajectory(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                      VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                      VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                      HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR, Telemetry telemetry){
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
        this.telemetry = telemetry;
        currentTrajectory = drive.actionBuilder(pose);

    }
   public Action getInitial(){
        TrajectoryActionBuilder initial = currentTrajectory
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(200))
                .stopAndAdd(verticalWristRR.VerticalWristAction(ConfigurationSecondRobot.verticalWristWall))
                .strafeToConstantHeading(new Vector2d(0, 0));
        currentTrajectory = initial.endTrajectory().fresh();
        return initial.build();
   }
   public Action getButter(){
       int tick = horizontalSlideRR.setSlideDistanceMath(y + horizontalSlideRR.getSlideDistanceMath(), 3000);
       x = GlobalVariables.Y + drive.getLastPinpointPose().position.x;
       y = drive.getLastPinpointPose().position.y;
        telemetry.addData("tick", tick);
        telemetry.update();
        TrajectoryActionBuilder Butter = currentTrajectory
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(tick))
                .stopAndAdd(horizontalRollRR.horizontalRollAction(horizontalWristRR.getRotation()))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberWide))
                .stopAndAdd(verticalGrabberRR.verticalGrabberAction(ConfigurationSecondRobot.verticalOpen))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristIntake))
                .strafeToConstantHeading(new Vector2d(x, y))
                .stopAndAdd(horizontalGrabberRR.horizontalGrabberAction(ConfigurationSecondRobot.horizontalGrabberClose))
                .waitSeconds(ConfigurationSecondRobot.horizontalGrabberWideTime/1000)
                .stopAndAdd(horizontalRollRR.horizontalRollAction(ConfigurationSecondRobot.flat))
                .stopAndAdd(horizontalWristRR.horizontalWristAction(ConfigurationSecondRobot.horizontalWristTransfer))
                .stopAndAdd(horizontalSlideRR.horizontalSlideActions(ConfigurationSecondRobot.horizontalSlideRetract));
        currentTrajectory = Butter.endTrajectory().fresh();
        return Butter.build();
   }
}
