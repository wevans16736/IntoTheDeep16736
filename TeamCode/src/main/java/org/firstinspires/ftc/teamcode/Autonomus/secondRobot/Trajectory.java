package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.PinpointDrive;

public class Trajectory {
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    public Trajectory(PinpointDrive drive, Pose2d pose){
        this.drive = drive;
        this.pose = pose;
    }

    public void setCurrentPose(TrajectoryActionBuilder currentTrajectory){
        this.currentTrajectory = currentTrajectory;
    }
    TrajectoryActionBuilder HangTrajectory = drive.actionBuilder(pose)
            .strafeTo(new Vector2d(-15, 30));

    TrajectoryActionBuilder HangAttachment = drive.actionBuilder(pose);

    TrajectoryActionBuilder ButterPickupAttachment = drive.actionBuilder(pose);

    public Action runButterPickup(){
        Action ActionButterPickup = currentTrajectory.endTrajectory().fresh()
                .strafeTo(new Vector2d(0,0))
                .build();

        return  ActionButterPickup;
    }
}
