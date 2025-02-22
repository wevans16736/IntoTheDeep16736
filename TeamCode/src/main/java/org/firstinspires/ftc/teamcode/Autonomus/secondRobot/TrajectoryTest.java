package org.firstinspires.ftc.teamcode.Autonomus.secondRobot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.RobotSensor;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

public class TrajectoryTest {
    VerticalSlideRR verticalSlideRR;
    VerticalWristRR verticalWristRR;
    VerticalGrabberRR verticalGrabberRR;
    VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR;
    HorizontalRollRR horizontalRollRR;
    HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR;
    PinpointDrive drive;
    Pose2d pose;
    TrajectoryActionBuilder currentTrajectory;
    RobotSensor robotSensor;
    Telemetry telemetry;

    public TrajectoryTest(PinpointDrive drive, Pose2d pose, VerticalSlideRR verticalSlideRR,
                          VerticalWristRR verticalWristRR, VerticalGrabberRR verticalGrabberRR,
                          VerticalHangerRR verticalHangerRR, HorizontalSlideRR horizontalSlideRR, HorizontalRollRR horizontalRollRR,
                          HorizontalGrabberRR horizontalGrabberRR, HorizontalWristRR horizontalWristRR, RobotSensor robotSensor, Telemetry telemetry) {
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
        this.robotSensor = robotSensor;

        this.telemetry = telemetry;
        currentTrajectory = drive.actionBuilder(pose);
    }

    VelConstraint butterSpeed = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(50),
            new AngularVelConstraint(Math.PI)
    ));
    public Action moveSide(){
        TrajectoryActionBuilder Sideway = currentTrajectory
                .strafeToConstantHeading(new Vector2d( -0, 0));
        currentTrajectory = Sideway.endTrajectory().fresh();
        return Sideway.build();
    }
    public Action grabButter(){
        double x = GlobalVariables.X;
        double y = GlobalVariables.Y;
        TrajectoryActionBuilder Butter = currentTrajectory
                .strafeToConstantHeading(new Vector2d(x,y))
                .stopAndAdd(robotSensor.visionOff());
//        .strafeToConstantHeading(new Vector2d(0, -5));
        telemetry.addData("X", GlobalVariables.X);
        telemetry.addData("Y", GlobalVariables.Y);
        telemetry.update();
        currentTrajectory = Butter.endTrajectory().fresh();
        return  Butter.build();
    }
    public Action move(){
        TrajectoryActionBuilder Move = currentTrajectory
                .strafeTo(new Vector2d(0, 20));
        currentTrajectory = Move.endTrajectory().fresh();
        return Move.build();
    }
    public Action scanButter(){
        TrajectoryActionBuilder scan = currentTrajectory
                .stopAndAdd(robotSensor.visionOn())
//                .waitSeconds(3)
                .stopAndAdd(robotSensor.visionOff())
                .waitSeconds(3);
//                .stopAndAdd(robotSensor.visionScan());
        currentTrajectory = scan.endTrajectory().fresh();
        return scan.build();
    }
    public Action turn(){
        TrajectoryActionBuilder turn = currentTrajectory
                .strafeToLinearHeading(new Vector2d(0, -7), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(25, -17), Math.toRadians(-90));
        currentTrajectory = turn.endTrajectory().fresh();
        return turn.build();
    }
}