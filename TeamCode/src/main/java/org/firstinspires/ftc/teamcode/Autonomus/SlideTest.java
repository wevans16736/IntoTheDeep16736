package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalSlideActions;
@Config
@TeleOp(name="SlideTest", group = "SecondRobot")
public class SlideTest extends LinearOpMode {
    public static int horizontalTick = 320;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx horizontalSlide = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_ARM);
        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setVelocity(2000);
        horizontalSlide.setTargetPosition(0);

        DcMotorEx verticalSlide = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();
        while(opModeIsActive()) {
            horizontalSlide.setTargetPosition(horizontalTick);
            telemetry.addData("vertical Slide Position", verticalSlide.getCurrentPosition());
            telemetry.addData("horizontal Slide Position", horizontalSlide.getCurrentPosition());

            dashboardTelemetry.addData("vertical Slide Position", verticalSlide.getCurrentPosition());
            dashboardTelemetry.addData("horizontal Slide Position", horizontalSlide.getCurrentPosition());

            dashboardTelemetry.update();
            telemetry.update();
        }

    }
 }