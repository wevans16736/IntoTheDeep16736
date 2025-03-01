package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class LimeSweet {
    Limelight3A lime; HardwareMap hardwareMap; Telemetry telemetry;
    public LimeSweet(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.lime = hardwareMap.get(Limelight3A.class, "limeLight");
        lime.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        lime.start(); // This tells Limelight to start looking!
        lime.pipelineSwitch(1);
    }

    public void scan(){
        LLResult result = lime.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            double distance = 5.0;

            GlobalVariables.X = Math.tan(tx)*distance;
            GlobalVariables.Y = Math.tan(ty)*distance;

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.addData("X inch", GlobalVariables.X);
            telemetry.addData("Y inch", GlobalVariables.Y);
            telemetry.update();
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.update();
        }
    }
    public void stop(){
        lime.stop();
    }
}
