package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "LimeLightTest", group = "LimeLight")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Set up limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        //start polling data
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
//                Pose3D botpose = result.getBotpose();
//                double captureLatency = result.getCaptureLatency();
//                double targetingLatency = result.getTargetingLatency();
//                double parseLatency = result.getParseLatency();
//                telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                telemetry.addData("Parse Latency", parseLatency);
//                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

//                    telemetry.addData("Botpose", botpose.toString());

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                } else {
                    telemetry.addData("Limelight", "No data available");
                }
                telemetry.update();
                telemetry.clearAll();
            }
        }
    }
}
