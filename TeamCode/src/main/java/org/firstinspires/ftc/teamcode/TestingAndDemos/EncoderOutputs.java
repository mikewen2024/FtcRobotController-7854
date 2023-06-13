package org.firstinspires.ftc.teamcode.TestingAndDemos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Odometry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Vector2;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/*
 * Test the dashboard gamepad integration.
 * Demo for data output to dashbaord in graphs
 */
@Autonomous
public class EncoderOutputs extends LinearOpMode {

    MecanumDrive drive;
    Odometry odometry;

    private void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);

        // Telemetry for encoders ig
        telemetry.addData("Left", odometry.leftTicks);
        telemetry.addData("Right", odometry.rightTicks);
        telemetry.addData("Front", odometry.topTicks);

        telemetry.addData("x", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());


        for (Field field : gamepad.getClass().getFields()) {


            if (Modifier.isStatic(field.getModifiers())) continue;

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            drive.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            drive.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            drive.FL.setVelocity(2.0);
//            drive.FR.setVelocity(2.0);
//            drive.BL.setVelocity(2.0);
//            drive.BR.setVelocity(2.0);


//
//            drive.FL.setPower(0.1);
//            drive.FR.setPower(0.1);
//            drive.BL.setPower(0.1);
//            drive.BR.setPower(0.1);




            odometry.updatePosition();
            odometry.updateTime();

            logGamepad(telemetry, gamepad1, "gamepad1");
//            logGamepad(telemetry, gamepad2, "gamepad2");

            double bx = odometry.getXCoordinate();
            double by = odometry.getYCoordinate();
            double l = 5;

            double[] bxPoints = { l, -l, -l, l };
            double[] byPoints = { l, l, -l, -l };
            rotatePoints(bxPoints, byPoints, odometry.getRotationRadians());
            for (int i = 0; i < 4; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .fillPolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();

            sleep(20);
        }

    }
    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }
}