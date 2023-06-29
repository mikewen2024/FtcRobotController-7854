package org.firstinspires.ftc.teamcode.TestingAndDemos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
public class EncoderOutputs2 extends LinearOpMode {

    static MecanumDrive drive;
    static Odometry odometry;
    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);

        // Telemetry for encoders ig
//        telemetry.addData("Left", odometry.leftTicks);
//        telemetry.addData("Right", odometry.rightTicks);
//        telemetry.addData("Front", odometry.topTicks);
//
        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());
//
//        telemetry.addData("\nBL", drive.BL.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("BR", drive.BR.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("FL", drive.FL.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("FR", drive.FR.getCurrent(CurrentUnit.AMPS));

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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Init and start procedure
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
        drive = new MecanumDrive(hardwareMap, telemetry);

        waitForStart();

        // Starting odometry thread
        odometry.run();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            logGamepad(telemetry, gamepad1, "gamepad1");
            telemetry.update();

            sleep(20);
        }
    }
}