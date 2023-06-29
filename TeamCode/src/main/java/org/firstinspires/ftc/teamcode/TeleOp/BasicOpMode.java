package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Odometry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.PIDDrive;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.SectionSpline;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Vector2;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/*
Control hub:
Motor Ports:
0. BL, left encoder
1. BR, front encoder
2. FL
3. FR, right encoder
 */

@Config
@TeleOp(name = "Basic OpMode")
public class BasicOpMode extends OpMode {
    // Dashboard Obj
    public static FtcDashboard dashboard;
    TelemetryPacket packet;
    MultipleTelemetry dataDump;
    ElapsedTime timer;

    // Hardware
    public MecanumDrive drive;
    public Odometry odometry;
    public PIDDrive pidDrive;
    public double [][] roots = {
            {0.0, 0.0},
            {10.0, 23.0},
            {0.0, 33.0},
            {-23.0, 23.0}
    };
    public SectionSpline spline;
    public double [] targetState = {0.0, 0.0, 0.0};

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    public double t = 0.1;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        timer.startTime();

        // Drivetrain
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
        pidDrive = new PIDDrive(this.odometry, 0.0, 0.0, - Math.PI, telemetry);
        spline = new SectionSpline(roots, 6.0);

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
        packet = new TelemetryPacket();
        dataDump = new MultipleTelemetry();
    }

    boolean odometryRunning = false;
    double [] PIDOutputs = {0.0, 0.0, 0.0};
    @Override
    public void loop() {
        if(gamepad1.x){
            odometry.resetEncoders();
        }

        if(!(t <= 0.05 && -gamepad1.left_stick_y < 0.0) && !(t >= 0.95 && -gamepad1.left_stick_y > 0.0)){
            t -= gamepad1.left_stick_y * 0.01;
        }
        telemetry.addData("Time t", t);
        telemetry.addData("Tangent x component times 50", 50.0 * spline.tangent[0]);
        telemetry.addData("Tangent y component times 50", 50.0 * spline.tangent[1]);
        this.targetState [0] = spline.getState(t)[0];
        this.targetState [1] = spline.getState(t)[1];
        this.targetState [2] = spline.getTangentAngle(t);

        pidDrive.setTargetState(this.targetState [0], this.targetState [1], this.targetState [2]);
        odometry.updatePosition();
        PIDOutputs = pidDrive.updatePID();
        drive.FieldOrientedDrive(PIDOutputs [0], PIDOutputs [1], PIDOutputs [2], odometry.getRotationRadians(), telemetry);

        // Start running odometry thread as soon as started, no multi threading rn since running into issues
//        if(!odometryRunning){
//            odometry.run();
//            odometryRunning = true;
//        }

        // Handle controller inputs with low pass filter and input into drive

//        if(gamepad1.left_bumper){
//            drive.FieldOrientedDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    odometry.getRotationRadians(), telemetry);
//            telemetry.addLine("Field Oriented");
//        }else{
//            drive.NormalDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    telemetry);
//        }
//
//
//        previousInputs [0] = gamepad1.left_stick_x;
//        previousInputs [1] = gamepad1.left_stick_y;
//        previousInputs [2] = gamepad1.right_stick_x;

//        telemetry();

        // Line following
    }

    public void telemetry(){
//        telemetryPacket.fieldOverlay().setFill("pink").fillRect(0, 0, 1000, 500);
//        telemetryPacket.put("uwu", 20);
//        dashboard.sendTelemetryPacket(packet);

//        logGamepad(telemetry, gamepad1, "Raw gamepad inputs");

        telemetry.update();
    }

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }

            // Adding data:
//            telemetry.addData("Left stick x", gamepad.left_stick_x);
        }
    }
}
