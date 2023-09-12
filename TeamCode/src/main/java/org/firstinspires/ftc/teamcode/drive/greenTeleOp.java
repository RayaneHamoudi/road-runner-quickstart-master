package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name="greenTeleOp", group="4544") //a person on our team is very gay but we dont know who :)
@Config
public class greenTeleOp extends LinearOpMode {
    // initialize the drivetrain motors

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor BL;
    // initialize the output lift Motors

    private DcMotor liftLeft;
    private DcMotor liftRight;


    // initialize the intake (claw)
    private Servo clawLeft;
    private Servo clawRight;

    public static double kG = 0.05;

    public void runOpMode() throws InterruptedException{

        double ticks = 537.68;
        int halfRot = (int)ticks/2;
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");



        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // clawLeft.setDirection(Servo.Direction.REVERSE);
        /*
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */



        //0 is stationary, 1 is going up, and 2 is going down for the linear slide
        //This is a quick replacement for an enum/FSM
        //int goinUp = 0;

        waitForStart();

        while(opModeIsActive()) {
            //DT
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x / 1.5;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            FL.setPower(v1 * 0.9);
            FR.setPower(v2 * 0.9);
            BL.setPower(v3 * 0.9);
            BR.setPower(v4 * 0.9);
            //This sets a power level to the motors


            if(gamepad1.right_bumper){
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0);
            }
            else{
                clawLeft.setPosition(0);
                clawRight.setPosition(0.5);
            }


            if(gamepad1.left_trigger != 0){
                //Going down
                liftLeft.setPower(-0.9);
                liftRight.setPower(-0.9);
            }
            else if(gamepad1.right_trigger != 0){
                //Going up
                liftLeft.setPower(0.9);
                liftRight.setPower(0.9);
            }
            else{
                //liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                //liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftLeft.setPower(kG);
                liftRight.setPower(kG);
            }

            telemetry.addData("liftLeft = ", liftLeft.getCurrentPosition());
            telemetry.addData("liftRight = ", liftRight.getCurrentPosition());

            telemetry.update();

            //4000
        }
    }
}