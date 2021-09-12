package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleopv2", group="Linear Opmode")

public class teleopv2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor intake = null;
    private DcMotor transfer = null;
    private DcMotor shooter = null;
    private DcMotor arm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class,"intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        arm = hardwareMap.get(DcMotor.class, "arm");
        Servo claw1 = hardwareMap.get(Servo.class, "claw1");
        Servo claw2 = hardwareMap.get(Servo.class, "claw2");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");

        boolean intakeMotorStatus = false;
        boolean intakeButtonStatus = false;
        boolean shooterMotorStatus = false;
        boolean shooterButtonStatus = false;

        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double tlpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double trpower   = Range.clip(drive - turn, -1.0, 1.0) ;
            double blpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double brpower    = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.left_stick_x < 0) { //strafe left
                tl.setPower(-0.7);
                tr.setPower(0.7);
                bl.setPower(0.7);
                br.setPower(-0.7);
                continue;
            }
            if (gamepad1.left_stick_x > 0) { //strafe right
                tl.setPower(0.7);
                tr.setPower(-0.7);
                bl.setPower(-0.7);
                br.setPower(0.7);
                continue;
            }
            if (gamepad1.left_trigger == 1 && !intakeButtonStatus) {
                intakeMotorStatus = !intakeMotorStatus;
                intakeButtonStatus = true;
            }
            if (gamepad1.left_trigger == 0) intakeButtonStatus = false;
            intake.setPower(intakeMotorStatus ? 1 : 0);
            transfer.setPower(intakeMotorStatus ? 0.6 : 0); //toggles intake and transfer

            if (gamepad1.y) { //reverses intake and transfer
                intake.setPower(-1);
                transfer.setPower(-0.6);
            }

            if (gamepad1.right_trigger == 1 && !shooterButtonStatus){
                shooterMotorStatus = !shooterMotorStatus;
                shooterButtonStatus = true;
            }
            if (gamepad1.right_trigger == 0) shooterButtonStatus = false;
            shooter.setPower(shooterMotorStatus ? -0.85 : 0); //toggles shooter

            if(gamepad1.right_bumper) { //reverses shooter
                shooter.setPower(0.8);
            }
            if(gamepad1.dpad_up) { //arm up
                arm.setPower(0.8);
            } else arm.setPower(0);

            if(gamepad1.dpad_down){ //arm down
                arm.setPower(-0.5);
            } else arm.setPower(0);

            if(gamepad1.dpad_left){ //claw open
                claw1.setPosition(1);
                claw2.setPosition(0);
            }
            if(gamepad1.dpad_right){ //claw close
                claw1.setPosition(0);
                claw2.setPosition(1);
            }
            if(gamepad1.a){ //shooting mode
                stopper.setPosition(0);
            }
            if(gamepad1.b){ //stopping mode
                stopper.setPosition(1);
            }

            // Send calculated power to wheels
            tl.setPower(tlpower);
            bl.setPower(blpower);
            tr.setPower(trpower);
            br.setPower(brpower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", tlpower, trpower);
            telemetry.update();
        }
    }
}
