/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoREG", group="Main")

public class AutoReg extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime  = new ElapsedTime();

    private DcMotor motor_left1  = null;
    private DcMotor motor_left2  = null;
    private DcMotor motor_right1 = null;
    private DcMotor motor_right2 = null;
    private DcMotor motor_lift = null;
    private DcMotor motor_intake = null;

    private Servo servo_marker = null;

    private CRServo servo_intake = null;
    private CRServo servo_intake_2= null;


    //Vars
    private double ratio = 0.75;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motor_left1 = hardwareMap.get(DcMotor.class, "ml1");
        motor_left2 = hardwareMap.get(DcMotor.class, "ml2");
        motor_right1 = hardwareMap.get(DcMotor.class, "mr1");
        motor_right2 = hardwareMap.get(DcMotor.class, "mr2");


        motor_left1.setDirection(DcMotor.Direction.FORWARD);
        motor_left2.setDirection(DcMotor.Direction.FORWARD);
        motor_right1.setDirection(DcMotor.Direction.REVERSE);
        motor_right2.setDirection(DcMotor.Direction.REVERSE);


        //motor_thread.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_intake = hardwareMap.get(DcMotor.class,"min");
        motor_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo_intake = hardwareMap.get(CRServo.class,"crsin1");
        servo_intake_2 = hardwareMap.get(CRServo.class, "crsin2");

        motor_lift = hardwareMap.get(DcMotor.class,"mlift");
        motor_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_lift.setTargetPosition(-14440);
        //8820-> 8800

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        waitForStart();
        runtime.reset();


        motor_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motor_lift.isBusy()&& opModeIsActive()){
            motor_lift.setPower(0.5);
        }

        motor_lift.setPower(0);
        runtime.reset();
        while(runtime.time()<0.5){

            setBasePower(-0.5,-0.5);

        }
        setBasePower(0,0);

        //START


        //END

    }
    private void setBasePowerLeft(double power){

        motor_left1.setPower(power);
        motor_left2.setPower(power);

    }
    private void setBasePowerRight(double power){

        motor_right1.setPower(power);
        motor_right2.setPower(power);

    }
    private void setBasePower(double power_left,double power_right){

        setBasePowerLeft(power_left);
        setBasePowerRight(power_right);

    }
    private void moveRobot(){
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        setBasePower(rightPower*0.33,leftPower*0.33);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    private void moveLift() {

        if (gamepad1.dpad_left) {
            motor_lift.setPower(-1);
        } else {
            motor_lift.setPower(0);
        }

        if (gamepad1.dpad_right) {
            motor_lift.setPower(1);
        } else {
            motor_lift.setPower(0);
        }
    }
    private void moveIntake(){

        double trigger_left = gamepad1.left_trigger;
        double trigger_right = gamepad1.right_trigger;
        if(trigger_right > 0.1){
            motor_intake.setPower(trigger_right*ratio);

        }else if(trigger_left > 0.1){
            motor_intake.setPower(-trigger_left*ratio);
        }else{
            motor_intake.setPower(0.0);
        }

//        if(gamepad1.a){
//            servo_intake.setPower(1.0);
//        }else{
//            servo_intake.setPower(0.0);
//        }
//
//        if(gamepad1.b){
//            servo_intake.setPower(-1.0);
//        }else{
//            servo_intake.setPower(0.0);
//        }

        if (gamepad1.a) {
            servo_intake_2.setPower(1);
            servo_intake.setPower(-1);
        }
        if(gamepad1.b){
            servo_intake_2.setPower(-1);
            servo_intake.setPower(1);
        }
        if(gamepad1.x){
            servo_intake_2.setPower(0);
            servo_intake.setPower(0);
        }
    }
    private void UpdateInfo(){
        telemetry.addData("Enc Intake",motor_intake.getCurrentPosition());
        telemetry.addData("Enc Lift",motor_lift.getCurrentPosition());
    }
}
