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

package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import workspace.Bot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Stone Delivery", group="Linear Opmode")

public class StoneDelivery extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor arm = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    private Bot bot = null;
    
    
    private void leftWheels(double spd){
        leftDrive_0.setPower(spd);
        leftDrive_1.setPower(spd);
    }
    
    private void rightWheels(double spd){
        rightDrive_0.setPower(spd);
        rightDrive_1.setPower(spd);
    }

    @Override
    public void runOpMode() {
        double speed = 6.45/4;  // it took 6.45 sec to go 4 tiles at .5 spd
        double turn90 = 1.41;
        //bot = Bot(hardwareMap, telemetry);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward 1.5 tiles
        leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5*speed)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Lower the Arm and close claw
        leftWheels(0.0);
        rightWheels(0.0);
        runtime.reset();
        arm.setPower(-0.25);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Arm Lowered");
        }
        claw1.setPosition(0.75);
        claw2.setPosition(0.75);
        arm.setPower(0.0);

        // Step 3:  Drive bacwards 1.0 tiles
        leftWheels(-1.0);
        rightWheels(-1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1*speed)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
        // Step 4:  Turn right 90 degrees
        leftWheels(1.0);
        rightWheels(-1.0);
        
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < turn90)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 5:  Move forward 3.3 tiles
        leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.3*speed)) {
            telemetry.addData("Path", "Leg 5: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        
        // Step 6:  Turn left 90 degrees
        leftWheels(-1.0);
        rightWheels(1.0);
        arm.setPower(0.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < turn90)) {
            //telemetry.addData("Path", "Leg 6: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0.0);
        
        //Step 7: Forward 0.44 tiles
        leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < (0.45*speed))) {
            telemetry.update();
        }
        

        // Step 8:  Reverse.
        //leftWheels(-1.0);
        //rightWheels(-1.0);
        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < (3*speed))) {         
            //telemetry.addData("Path", "Complete");
            //telemetry.update();
        //}
        //leftWheels(0.0);
        //rightWheels(0.0);
        //sleep(1000);
    }
}
