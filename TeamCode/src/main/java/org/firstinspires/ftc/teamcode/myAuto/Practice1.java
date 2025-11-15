package org.firstinspires.ftc.teamcode.myAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Practice1", group = "myAuto")
public class Practice1 extends OpMode {
    public enum State {
        INIT,
        SERVOS,
        SERVOS_MOTOR,
        SERVO1_MOTOR,
        SERVO1_ONLY;
    }
    private DcMotor motor1;
    private DcMotor motor2;
    private Servo servo1;
    private Servo servo2;
    private Gamepad previousGamepad;
    private Gamepad currentGamepad;
    private State robotState;
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        previousGamepad = new Gamepad();
        currentGamepad = new Gamepad();
        robotState = State.INIT;
    }

    public void loop() {
        motor2.setPower(gamepad1.left_stick_y);
        switch(robotState) {
            case INIT:
                servo1.setPosition(0);
                servo2.setPosition(0);
                motor1.setPower(0);
                if(!previousGamepad.x && currentGamepad.x) {
                    robotState = State.SERVOS;
                }
                else if(!previousGamepad.b && currentGamepad.b) {
                    robotState = State.SERVO1_MOTOR;
                }
                else if(!previousGamepad.y && currentGamepad.y) {
                    robotState = State.SERVO1_ONLY;
                }
                break;
            case SERVOS:
                servo1.setPosition(1);
                servo2.setPosition(1);
                motor1.setPower(0);
                if(!previousGamepad.x && currentGamepad.x) {
                    robotState = State.SERVOS_MOTOR;
                }
            case SERVOS_MOTOR:
                servo1.setPosition(1);
                servo2.setPosition(1);
                motor1.setPower(1);
                if(!previousGamepad.a && currentGamepad.a) {
                    robotState = State.SERVO1_MOTOR;
                }
            case SERVO1_MOTOR:
                servo1.setPosition(1);
                servo2.setPosition(0);
                motor1.setPower(1);
                if(!previousGamepad.a && currentGamepad.a) {
                    robotState = State.INIT;
                }
            case SERVO1_ONLY:
                servo1.setPosition(1);
                servo2.setPosition(0);
                motor1.setPower(0);
                if(!previousGamepad.x && currentGamepad.x) {
                    robotState = State.SERVO1_MOTOR;
                }
        }
    }
}
