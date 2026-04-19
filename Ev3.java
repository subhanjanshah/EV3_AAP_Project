package src;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Ev3 {

    public static float lineValue = 0;
    public static float distance = 1.0f; 
    public static int status = 1; // 1 = Follow, 2 = Avoid

    public static void main(String[] args) {
        DataThread reader = new DataThread();
        reader.setDaemon(true); 
        reader.start();

        Controller brain = new Controller();
        brain.runLogic();
    }
}

class DataThread extends Thread {
    public void run() {
        EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);
        EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S2);
        
        SampleProvider colorGet = color.getRedMode();
        SampleProvider ultraGet = ultra.getDistanceMode();
        
        float[] cSample = new float[colorGet.sampleSize()];
        float[] uSample = new float[ultraGet.sampleSize()];

        while (true) {
            colorGet.fetchSample(cSample, 0);
            ultraGet.fetchSample(uSample, 0);
            
            test1.lineValue = cSample[0];
            test1.distance = uSample[0];
            
            // Trigger Avoidance only if we are currently following the line
            if (test1.distance < 0.15 && test1.status == 1) {
                test1.status = 2;
            }

            try { Thread.sleep(20); } catch (Exception e) {} // Faster polling for PID
        }
    }
}

class Controller {
    // PID memory variables
    private float integral = 0;
    private float lastError = 0;

    public void runLogic() {
        while (!Button.ESCAPE.isDown()) {
            if (test1.status == 1) {
                followLine();
            } else if (test1.status == 2) {
                avoidObstacle();
            }
        }
    }

    private void followLine() {
        // 1. Off-Track Safety Stop (pure white)
        if (test1.lineValue >= 0.95) {
            Motor.A.stop(true);
            Motor.B.stop(false);
            LCD.drawString("Off Track! Stop.", 0, 4);
            integral = 0; 
            lastError = 0;
            return; 
        }

        // 2. The PID Controller
        float target = 0.5f; 
        float error = target - test1.lineValue; 

        // CRITICAL BUG FIX: The Edge Inverter
        // If your robot spins OFF the line immediately, change this to false!
        boolean followRightEdge = true; 
        if (!followRightEdge) {
            error = -error; 
        }

        integral = (integral + error) * 0.5f; 
        float derivative = error - lastError;

        // Smoothed out PID Constants (Less aggressive so it stops spinning out)
        float Kp = 400.0f;  
        float Ki = 2.0f;    
        float Kd = 250.0f;  

        float turn = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;

        // Motor Power Application
        int baseSpeed = 250; // Dropped base speed for testing stability

        int speedA = (int)(baseSpeed + turn);
        int speedB = (int)(baseSpeed - turn);

        if (speedA < 0) speedA = 0;
        if (speedB < 0) speedB = 0;
        if (speedA > 600) speedA = 600;
        if (speedB > 600) speedB = 600;

        Motor.A.setSpeed(speedA);
        Motor.B.setSpeed(speedB);
        
        Motor.A.forward();
        Motor.B.forward();
        
        LCD.drawString("Tracking Line...", 0, 4);
    }

    private void avoidObstacle() {
        LCD.clear();
        LCD.drawString("Object Detected!", 0, 2);
        Motor.A.stop(true);
        Motor.B.stop(false);
        Delay.msDelay(500); 
        
        // Step 1: Turn Left to Check
        Motor.A.rotate(-200, true); 
        Motor.B.rotate(200, false);
        Delay.msDelay(400); // Give ultrasonic sensor a moment to poll

        if (test1.distance > 0.20) {
            // Path is clear. Go straight past the object.
            LCD.drawString("Left Clear!     ", 0, 3);
            bypassStraight();
        } else {
            // Step 2: Object is still there. Move back to center.
            Motor.A.rotate(200, true);
            Motor.B.rotate(-200, false);
            Delay.msDelay(400);

            // Step 3: Turn Right to Check
            Motor.A.rotate(200, true);
            Motor.B.rotate(-200, false);
            Delay.msDelay(400);

            if (test1.distance > 0.20) {
                // Path is clear. Go around.
                LCD.drawString("Right Clear!    ", 0, 3);
                bypassStraight(); 
            } else {
                // Blocked on all sides
                LCD.drawString("Blocked in!     ", 0, 3);
                Motor.A.stop(true);
                Motor.B.stop(false);
                Delay.msDelay(2000); // Wait and hope it moves
            }
        }
        
        // Reset PID memory before returning to line follow to prevent sudden jerks
        integral = 0;
        lastError = 0;
        test1.status = 1; 
        LCD.clear();
    }

    private void bypassStraight() {
        // Drives straight to bypass the object. 
        Motor.A.setSpeed(300);
        Motor.B.setSpeed(300);
        
        // Rotate wheels 800 degrees forward
        Motor.A.rotate(800, true);
        Motor.B.rotate(800, false);
        
        // IMPORTANT: Once it finishes going straight, you will need to manually 
        // angle it back toward the line so the sensors can catch the track again!
    }
}
