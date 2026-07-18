package frc.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MacroManager {
    private static MacroManager instance;

    public static MacroManager getInstance() {
        if (instance == null) {
            instance = new MacroManager();
        }
        return instance;
    }

    public static class MacroFrame {
        public double timestamp;
        public double vx;
        public double vy;
        public double omega;
        public boolean buttonB;
        public boolean buttonY;
        public boolean buttonLB;
        public boolean buttonRB;
        public boolean buttonLT;
        public boolean buttonRT;
        public boolean buttonBack;

        public MacroFrame(double timestamp, double vx, double vy, double omega,
                          boolean buttonB, boolean buttonY, boolean buttonLB,
                          boolean buttonRB, boolean buttonLT, boolean buttonRT,
                          boolean buttonBack) {
            this.timestamp = timestamp;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            this.buttonB = buttonB;
            this.buttonY = buttonY;
            this.buttonLB = buttonLB;
            this.buttonRB = buttonRB;
            this.buttonLT = buttonLT;
            this.buttonRT = buttonRT;
            this.buttonBack = buttonBack;
        }

        @Override
        public String toString() {
            return String.format("%.3f,%.3f,%.3f,%.3f,%b,%b,%b,%b,%b,%b,%b",
                timestamp, vx, vy, omega, buttonB, buttonY, buttonLB, buttonRB, buttonLT, buttonRT, buttonBack);
        }

        public static MacroFrame fromCSV(String line) {
            String[] parts = line.split(",");
            if (parts.length < 11) return null;
            try {
                double timestamp = Double.parseDouble(parts[0]);
                double vx = Double.parseDouble(parts[1]);
                double vy = Double.parseDouble(parts[2]);
                double omega = Double.parseDouble(parts[3]);
                boolean buttonB = Boolean.parseBoolean(parts[4]);
                boolean buttonY = Boolean.parseBoolean(parts[5]);
                boolean buttonLB = Boolean.parseBoolean(parts[6]);
                boolean buttonRB = Boolean.parseBoolean(parts[7]);
                boolean buttonLT = Boolean.parseBoolean(parts[8]);
                boolean buttonRT = Boolean.parseBoolean(parts[9]);
                boolean buttonBack = Boolean.parseBoolean(parts[10]);
                return new MacroFrame(timestamp, vx, vy, omega, buttonB, buttonY, buttonLB, buttonRB, buttonLT, buttonRT, buttonBack);
            } catch (NumberFormatException e) {
                return null;
            }
        }
    }

    private final List<MacroFrame> recordedFrames = new ArrayList<>();
    private boolean isRecording = false;
    private double recordingStartTime = 0;
    private final File recordingFile;

    // Detection for Start + Back hold to toggle
    private double comboHoldStartTime = -1;
    private static final double COMBO_HOLD_DURATION = 1.0; // 1 second
    private boolean comboTriggeredThisPress = false;

    private MacroManager() {
        String dir = Filesystem.getOperatingDirectory().getPath();
        recordingFile = new File(dir + "/recorded_macro.csv");
    }

    public boolean isRecording() {
        return isRecording;
    }

    public void startRecording() {
        if (isRecording) return;
        recordedFrames.clear();
        isRecording = true;
        recordingStartTime = Timer.getFPGATimestamp();
        System.out.println("----- MACRO RECORDING STARTED -----");
    }

    public void stopRecording() {
        if (!isRecording) return;
        isRecording = false;
        System.out.println("----- MACRO RECORDING STOPPED -----");
        saveToFile();
    }

    public void update() {
        RobotContainer rc = Robot.getInstance();
        if (rc == null || rc.gamepad == null) return;

        // Check if both Start and Back are held down
        boolean startPressed = rc.gamepad.start().getAsBoolean();
        boolean backPressed = rc.gamepad.back().getAsBoolean();

        if (startPressed && backPressed) {
            if (comboHoldStartTime == -1) {
                comboHoldStartTime = Timer.getFPGATimestamp();
                comboTriggeredThisPress = false;
            } else if (!comboTriggeredThisPress && (Timer.getFPGATimestamp() - comboHoldStartTime >= COMBO_HOLD_DURATION)) {
                // Trigger toggle
                if (isRecording) {
                    stopRecording();
                } else {
                    startRecording();
                }
                comboTriggeredThisPress = true;
            }
        } else {
            comboHoldStartTime = -1;
            comboTriggeredThisPress = false;
        }

        // If recording, log the current frame
        if (isRecording) {
            if (!edu.wpi.first.wpilibj.RobotState.isTeleop()) {
                stopRecording();
                return;
            }
            double elapsed = Timer.getFPGATimestamp() - recordingStartTime;
            if (elapsed > 120.0) { // Limit to 2 minutes
                stopRecording();
                return;
            }

            double vx = rc.getTeleopVx();
            double vy = rc.getTeleopVy();
            double omega = rc.getTeleopOmega();

            boolean buttonB = rc.gamepad.b().getAsBoolean();
            boolean buttonY = rc.gamepad.y().getAsBoolean();
            boolean buttonLB = rc.gamepad.leftBumper().getAsBoolean();
            boolean buttonRB = rc.gamepad.rightBumper().getAsBoolean();
            boolean buttonLT = rc.gamepad.leftTrigger().getAsBoolean();
            boolean buttonRT = rc.gamepad.rightTrigger().getAsBoolean();
            boolean buttonBack = rc.gamepad.back().getAsBoolean();

            recordedFrames.add(new MacroFrame(elapsed, vx, vy, omega, buttonB, buttonY, buttonLB, buttonRB, buttonLT, buttonRT, buttonBack));
        }

        SmartDashboard.putBoolean("Macro Recording Active", isRecording);
        SmartDashboard.putNumber("Macro Frame Count", recordedFrames.size());
    }

    private void saveToFile() {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(recordingFile))) {
            writer.write("timestamp,vx,vy,omega,buttonB,buttonY,buttonLB,buttonRB,buttonLT,buttonRT,buttonBack\n");
            for (MacroFrame frame : recordedFrames) {
                writer.write(frame.toString() + "\n");
            }
            System.out.println("Macro saved successfully to: " + recordingFile.getAbsolutePath());
        } catch (IOException e) {
            System.err.println("Failed to save macro to file: " + e.getMessage());
        }
    }

    public List<MacroFrame> loadFromFile() {
        List<MacroFrame> frames = new ArrayList<>();
        if (!recordingFile.exists()) {
            System.err.println("No recorded macro file found at: " + recordingFile.getAbsolutePath());
            return frames;
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(recordingFile))) {
            String line = reader.readLine(); // Header
            while ((line = reader.readLine()) != null) {
                MacroFrame frame = MacroFrame.fromCSV(line);
                if (frame != null) {
                    frames.add(frame);
                }
            }
            System.out.println("Macro loaded successfully from: " + recordingFile.getAbsolutePath() + " (" + frames.size() + " frames)");
        } catch (IOException e) {
            System.err.println("Failed to load macro from file: " + e.getMessage());
        }
        return frames;
    }
}
