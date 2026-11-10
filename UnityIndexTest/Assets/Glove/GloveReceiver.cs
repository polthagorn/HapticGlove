using System;
using System.IO;
using System.IO.Ports;
using System.Threading;
using System.Text;
using Stopwatch = System.Diagnostics.Stopwatch;
using UnityEngine;

// The simple tabbed window and its servo modes live in GloveSimpleUI.cs (same class).
[DefaultExecutionOrder(100)]
public partial class GloveReceiver : MonoBehaviour
{
    [Tooltip("Use the same port shown in Arduino IDE, e.g. COM3 or /dev/cu.usbserial-110.")]
    public string portName = "COM3";
    [Tooltip("Scan every serial port and pick the one sending GLOVE1 packets. portName is tried first.")]
    public bool autoSelectPort = true;
    public int baudRate = 115200;
    public float smoothing = 15f;
    public float timeoutSeconds = 1f;
    public bool showControls = true;
    public float[] openRaw = new float[5];
    public float[] closedRaw = {4095,4095,4095,4095,4095};
    public readonly float[] Curl = new float[5];
    // Preserve the original smoothed animation; contact uses a separate sample.
    public readonly float[] MeasuredCurl = new float[5];
    // Worn early-contact/backdrive was observed; three-point mapping is not validated.
    public readonly int[] Raw = new int[5];
    public bool Connected { get { return lastTick > 0 &&
        (Stopwatch.GetTimestamp()-lastTick)/(double)Stopwatch.Frequency < timeoutSeconds; } }
    public IndexContactSimulation contactSimulation;
    string simulationReply = "Waiting for simulation firmware ACK";
    long simulationReplyTick;
    bool calibrationEnabled;
    bool detachedBench = true;
    bool showManual;
    bool showSphereControls = true;
    bool showSensorCalibration = true;
    Vector2 sensorPanelScroll;
    Vector2 spherePanelScroll;
    Vector2 manualPanelScroll;
    bool driveUsesMouse;
    bool contactDrive, tableConfirmed;
    bool checkingClearance;
    bool checkingContactStop;
    public bool CheckingContactStop { get { return CheckingClearance && checkingContactStop; } }
    int checkTarget;
    float checkLimit;
    public int ClearanceTarget { get { return checkTarget; } }
    public float ClearanceLimit { get { return checkLimit; } }
    public bool CheckingClearance { get { return contactDrive && checkingClearance; } }
    public void FinishClearanceCheck() {
        contactDrive = false;
        checkingClearance = false;
        SubmitHardware("HAP1,STOP");
    }
    int approachRate = 800, releaseRate = 1200, preparationRate = 2000;
    bool rateFirmwareReady;
    string firmwareRateStatus = "Waiting for firmware speed info";
    const string SpeedKey = "HapticGlove.IndexSpeed.V2.";
    string RatePacket { get { return "HAP1,RATE," + approachRate + "," + releaseRate; } }
    string transportStatus = "No servo target sent";
    string hardwareReply = "Not enabled";
    public bool HardwareActive { get { return contactDrive || calibrationEnabled || degreeMode || simpleServo || fistMode; } }
    public bool CanMarkBenchTouch(int target) {
        return Connected && SensorCalibrationValid && contactDrive && driveUsesMouse &&
            hardwareReply == "HAPACK,TARGET," + target;
    }
    bool SensorCalibrationValid {
        get { return openRaw != null && closedRaw != null && openRaw.Length == 5 && closedRaw.Length == 5 &&
            Mathf.Abs(closedRaw[1] - openRaw[1]) >= 50; }
    }
    long lastCalibrationAction;
    int calibrationTarget;
    string calibrationReply = "Manual calibration OFF";
    // True-degree servo calibration (firmware CAL3). Table: degree -> {pulseUs, thumb, index, middle, ring, little}.
    bool degreeMode, degreeSweeping, showDegree;
    float degreeCommandTime;
    int degreeTarget = 90;
    string degreeText = "90", sweepFromText = "0", sweepToText = "180", sweepStepText = "1", sweepSettleText = "300";
    string degreeReply = "Degree mode OFF";
    string degreeStatus = "No sweep yet";
    Vector2 degreePanelScroll;
    readonly System.Collections.Generic.SortedDictionary<int,int[]> degreeTable =
        new System.Collections.Generic.SortedDictionary<int,int[]>();
    string DegreeTablePath { get { return Path.Combine(Application.persistentDataPath, "servo-degree-calibration.csv"); } }
    public bool TryGetDegreeReading(int degree, out int indexRaw) {
        int[] row;
        indexRaw = degreeTable.TryGetValue(degree, out row) ? row[2] : -1;
        return indexRaw >= 0;
    }
    sealed class SerialSession
    {
        public volatile bool stop;
        public readonly object gate = new object();
        public string command = "RELEASE";
        public long commandTick, ackTick;
        public string ack = "Waiting for simulation firmware ACK";
        public long infoTick;
        public string speedInfo = "";
        public string pendingHardware;
        public bool driveContact;
        public int driveTarget = 2050;
        public int approachRate = 800, releaseRate = 1200;
        public string hardwareAck = "Not enabled";
        public long targetAckTick;
        public int sentTarget = -1, ackTarget = -1;
        public long sentTick;
        public double matchedAckMs = -1;
        public string pendingCalibration;
        public bool keepCalibration;
        public string calibrationPrefix = "CAL1,";
        public string calibrationAck = "Manual calibration OFF";
        public string degreeAck = "Degree mode OFF";
        public readonly System.Collections.Generic.List<int[]> sweepRows = new System.Collections.Generic.List<int[]>();
        public bool sweepDone;
        public readonly System.Collections.Generic.List<string> fistLines = new System.Collections.Generic.List<string>();
        public int[] raw;
        public long tick;
        public string status = "Opening USB serial...";
        public string activePort;
        public Thread thread;
    }
    SerialSession session;
    long lastTick;
    string message = "Waiting for ESP32";
    const string Key = "ESP32Glove.v1.";
    readonly string[] names = {"Thumb", "Index", "Middle", "Ring", "Little"};
    int curveFinger = 1;
    int curvePoint = 50;
    int customCurvePercent = 40;
    bool useCustomCurvePoint;
    const string CurveKey = "ESP32Glove.Curve.v1.";

    void OnEnable()
    {
        if (openRaw == null || openRaw.Length != 5) openRaw = new float[5];
        if (closedRaw == null || closedRaw.Length != 5) closedRaw = new float[] {4095,4095,4095,4095,4095};
        for (int i=0;i<5;i++) {
            openRaw[i] = PlayerPrefs.GetFloat(Key+"open"+i, openRaw[i]);
            closedRaw[i] = PlayerPrefs.GetFloat(Key+"closed"+i, closedRaw[i]);
        }
        approachRate = QuantizeRate(PlayerPrefs.GetInt(SpeedKey+"approach",800));
        releaseRate = QuantizeRate(PlayerPrefs.GetInt(SpeedKey+"release",1200));
        preparationRate = QuantizeRate(PlayerPrefs.GetInt(SpeedKey+"prepare",2000));
        lastTick = 0;
        calibrationEnabled = false;
        contactDrive = false;
        checkingClearance = false;
        tableConfirmed = false;
        calibrationTarget = 0;
        degreeMode = degreeSweeping = false;
        simpleServo = fistMode = fistSweeping = false;
        LoadDegreeTable();
        LoadFistTable();
        SerialSession state = new SerialSession();
        session = state;
        string selectedPort = portName;
        int selectedBaud = baudRate;
        bool auto = autoSelectPort;
        state.thread = new Thread(() => ReadSerial(state, selectedPort, selectedBaud, auto));
        state.thread.IsBackground = true;
        state.thread.Name = "ESP32 glove USB";
        state.thread.Start();
    }

    // Listen-only probe: never writes to a port, so other devices (e.g. the Uno) get no commands.
    static string FindGlovePort(SerialSession state, string preferred, int baud)
    {
        string[] ports;
        try { ports = SerialPort.GetPortNames(); }
        catch (Exception) { return null; }
        Array.Sort(ports, StringComparer.OrdinalIgnoreCase);
        int pi = Array.IndexOf(ports, preferred);
        if (pi > 0) { ports[pi] = ports[0]; ports[0] = preferred; }
        if (ports.Length == 0) {
            lock(state.gate) state.status = "Auto port: no serial ports found (plug in ESP32)";
            return null;
        }
        foreach (string candidate in ports) {
            if (state.stop) return null;
            lock(state.gate) state.status = "Auto port: scanning " + candidate + " (" + string.Join(", ", ports) + ")";
            try {
                using (SerialPort probe = new SerialPort(candidate, baud)) {
                    probe.ReadTimeout = 50;
                    probe.DtrEnable = false;
                    probe.RtsEnable = false;
                    probe.Open();
                    StringBuilder line = new StringBuilder(96);
                    long start = Stopwatch.GetTimestamp();
                    while (!state.stop && (Stopwatch.GetTimestamp()-start)/(double)Stopwatch.Frequency < 2.0) {
                        int ch;
                        try { ch = probe.ReadChar(); }
                        catch (TimeoutException) { continue; }
                        if (ch == '\n') {
                            int[] values;
                            if (TryParse(line.ToString(), out values)) return candidate;
                            line.Length = 0;
                        } else if (line.Length < 96) line.Append((char)ch);
                        else line.Length = 0;
                    }
                }
            } catch (Exception) {
                // Busy (Arduino Serial Monitor, Uno link) or not a real device: skip it.
            }
        }
        lock(state.gate) state.status = "Auto port: no ESP32 glove found on " + string.Join(", ", ports) + " (retrying; close Serial Monitor)";
        return null;
    }

    static void ReadSerial(SerialSession state, string configuredPort, int selectedBaud, bool autoSelect)
    {
        while (!state.stop) {
            string selectedPort = configuredPort;
            if (autoSelect) {
                selectedPort = FindGlovePort(state, configuredPort, selectedBaud);
                if (selectedPort == null) {
                    for (int i=0;i<20 && !state.stop;i++) Thread.Sleep(100);
                    continue;
                }
            }
            lock(state.gate) state.activePort = selectedPort;
            try {
                using (SerialPort serial = new SerialPort(selectedPort,selectedBaud)) {
                    serial.ReadTimeout = 5;
                    serial.WriteTimeout = 100;
                    serial.DtrEnable = false;
                    serial.RtsEnable = false;
                    serial.Open();
                    // Release first on every new connection; the worker alone owns RX/TX.
                    serial.Write("HAP1,STOP\nCAL1,STOP\nCAL2,STOP\nCAL3,STOP\nCAL4,STOP\nSIM1,RELEASE\n");
                    lock(state.gate) {
                        state.pendingCalibration = null;
                        state.pendingHardware = null;
                        state.hardwareAck = "HAPACK,OFF";
                        state.infoTick = 0;
                        state.sentTarget = state.ackTarget = -1;
                        state.sentTick = 0; state.matchedAckMs = -1;
                    }
                    int sentApproachRate = -1, sentReleaseRate = -1;
                    long lastInfo = 0;
                    long lastSend = Stopwatch.GetTimestamp();
                    lock(state.gate) { state.ackTick = 0; state.ack = "Waiting for simulation firmware ACK"; }
                    lock(state.gate) state.status = "USB open: " + selectedPort + "; waiting for valid data";
                    StringBuilder line = new StringBuilder(96);
                    bool discard = false;
                    while (!state.stop) {
                        long now = Stopwatch.GetTimestamp();
                        if (lastInfo == 0 || (now-lastInfo)/(double)Stopwatch.Frequency >= 1.0) {
                            serial.Write("HAP1,INFO\n");
                            lastInfo = now;
                        }
                        bool urgent;
                        lock(state.gate) urgent = state.pendingHardware != null || state.pendingCalibration != null ||
                            (state.driveContact && (state.driveTarget != state.sentTarget || state.approachRate != sentApproachRate || state.releaseRate != sentReleaseRate));
                        if (urgent || (now-lastSend)/(double)Stopwatch.Frequency >= 0.02) {
                            string command, calibration, hardware;
                            int requestedApproachRate, requestedReleaseRate;
                            lock(state.gate) {
                                bool freshCommand = state.commandTick > 0 && (now-state.commandTick)/(double)Stopwatch.Frequency < 0.35;
                                bool freshSensors = state.tick > 0 && (now-state.tick)/(double)Stopwatch.Frequency < 1.0;
                                command = freshCommand && freshSensors ? state.command : "RELEASE";
                                calibration = freshCommand && freshSensors ? state.pendingCalibration : "CAL1,STOP\nCAL2,STOP\nCAL3,STOP\nCAL4,STOP";
                                state.pendingCalibration = null;
                                hardware = freshCommand && freshSensors ? state.pendingHardware : "HAP1,STOP";
                                state.pendingHardware = null;
                                requestedApproachRate = state.approachRate; requestedReleaseRate = state.releaseRate;
                                if (hardware == null && state.driveContact && freshCommand && freshSensors)
                                    hardware = "HAP1,TARGET," + state.driveTarget;
                                if (calibration == null && state.keepCalibration && freshCommand && freshSensors)
                                    calibration = state.calibrationPrefix + "KEEP";
                            }
                            serial.Write("SIM1," + command + "\n");
                            if (hardware != null) {
                                int target;
                                if (hardware.StartsWith("HAP1,TARGET,", StringComparison.Ordinal) && int.TryParse(hardware.Substring(12),out target)) {
                                    if (sentApproachRate != requestedApproachRate || sentReleaseRate != requestedReleaseRate) {
                                        serial.Write("HAP1,RATE," + requestedApproachRate + "," + requestedReleaseRate + "\n");
                                        sentApproachRate = requestedApproachRate; sentReleaseRate = requestedReleaseRate;
                                    }
                                    lock(state.gate) {
                                        if (state.sentTarget != target) {
                                            state.sentTarget = target;
                                            state.sentTick = Stopwatch.GetTimestamp();
                                            state.matchedAckMs = -1;
                                        }
                                    }
                                }
                                serial.Write(hardware + "\n");
                            }
                            if (calibration != null) serial.Write(calibration + "\n");
                            lastSend = now;
                        }
                        int ch;
                        try { ch = serial.ReadChar(); }
                        catch (TimeoutException) { continue; }
                        if (ch < 0) continue;
                        if (ch == '\n') {
                            string packet = line.ToString().Trim();
                            if (!discard && ValidSpeedInfo(packet)) {
                                lock(state.gate) { state.speedInfo = packet; state.infoTick = Stopwatch.GetTimestamp(); }
                            }
                            if (!discard && packet.StartsWith("HAPACK,", StringComparison.Ordinal)) {
                                lock(state.gate) {
                                    state.hardwareAck = packet;
                                    int target;
                                    if (packet.StartsWith("HAPACK,TARGET,", StringComparison.Ordinal) && int.TryParse(packet.Substring(14),out target)) {
                                        state.ackTarget = target;
                                        state.targetAckTick = Stopwatch.GetTimestamp();
                                        if (target == state.sentTarget && state.sentTick > 0 && state.matchedAckMs < 0)
                                            state.matchedAckMs = (Stopwatch.GetTimestamp()-state.sentTick)*1000.0/Stopwatch.Frequency;
                                    }
                                }
                            }
                            if (!discard && (packet.StartsWith("CALACK,", StringComparison.Ordinal) || packet.StartsWith("BENCHACK,", StringComparison.Ordinal))) {
                                lock(state.gate) state.calibrationAck = packet;
                            }
                            if (!discard && packet.StartsWith("DEGACK,", StringComparison.Ordinal)) {
                                lock(state.gate) state.degreeAck = packet;
                            }
                            if (!discard && (packet.StartsWith("FIST4,", StringComparison.Ordinal) ||
                                packet.StartsWith("FISTACK,", StringComparison.Ordinal))) {
                                lock(state.gate) state.fistLines.Add(packet);
                            }
                            if (!discard && packet == "SWEEP3,DONE") {
                                lock(state.gate) state.sweepDone = true;
                            } else if (!discard && packet.StartsWith("SWEEP3,", StringComparison.Ordinal)) {
                                string[] parts = packet.Split(',');
                                int[] row = new int[7];
                                bool ok = parts.Length == 8;
                                for (int i=0; ok && i<7; i++) ok = int.TryParse(parts[i+1], out row[i]);
                                if (ok && row[0] >= 0 && row[0] <= 180) lock(state.gate) state.sweepRows.Add(row);
                            }
                            if (!discard && IsSimulationAck(packet)) {
                                lock(state.gate) { state.ack = packet; state.ackTick = Stopwatch.GetTimestamp(); }
                            }
                            int[] values;
                            if (!discard && TryParse(packet,out values)) {
                                lock(state.gate) {
                                    state.raw = values;
                                    state.tick = Stopwatch.GetTimestamp();
                                    state.status = "Receiving USB: " + selectedPort;
                                }
                            }
                            line.Length = 0;
                            discard = false;
                        } else if (!discard) {
                            if (line.Length >= 96) { discard = true; line.Length = 0; }
                            else line.Append((char)ch);
                        }
                    }
                    serial.Write("HAP1,STOP\nCAL1,STOP\nCAL2,STOP\nCAL3,STOP\nCAL4,STOP\nSIM1,RELEASE\n");
                }
            } catch (Exception e) {
                lock(state.gate) {
                    state.status = "USB: " + e.Message + " (retrying)";
                    state.tick = 0;
                    state.ackTick = 0;
                }
            }
            // Responsive cancellation while waiting to reconnect.
            for (int i=0;i<20 && !state.stop;i++) Thread.Sleep(100);
        }
    }

    void Update()
    {
        SerialSession state = session;
        if (state == null) return;
        if (calibrationEnabled && (Stopwatch.GetTimestamp()-lastCalibrationAction)/(double)Stopwatch.Frequency > 10) {
            calibrationEnabled = false;
            SubmitCalibration("CAL1,STOP");
        }
        bool sweepFinished = false;
        UpdateSimpleServo();
        lock(state.gate) {
            fistBuffer.AddRange(state.fistLines);
            state.fistLines.Clear();
            message = state.status;
            // Keep portName in sync so UnoSimulationLink can refuse the ESP32 port.
            if (!string.IsNullOrEmpty(state.activePort)) portName = state.activePort;
            lastTick = state.tick;
            simulationReply = state.ack;
            simulationReplyTick = state.ackTick;
            hardwareReply = state.hardwareAck;
            transportStatus = state.sentTarget < 0 ? "No servo target sent" :
                "Sent " + state.sentTarget + " / ACK " + state.ackTarget +
                (state.matchedAckMs < 0 ? " | waiting" : " | matching ACK after " + state.matchedAckMs.ToString("F1") + " ms");
            rateFirmwareReady = state.infoTick > 0 && (Stopwatch.GetTimestamp()-state.infoTick)/(double)Stopwatch.Frequency < 3;
            firmwareRateStatus = rateFirmwareReady ? state.speedInfo : "Old/unknown firmware: speed control not detected";
            bool canDrive = Connected && tableConfirmed && contactSimulation != null &&
                contactSimulation.isActiveAndEnabled && contactSimulation.CalibrationValid &&
                (contactSimulation.mouseSlider || SensorCalibrationValid) &&
                contactSimulation.mouseSlider == driveUsesMouse;
            if (contactDrive && (!canDrive || hardwareReply == "HAPACK,OFF" ||
                hardwareReply.Contains("ERROR") || hardwareReply.Contains("TIMEOUT"))) {
                contactDrive = false;
                state.pendingHardware = "HAP1,STOP";
            }
            // A delayed HAPACK,OFF from an earlier stop must not cancel a fresh Enable.
            bool simpleGrace = Time.unscaledTime - simpleServoStartTime < 0.5f;
            if (simpleServo && (!Connected || (!simpleGrace && (hardwareReply == "HAPACK,OFF" ||
                hardwareReply.Contains("ERROR") || hardwareReply.Contains("TIMEOUT"))))) {
                simpleServo = false;
                simpleStatus = "Servo stopped: " + (Connected ? hardwareReply : "ESP32 disconnected");
                state.pendingHardware = "HAP1,STOP";
            }
            state.driveContact = contactDrive || simpleServo;
            state.driveTarget = simpleServo ? simpleTarget : CheckingClearance ? checkTarget : contactSimulation != null ? Mathf.Clamp(contactSimulation.HardwareTargetUs, 1000, 2050) : 2050;
            calibrationReply = state.calibrationAck;
            // Ignore OFF acknowledgements here: a delayed BENCHACK,OFF from a
            // previous stop can arrive just after a new ARM and cancel its keepalive.
            if (calibrationReply.Contains("TIMEOUT") || calibrationReply.Contains("ERROR")) {
                calibrationEnabled = false;
            }
            if (!Connected) calibrationEnabled = false;
            degreeReply = state.degreeAck;
            foreach (int[] row in state.sweepRows) {
                degreeTable[row[0]] = new int[] {row[1], row[2], row[3], row[4], row[5], row[6]};
                degreeStatus = "Sweeping: recorded " + row[0] + "° → index pot " + row[3];
            }
            state.sweepRows.Clear();
            sweepFinished = state.sweepDone;
            state.sweepDone = false;
            // A delayed DEGACK,OFF from an earlier stop must not cancel a command just sent.
            bool recentCommand = Time.unscaledTime - degreeCommandTime < 0.5f;
            if (degreeMode && (!Connected || (!recentCommand && (degreeReply == "DEGACK,OFF" ||
                degreeReply.Contains("ERROR") || degreeReply.Contains("REJECT"))))) {
                if (degreeSweeping) degreeStatus = "Sweep stopped: " + (Connected ? degreeReply : "ESP32 disconnected");
                degreeMode = degreeSweeping = false;
            }
            if (degreeMode && degreeReply.StartsWith("Pending", StringComparison.Ordinal) &&
                Time.unscaledTime - degreeCommandTime > 1.5f) {
                degreeStatus = "No DEGACK from ESP32: upload the new firmware (CAL3)";
                degreeMode = degreeSweeping = false;
            }
            state.keepCalibration = calibrationEnabled || degreeMode || fistMode;
            state.calibrationPrefix = degreeMode ? "CAL3," : fistMode ? "CAL4," : detachedBench ? "CAL2," : "CAL1,";
            state.command = contactSimulation != null ? contactSimulation.SimulatedCommand : "RELEASE";
            state.commandTick = Stopwatch.GetTimestamp();
            if (state.raw != null) Array.Copy(state.raw,Raw,5);
        }
        ProcessFistLines();
        if (sweepFinished) {
            degreeSweeping = false;
            StopDegree();
            degreeStatus = SaveDegreeTable() ? "Sweep done: " + degreeTable.Count + " points saved to " + DegreeTablePath
                : "Sweep done but CSV save failed (table kept in memory)";
        }
        if (!Connected) return; // Hold last pose on disconnect, with a visible warning.
        float blend = 1f - Mathf.Exp(-Mathf.Max(0,smoothing)*Time.unscaledDeltaTime);
        for (int i=0;i<5;i++) {
            float target = CalibratedCurl(i, Raw[i]);
            MeasuredCurl[i] = target;
            Curl[i] = Mathf.Lerp(Curl[i],target,blend);
        }
    }

    void LateUpdate()
    {
        SerialSession state = session;
        if (state == null) return;
        lock(state.gate) {
            state.driveTarget = simpleServo ? simpleTarget : CheckingClearance ? checkTarget : contactSimulation != null ? Mathf.Clamp(contactSimulation.HardwareTargetUs,1000,2050) : 2050;
            bool preparing = simpleServo ? false : CheckingClearance ? !checkingContactStop : contactSimulation != null && contactSimulation.IsPreparing;
            state.approachRate = preparing ? preparationRate : approachRate;
            state.releaseRate = preparing ? preparationRate : releaseRate;
            state.command = contactSimulation != null ? contactSimulation.SimulatedCommand : "RELEASE";
            state.commandTick = Stopwatch.GetTimestamp();
        }
    }

    public static bool IsSimulationAck(string packet)
    {
        if (packet == "ACK1,RELEASE" || packet == "ACK1,RELEASE_TIMEOUT") return true;
        const string prefix = "ACK1,LIMIT,";
        float curl;
        return packet.StartsWith(prefix, StringComparison.Ordinal) &&
            float.TryParse(packet.Substring(prefix.Length), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out curl) &&
            !float.IsNaN(curl) && !float.IsInfinity(curl) && curl >= 0 && curl <= 1;
    }

    public static bool TryParse(string packet,out int[] values)
    {
        values = null;
        string[] parts = packet.Trim().Split(',');
        if (parts.Length!=6 || parts[0]!="GLOVE1") return false;
        int[] parsed = new int[5];
        for (int i=0;i<5;i++)
            if (!int.TryParse(parts[i+1],out parsed[i]) || parsed[i]<0 || parsed[i]>4095) return false;
        values = parsed;
        return true;
    }

    public void Capture(bool closed)
    {
        if (!Connected) return;
        for (int i=0;i<5;i++) {
            if (closed) closedRaw[i]=Raw[i]; else openRaw[i]=Raw[i];
            PlayerPrefs.SetFloat(Key+(closed?"closed":"open")+i,Raw[i]);
        }
        PlayerPrefs.Save();
    }

    float CalibratedCurl(int finger, int raw)
    {
        float span = closedRaw[finger] - openRaw[finger];
        if (Mathf.Abs(span) < 50) return 0;
        float previousRaw = openRaw[finger], previousCurl = 0;
        int customPercent = PlayerPrefs.GetInt(CurveKey + finger + ".customPercent", -1);
        for (int point = 1; point < 100; point++) {
            string key = point == customPercent
                ? CurveKey + finger + ".customRaw"
                : (point == 25 || point == 50 || point == 75 ? CurveKey + finger + "." + point : null);
            if (key == null || !PlayerPrefs.HasKey(key)) continue;
            float nextRaw = PlayerPrefs.GetFloat(key), nextCurl = point / 100f;
            if ((span > 0 && raw <= nextRaw) || (span < 0 && raw >= nextRaw))
                return Mathf.Clamp01(Mathf.Lerp(previousCurl, nextCurl,
                    Mathf.InverseLerp(previousRaw, nextRaw, raw)));
            previousRaw = nextRaw; previousCurl = nextCurl;
        }
        return Mathf.Clamp01(Mathf.Lerp(previousCurl, 1f,
            Mathf.InverseLerp(previousRaw, closedRaw[finger], raw)));
    }

    void CaptureCurvePoint()
    {
        if (!Connected) return;
        int selectedPercent = useCustomCurvePoint ? customCurvePercent : curvePoint;
        if (selectedPercent == 0 || selectedPercent == 100) {
            bool closed = selectedPercent == 100;
            if (closed) closedRaw[curveFinger] = Raw[curveFinger];
            else openRaw[curveFinger] = Raw[curveFinger];
            PlayerPrefs.SetFloat(Key + (closed ? "closed" : "open") + curveFinger, Raw[curveFinger]);
        } else if (useCustomCurvePoint) {
            PlayerPrefs.SetInt(CurveKey + curveFinger + ".customPercent", selectedPercent);
            PlayerPrefs.SetFloat(CurveKey + curveFinger + ".customRaw", Raw[curveFinger]);
        } else {
            PlayerPrefs.SetFloat(CurveKey + curveFinger + "." + selectedPercent, Raw[curveFinger]);
        }
        PlayerPrefs.Save();
    }

    void ClearCurve(int finger)
    {
        for (int point = 25; point <= 75; point += 25)
            PlayerPrefs.DeleteKey(CurveKey + finger + "." + point);
        PlayerPrefs.DeleteKey(CurveKey + finger + ".customPercent");
        PlayerPrefs.DeleteKey(CurveKey + finger + ".customRaw");
        PlayerPrefs.Save();
    }

    void SubmitCalibration(string packet)
    {
        if (session == null) return;
        lastCalibrationAction = Stopwatch.GetTimestamp();
        lock(session.gate) {
            if (detachedBench) packet = packet.Replace("CAL1,", "CAL2,");
            session.pendingCalibration = packet;
            session.calibrationAck = "Pending: " + packet;
        }
    }

    void SubmitHardware(string packet)
    {
        if (session == null) return;
        lock(session.gate) {
            session.pendingHardware = packet;
            session.hardwareAck = "Pending: " + packet;
        }
    }

    void StepCalibration(int delta)
    {
        calibrationTarget = Mathf.Clamp(calibrationTarget + delta, detachedBench ? 1000 : 1470, detachedBench ? 2050 : 1500);
        SubmitCalibration("CAL1,TARGET," + calibrationTarget);
    }

    bool CanUseDegree { get { return Connected && !contactDrive && !calibrationEnabled && !simpleServo && !fistMode; } }

    // Must match DEG0_US / DEG180_US / degToUs() in the firmware.
    const int Deg0Us = 500, Deg180Us = 2500;
    static int DegreeToUs(int degree) { return Deg0Us + ((Deg180Us - Deg0Us) * degree + 90) / 180; }
    static string DegreeRangeLabel(string text) {
        int degree;
        if (!int.TryParse(text, out degree)) return "?";
        int us = DegreeToUs(Mathf.Clamp(degree, 0, 180));
        return degree + "° = " + us + " us" + (us < 1000 || us > 2050 ? " (OUTSIDE tested band!)" : "");
    }

    void SubmitDegree(string packet)
    {
        if (session == null) return;
        degreeMode = true;
        degreeCommandTime = Time.unscaledTime;
        lock(session.gate) {
            session.pendingCalibration = packet;
            session.degreeAck = "Pending: " + packet;
            session.calibrationPrefix = "CAL3,";
            session.keepCalibration = true;
        }
    }

    void MoveToDegree(int degree)
    {
        if (!CanUseDegree) return;
        degreeTarget = Mathf.Clamp(degree, 0, 180);
        degreeText = degreeTarget.ToString();
        degreeSweeping = false;
        SubmitDegree("CAL3,DEG," + degreeTarget);
    }

    void StartDegreeSweep()
    {
        int from, to, step, settle;
        if (!int.TryParse(sweepFromText, out from) || !int.TryParse(sweepToText, out to) ||
            !int.TryParse(sweepStepText, out step) || !int.TryParse(sweepSettleText, out settle)) {
            degreeStatus = "Sweep fields must be whole numbers"; return;
        }
        from = Mathf.Clamp(from, 0, 180); to = Mathf.Clamp(to, 0, 180);
        step = Mathf.Clamp(step, 1, 90); settle = Mathf.Clamp(settle, 50, 5000);
        sweepFromText = from.ToString(); sweepToText = to.ToString();
        sweepStepText = step.ToString(); sweepSettleText = settle.ToString();
        if (!CanUseDegree) return;
        degreeSweeping = true;
        degreeStatus = "Sweeping " + from + "° → " + to + "° every " + step + "°";
        SubmitDegree("CAL3,SWEEP," + from + "," + to + "," + step + "," + settle);
    }

    void StopDegree()
    {
        if (degreeSweeping) degreeStatus = "Sweep stopped by user (" + degreeTable.Count + " points kept)";
        degreeMode = degreeSweeping = false;
        if (session == null) return;
        lock(session.gate) {
            session.keepCalibration = calibrationEnabled;
            session.pendingCalibration = "CAL3,STOP";
        }
    }

    // Manual move: store the live pot reading at the current degree.
    void RecordCurrentDegree()
    {
        int us = 0;
        string[] parts = degreeReply.Split(',');
        if (parts.Length == 4 && parts[1] == "TARGET") int.TryParse(parts[3], out us);
        degreeTable[degreeTarget] = new int[] {us, Raw[0], Raw[1], Raw[2], Raw[3], Raw[4]};
        degreeStatus = SaveDegreeTable() ? "Recorded " + degreeTarget + "° → index pot " + Raw[1] : "Recorded (CSV save failed)";
    }

    bool SaveDegreeTable()
    {
        var csv = new StringBuilder("degree,pulse_us,thumb,index,middle,ring,little\n");
        foreach (var kv in degreeTable)
            csv.Append(kv.Key).Append(',').Append(string.Join(",", kv.Value)).Append('\n');
        try { File.WriteAllText(DegreeTablePath, csv.ToString()); return true; }
        catch (Exception e) { Debug.LogWarning("Degree table save failed: " + e.Message); return false; }
    }

    void LoadDegreeTable()
    {
        degreeTable.Clear();
        try {
            if (!File.Exists(DegreeTablePath)) return;
            foreach (string text in File.ReadAllLines(DegreeTablePath)) {
                string[] parts = text.Split(',');
                int degree;
                if (parts.Length != 7 || !int.TryParse(parts[0], out degree) || degree < 0 || degree > 180) continue;
                int[] row = new int[6];
                bool ok = true;
                for (int i=0; ok && i<6; i++) ok = int.TryParse(parts[i+1], out row[i]);
                if (ok) degreeTable[degree] = row;
            }
            if (degreeTable.Count > 0) degreeStatus = "Loaded " + degreeTable.Count + " saved points";
        } catch (Exception e) { Debug.LogWarning("Degree table load failed: " + e.Message); }
    }

    static int QuantizeRate(int value) { return Mathf.Clamp(Mathf.RoundToInt(value / 50f) * 50,100,2000); }
    static bool ValidSpeedInfo(string packet) {
        string[] parts = packet.Split(',');
        int a,b;
        return parts.Length == 4 && parts[0] == "HAPINFO" && parts[1] == "2" &&
            int.TryParse(parts[2],out a) && int.TryParse(parts[3],out b) &&
            a >= 100 && a <= 2000 && b >= 100 && b <= 2000 && a % 50 == 0 && b % 50 == 0;
    }
    void SaveSpeed() {
        PlayerPrefs.SetInt(SpeedKey+"approach",approachRate);
        PlayerPrefs.SetInt(SpeedKey+"release",releaseRate);
        PlayerPrefs.SetInt(SpeedKey+"prepare",preparationRate);
        PlayerPrefs.Save();
    }

    void StartStopCheck(bool contactStop) {
        checkLimit = contactSimulation.SurfaceLimit;
        checkingContactStop = contactStop;
        checkTarget = contactStop ? contactSimulation.BenchContactTargetUs : contactSimulation.FirstTouchTestTarget;
        contactSimulation.mouseSlider = true;
        contactSimulation.testCurl = 0;
        tableConfirmed = driveUsesMouse = checkingClearance = contactDrive = true;
        SubmitHardware(RatePacket + "\nHAP1,ARM");
    }

    string EnableBlockReason()
    {
        if (!Connected) return "Waiting: ESP32 must show LIVE";
        if (!rateFirmwareReady) return "Upload speed-control firmware once; then adjust speed here";
        if (!tableConfirmed) return "Tick: Index arm travel clear; release control within reach";
        if (calibrationEnabled) return "Stop manual calibration first";
        if (degreeMode) return "Stop servo degree calibration first";
        if (simpleServo || fistMode) return "Stop the simple-view servo first";
        if (contactDrive) return "Sphere control is already enabled";
        if (contactSimulation == null || !contactSimulation.isActiveAndEnabled) return "Index contact component is unavailable";
        if (!contactSimulation.CalibrationValid) return "Index servo table is invalid";
        if (!contactSimulation.mouseSlider && !SensorCalibrationValid)
            return "Index sensor OPEN/CLOSED range is below 50 counts";
        if (!contactSimulation.mouseSlider && MeasuredCurl[1] > 0.05f)
            return "Open index before enabling control";
        return "";
    }

    void StopSphereControl() {
        contactDrive = false;
        tableConfirmed = false;
        if (session == null) return;
        lock(session.gate) {
            session.driveContact = false;
            session.driveTarget = 2050;
            session.pendingHardware = "HAP1,STOP";
        }
    }

    void OnGUI()
    {
        if (HardwareActive && Event.current.type == EventType.KeyDown &&
            (Event.current.keyCode == KeyCode.Space || Event.current.keyCode == KeyCode.Escape)) {
            StopAllServo("Stopped by keyboard");
            Event.current.Use();
            return;
        }
        if (!showControls) return;
        if (!ShowAdvancedPanels) { DrawSimpleWindow(); return; }
        if (GUI.Button(new Rect(Screen.width - 212, Screen.height - 44, 200, 32), "Back to simple view")) {
            ShowAdvancedPanels = false; return;
        }
        if (contactDrive && Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Space) {
            StopSphereControl();
            Event.current.Use();
        }
        if (!showControls) return;
        GUILayout.BeginArea(new Rect(410,12,440, showManual ? 330 : 40), GUI.skin.box);
        showManual = GUILayout.Toggle(showManual, "Manual index calibration (advanced / not worn)");
        if (showManual) {
            manualPanelScroll = GUILayout.BeginScrollView(manualPanelScroll);
            detachedBench = true;
            GUILayout.Label("GPIO17 | 1000–2050 us | no change to horn alignment");
            GUILayout.Label(calibrationReply);
            GUI.enabled = Connected && !HardwareActive;
            if (GUILayout.Button("START at 2000 us")) {
                calibrationTarget = 2000; calibrationEnabled = true; SubmitCalibration("CAL1,ARM2000");
            }
            if (GUILayout.Button("START at 1000 us (fully open mechanism only)")) {
                calibrationTarget = 1000; calibrationEnabled = true; SubmitCalibration("CAL1,ARM");
            }
            GUI.enabled = Connected && calibrationEnabled && !contactDrive;
            GUILayout.Label(calibrationTarget == 0 ? "No target sent" : "Requested: " + calibrationTarget + " us (not measured)");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("-10")) StepCalibration(-10);
            if (GUILayout.Button("+10")) StepCalibration(10);
            GUILayout.EndHorizontal();
            if (calibrationTarget < 2000 && GUILayout.Button("+50")) StepCalibration(50);
            GUI.enabled = true;
            if (GUILayout.Button("STOP manual pulses (does not release mechanism)")) {
                calibrationEnabled = false; SubmitCalibration("CAL1,STOP");
            }
            GUILayout.EndScrollView();
        }
        GUI.enabled = true;
        GUILayout.EndArea();
        float sphereTop = showManual ? 355 : 65;
        GUILayout.BeginArea(new Rect(410,sphereTop,440,showSphereControls ? Mathf.Max(180,Screen.height-sphereTop-12) : 40), GUI.skin.box);
        showSphereControls = GUILayout.Toggle(showSphereControls, showSphereControls ? "⌄ SPHERE → INDEX SERVO" : "› SPHERE → INDEX SERVO");
        if (showSphereControls) {
        spherePanelScroll = GUILayout.BeginScrollView(spherePanelScroll);
        GUI.enabled = !contactDrive;
        tableConfirmed = GUILayout.Toggle(tableConfirmed, "Index arm travel clear; release control within reach");
        if (contactSimulation != null) {
            bool mouse = GUILayout.Toggle(contactSimulation.mouseSlider, "Use mouse slider (OFF = ESP32 index sensor)");
            if (!contactDrive) contactSimulation.mouseSlider = mouse;
            GUILayout.Label("Input: " + (contactSimulation.mouseSlider ? "Mouse slider" : "ESP32 index sensor"));
        }
        GUI.enabled = !contactDrive && !calibrationEnabled;
        GUILayout.Label("Prepare: " + preparationRate + " us/s");
        preparationRate = QuantizeRate(Mathf.RoundToInt(GUILayout.HorizontalSlider(preparationRate,100,2000)));
        GUILayout.Label("Contact approach: " + approachRate + " us/s");
        approachRate = QuantizeRate(Mathf.RoundToInt(GUILayout.HorizontalSlider(approachRate,100,2000)));
        GUILayout.Label("Release: " + releaseRate + " us/s");
        releaseRate = QuantizeRate(Mathf.RoundToInt(GUILayout.HorizontalSlider(releaseRate,100,2000)));
        if (GUILayout.Button("Save speed (applied on Enable)")) SaveSpeed();
        GUI.enabled = true;
        GUILayout.Label(firmwareRateStatus);
        GUI.enabled = Connected && rateFirmwareReady && !HardwareActive && SensorCalibrationValid &&
            MeasuredCurl[1] <= 0.05f && contactSimulation != null && contactSimulation.CanCheckClearance;
        if (GUILayout.Button("Start index clearance check (glove on table, NOT worn)")) StartStopCheck(false);
        if (GUILayout.Button("Check CONTACT stop (glove on table, NOT worn)")) StartStopCheck(true);
        GUI.enabled = true;
        if (CheckingClearance) GUILayout.Label("Hold target: " + checkTarget + " us. Wait for horn to settle, then mark FIRST touch.");
        string blockReason = EnableBlockReason();
        GUI.enabled = blockReason.Length == 0;
        if (GUILayout.Button(contactSimulation != null && !contactSimulation.mouseSlider ? "Enable continuous index contact" : "Enable sphere control — start at 2050 us")) {
            checkingClearance = false;
            driveUsesMouse = contactSimulation.mouseSlider;
            if (!driveUsesMouse) contactSimulation.prepareAhead = true;
            contactDrive = true;
            SaveSpeed();
            SubmitHardware(RatePacket + "\nHAP1,ARM");
        }
        GUI.enabled = true;
        GUILayout.Label(blockReason.Length == 0 ? "Ready (sensor or mouse input)" : blockReason);
        if (GUILayout.Button("Release to 2050 and stop (Space)")) StopSphereControl();
        GUILayout.Label(hardwareReply);
        GUILayout.Label(transportStatus);
        GUILayout.Label("ACK confirms receipt, NOT motor position or completion");
        GUILayout.Label("Index control: prepare / hold / prepare. Input change releases.\nSpeed sliders apply when Enable is pressed.");
        GUILayout.EndScrollView();
        }
        GUILayout.EndArea();
        GUILayout.BeginArea(new Rect(12,12,390,310),GUI.skin.box);
        sensorPanelScroll = GUILayout.BeginScrollView(sensorPanelScroll);
        GUILayout.Label("ESP32 USB glove | " + (Connected ? "LIVE" : "DISCONNECTED / WAITING"));
        GUILayout.Label(message);
        bool ackFresh = simulationReplyTick > 0 &&
            (Stopwatch.GetTimestamp()-simulationReplyTick)/(double)Stopwatch.Frequency < 1.0;
        GUILayout.Label("ESP32 SIM ONLY: " + (ackFresh ? simulationReply : "No fresh ACK (upload simulation firmware)"));
        showSensorCalibration = GUILayout.Toggle(showSensorCalibration, "Sensor calibration and readings");
        if (showSensorCalibration) {
        GUI.enabled = Connected;
        if (GUILayout.Button("1. Hold hand OPEN, then click")) Capture(false);
        if (GUILayout.Button("2. Hold hand CLOSED, then click")) Capture(true);
        GUI.enabled = true;
        for (int i=0;i<5;i++) {
            string warning = Mathf.Abs(closedRaw[i]-openRaw[i])<50 ? " (recalibrate: range too small)" : "";
            GUILayout.Label(names[i]+": "+Raw[i]+" | "+Mathf.RoundToInt(Curl[i]*100)+"%"+warning);
        }
        GUILayout.Label("One sensor per finger estimates joint bending.\nHand position/rotation is fixed.");
        DrawCurveControls();
        }
        GUILayout.EndScrollView();
        GUILayout.EndArea();
        DrawDegreePanel();
    }

    // Old nonlinear finger-curve capture, shared by the old panel and the Tracking tab.
    void DrawCurveControls()
    {
        GUILayout.Label("Optional nonlinear sensor curve (hold the selected finger at the shown curl)");
        GUILayout.BeginHorizontal();
        for (int i=0;i<5;i++) if (GUILayout.Toggle(curveFinger == i, names[i], "Button")) curveFinger = i;
        GUILayout.EndHorizontal();
        GUILayout.BeginHorizontal();
        foreach (int point in new int[] {0,25,50,75,100})
            if (GUILayout.Toggle(!useCustomCurvePoint && curvePoint == point, point + "%", "Button")) {
                curvePoint = point; useCustomCurvePoint = false;
            }
        if (GUILayout.Toggle(useCustomCurvePoint, "Custom", "Button")) useCustomCurvePoint = true;
        GUILayout.EndHorizontal();
        if (useCustomCurvePoint) {
            GUILayout.Label("Custom curl: " + customCurvePercent + "%");
            customCurvePercent = Mathf.Clamp(Mathf.RoundToInt(GUILayout.HorizontalSlider(customCurvePercent,1,99)),1,99);
        }
        GUI.enabled = Connected;
        int selectedPercent = useCustomCurvePoint ? customCurvePercent : curvePoint;
        string pointLabel = selectedPercent == 0 ? "OPEN" : selectedPercent == 100 ? "CLOSED" : selectedPercent + "% curl";
        if (GUILayout.Button("Capture " + names[curveFinger] + " at " + pointLabel + " (Raw " + Raw[curveFinger] + ")")) CaptureCurvePoint();
        if (GUILayout.Button("Clear " + names[curveFinger] + " intermediate points")) ClearCurve(curveFinger);
        GUI.enabled = true;
    }

    void DrawDegreePanel()
    {
        // Own column right of the 410..850 panels; IndexContactSimulation owns (12,330) on the left.
        float top = 12, left = Mathf.Max(860, Screen.width - 452);
        GUILayout.BeginArea(new Rect(left, top, 440, showDegree ? Mathf.Max(200, Screen.height - top - 12) : 40), GUI.skin.box);
        showDegree = GUILayout.Toggle(showDegree, (showDegree ? "⌄" : "›") + " SERVO DEGREE CALIBRATION 0–180°");
        if (showDegree) {
            degreePanelScroll = GUILayout.BeginScrollView(degreePanelScroll);
            GUILayout.Label("True degrees: 0° = 500 us, 180° = 2500 us (DEG0_US/DEG180_US in firmware). NOT worn.\n" +
                "Horn connected: keep From/To inside 45°–139° (1000–2044 us, the tested band).\n" +
                "Full 0–180° only with the horn OFF (servo-only check; the pot will not track).");
            GUILayout.Label(degreeReply);
            GUI.enabled = CanUseDegree && !degreeSweeping;
            GUILayout.BeginHorizontal();
            GUILayout.Label("Degree", GUILayout.Width(50));
            degreeText = GUILayout.TextField(degreeText, 3, GUILayout.Width(45));
            int typed;
            if (GUILayout.Button("Move") && int.TryParse(degreeText, out typed)) MoveToDegree(typed);
            if (GUILayout.Button("-10")) MoveToDegree(degreeTarget - 10);
            if (GUILayout.Button("-1")) MoveToDegree(degreeTarget - 1);
            if (GUILayout.Button("+1")) MoveToDegree(degreeTarget + 1);
            if (GUILayout.Button("+10")) MoveToDegree(degreeTarget + 10);
            GUILayout.EndHorizontal();
            GUI.enabled = degreeMode && !degreeSweeping;
            if (GUILayout.Button("Record pot at " + degreeTarget + "° (index now " + Raw[1] + ")")) RecordCurrentDegree();
            GUI.enabled = CanUseDegree && !degreeSweeping;
            GUILayout.BeginHorizontal();
            GUILayout.Label("From", GUILayout.Width(34)); sweepFromText = GUILayout.TextField(sweepFromText, 3, GUILayout.Width(36));
            GUILayout.Label("To", GUILayout.Width(20)); sweepToText = GUILayout.TextField(sweepToText, 3, GUILayout.Width(36));
            GUILayout.Label("Step", GUILayout.Width(32)); sweepStepText = GUILayout.TextField(sweepStepText, 2, GUILayout.Width(28));
            GUILayout.Label("Settle ms", GUILayout.Width(60)); sweepSettleText = GUILayout.TextField(sweepSettleText, 4, GUILayout.Width(40));
            GUILayout.EndHorizontal();
            GUILayout.Label("Sweep range: " + DegreeRangeLabel(sweepFromText) + " → " + DegreeRangeLabel(sweepToText));
            if (GUILayout.Button("Sweep and record potentiometer at every step")) StartDegreeSweep();
            GUI.enabled = true;
            if (GUILayout.Button("STOP servo (Space / Esc)")) StopDegree();
            GUILayout.Label(degreeStatus);
            GUI.enabled = degreeTable.Count > 0 && !degreeSweeping;
            if (GUILayout.Button("Clear saved table")) { degreeTable.Clear(); SaveDegreeTable(); degreeStatus = "Table cleared"; }
            GUI.enabled = true;
            GUILayout.Label("deg | pulse us | INDEX pot | thumb middle ring little");
            foreach (var kv in degreeTable) {
                int[] r = kv.Value;
                GUILayout.Label(kv.Key + "° | " + r[0] + " | " + r[2] + " | " + r[1] + " " + r[3] + " " + r[4] + " " + r[5]);
            }
            GUILayout.EndScrollView();
        }
        GUILayout.EndArea();
    }

    void OnDisable()
    {
        SerialSession state = session;
        session = null;
        lastTick = 0;
        if (state == null) return;
        state.stop = true;
        // Worker owns and disposes its port, including on disconnect/errors.
        if (state.thread != null) state.thread.Join(500);
    }
}
