using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

// Simple tabbed window for GloveReceiver (same class, see GloveReceiver.cs).
//   1 Finger tracking   - OPEN/CLOSED capture + the old optional curve points
//   2 Ball + servo      - ENABLE SERVO: the arm goes straight to the ball's stop point
//                         and follows the ball sliders immediately (firmware HAP mode)
//   3 Servo calibration - worn fist sweep (firmware CAL4): measures, on your hand, where
//                         the arm stops the finger at each pulse; the ball then uses it
// The old panels are still available under "Advanced / old panels".
public partial class GloveReceiver
{
    public static bool ShowAdvancedPanels;

    // --- Tab 2: Enable servo ---
    bool simpleServo;
    float simpleServoStartTime;
    const int SimpleReleaseUs = 2500; // 180 deg: arm rest / release in tab 2 (must match firmware CONTACT_RELEASE_US)
    int simpleTarget = SimpleReleaseUs;
    string simpleStatus = "Servo OFF";

    // --- Tab 3: fist sweep ---
    bool fistMode, fistSweeping;
    float fistCommandTime;
    float lastFistCurl = -1;
    string fistReply = "";
    string fistStatus = "Not run yet";
    // Arm mounting (2026-11-12): 0 deg = 500 us = arm just touching the pot reel; 2500 us = furthest away.
    // Must match firmware MIN_US / MAX_US.
    const int ServoMinUs = 500, ServoMaxUs = 2500;
    string fistStepText = "20", fistSettleText = "250", fistToText = "500";
    float fistMinCurl = 0.7f;         // "make a fist" check before starting (slider in tab 3)
    const float FistJumpCurl = 0.20f; // finger jumped open between two steps = user relaxed
    readonly List<string> fistBuffer = new List<string>();
    // pulse us -> {index mean, min, max, stable}
    SortedDictionary<int,int[]> fistTable = new SortedDictionary<int,int[]>();
    SortedDictionary<int,int[]> fistNew = new SortedDictionary<int,int[]>();
    float[] fistCurveUs, fistCurveCurl;
    float fistCurveTime = -1;
    string FistTablePath { get { return Path.Combine(Application.persistentDataPath, "servo-fist-calibration-v2.csv"); } }

    // --- window ---
    int simpleTab;
    Vector2 simpleScroll;
    bool showCurve, showSpeed, showFistTable;
    GUIStyle titleStyle, bigButton, boldLabel, wrapLabel;

    // ================= servo logic =================

    void UpdateSimpleServo()
    {
        // Simple view: the virtual finger follows the real index sensor.
        if (!ShowAdvancedPanels && contactSimulation != null && !HardwareActive) contactSimulation.mouseSlider = false;
        if (!simpleServo) { simpleTarget = SimpleReleaseUs; return; }
        string why;
        int desired = BallStopUs(out why);
        // Straight to the ball's stop in both directions, at the Move to ball / Release speeds.
        // Only limit: the servo range (BallStopUs clamps to ServoMinUs..ServoMaxUs).
        simpleTarget = desired;
        simpleStatus = why;
    }

    int BallStopUs(out string why)
    {
        if (contactSimulation == null || !contactSimulation.BallOnPath) {
            why = "Ball is not on the finger path: arm released (180°, " + SimpleReleaseUs + " us)";
            return SimpleReleaseUs;
        }
        float limit = contactSimulation.SurfaceLimit;
        string pct = Pct(limit);
        int us;
        string source;
        if (FistCurveReady()) {
            us = FistTableUs(limit);
            if (us < 0) {
                why = "Ball at " + pct + " is outside your calibrated range (" + FistRangeLabel() + "): arm released";
                return SimpleReleaseUs;
            }
            source = "your hand calibration";
        } else {
            // The old bench table was measured with the arm on its previous spline tooth,
            // so it no longer matches. A tab 3 sweep is required.
            why = "No servo calibration for the new arm position yet: run tab 3 first. Arm released";
            return SimpleReleaseUs;
        }
        us = Mathf.Clamp(us, ServoMinUs, ServoMaxUs);
        why = "Arm at " + us + " us → finger stops at " + pct + "  (" + source + ")";
        return us;
    }

    string SimpleEnableBlock()
    {
        if (!Connected) return "Waiting for the ESP32 (must show LIVE)";
        if (!rateFirmwareReady) return "Upload the new firmware to the ESP32 first";
        if (!SensorCalibrationValid) return "Do tab 1 Finger tracking first (OPEN and CLOSED)";
        if (contactSimulation == null || !contactSimulation.isActiveAndEnabled) return "Index contact simulation is missing";
        if (HardwareActive && !simpleServo) return "Another servo mode is running: press STOP first";
        return "";
    }

    void SetSimpleServo(bool on)
    {
        if (on == simpleServo) return;
        if (on) {
            if (SimpleEnableBlock().Length > 0) return;
            simpleServo = true;
            simpleServoStartTime = Time.unscaledTime;
            simpleTarget = SimpleReleaseUs;
            contactSimulation.mouseSlider = false;
            simpleStatus = "Starting…";
            SaveSpeed();
            SubmitHardware(RatePacket + "\nHAP1,ARM");
        } else {
            simpleServo = false;
            simpleTarget = SimpleReleaseUs;
            simpleStatus = "Servo OFF (arm released to 180°). It only runs while tab 2 is open.";
            if (session != null) lock(session.gate) { session.driveContact = false; session.driveTarget = SimpleReleaseUs; }
            SubmitHardware("HAP1,STOP");
        }
    }

    void StopAllServo(string reason)
    {
        if (simpleServo) SetSimpleServo(false);
        if (fistMode || fistSweeping) StopFist(reason);
        if (degreeMode) StopDegree();
        if (contactDrive) StopSphereControl();
        if (calibrationEnabled) { calibrationEnabled = false; SubmitCalibration("CAL1,STOP"); }
    }

    // ================= fist sweep =================

    void FistLimits(out int lo, out int hi)
    {
        // Pot ends (0 / max) always stop the sweep. Your OPEN reading also stops it,
        // so the arm never pushes the finger past fully open.
        lo = 5; hi = 4090;
        float o = openRaw[1], c = closedRaw[1];
        int guard = Mathf.RoundToInt(Mathf.Abs(c - o) * 0.02f);
        if (c > o) lo = Mathf.Max(lo, Mathf.RoundToInt(o) + guard);
        else hi = Mathf.Min(hi, Mathf.RoundToInt(o) - guard);
    }

    string FistBlock()
    {
        if (!Connected) return "Waiting for the ESP32 (must show LIVE)";
        if (!SensorCalibrationValid) return "Do tab 1 Finger tracking first (OPEN and CLOSED)";
        if (HardwareActive) return "Another servo mode is running: press STOP first";
        if (MeasuredCurl[1] < fistMinCurl) return "Make a fist: index must read " + Pct(fistMinCurl) + " or more (now " + Pct(MeasuredCurl[1]) + ")";
        int lo, hi;
        FistLimits(out lo, out hi);
        if (Raw[1] <= lo || Raw[1] >= hi) return "Index pot is at its end (raw " + Raw[1] + "): adjust the pot";
        return "";
    }

    void SubmitFist(string packet)
    {
        if (session == null) return;
        fistCommandTime = Time.unscaledTime;
        lock(session.gate) {
            session.pendingCalibration = packet;
            session.calibrationPrefix = "CAL4,";
            session.keepCalibration = true;
        }
    }

    void StartFistSweep()
    {
        int step, settle, to;
        if (!int.TryParse(fistStepText, out step) || !int.TryParse(fistSettleText, out settle) ||
            !int.TryParse(fistToText, out to)) { fistStatus = "Settings must be whole numbers"; return; }
        step = Mathf.Clamp(step, 5, 200); settle = Mathf.Clamp(settle, 50, 3000); to = Mathf.Clamp(to, ServoMinUs, ServoMaxUs - 50);
        fistStepText = step.ToString(); fistSettleText = settle.ToString(); fistToText = to.ToString();
        if (FistBlock().Length > 0) return;
        int lo, hi;
        FistLimits(out lo, out hi);
        fistNew = new SortedDictionary<int,int[]>();
        lastFistCurl = -1;
        fistReply = "";
        fistMode = fistSweeping = true;
        fistStatus = "Sweeping… keep pushing your fist against the arm";
        SubmitFist("CAL4,SWEEP," + ServoMaxUs + "," + to + "," + step + "," + settle + "," + lo + "," + hi);
    }

    void StopFist(string reason)
    {
        if (session != null) lock(session.gate) session.pendingCalibration = "CAL4,STOP";
        EndFistSweep(reason);
        fistMode = false;
    }

    // Only a sweep that finished (DONE) or reached the pot/open limit replaces the saved
    // table. User stop, relax, disconnect or refusal keeps the previous table.
    void EndFistSweep(string reason) { EndFistSweep(reason, false); }

    void EndFistSweep(string reason, bool complete)
    {
        if (!fistSweeping) return;
        fistSweeping = false;
        if (!complete) {
            fistNew = new SortedDictionary<int,int[]>();
            fistStatus = reason + ". Previous table kept.";
            return;
        }
        if (fistNew.Count >= 2) {
            fistTable = fistNew;
            fistNew = new SortedDictionary<int,int[]>();
            fistCurveTime = -1;
            fistStatus = reason + ". " + fistTable.Count + " points saved" + (SaveFistTable() ? "" : " (CSV save failed)") +
                ". The ball in tab 2 now uses this.";
        } else {
            fistStatus = reason + ". Not enough points; the previous table is kept.";
        }
    }

    void ProcessFistLines()
    {
        foreach (string line in fistBuffer) {
            string[] p = line.Split(',');
            if (p[0] == "FISTACK") {
                fistReply = line;
                bool recent = Time.unscaledTime - fistCommandTime < 0.5f;
                if (fistSweeping && p.Length >= 2 && p[1] == "SWEEP")
                    fistStatus = "Arm is moving in toward your finger… keep your fist";
                if (fistSweeping && p.Length == 3 && p[1] == "CONTACT")
                    fistStatus = "Arm reached your finger at " + p[2] + " us. Measuring step by step, keep pushing";
                if (line.Contains("ERROR") || line.Contains("REJECT")) {
                    EndFistSweep(line.Contains("BUSY")
                        ? "The arm is still returning to rest. Wait a second and press START again"
                        : "ESP32 refused: " + line);
                    fistMode = false;
                } else if (line == "FISTACK,OFF" && !recent) {
                    EndFistSweep("Sweep ended");
                    fistMode = false;
                }
                continue;
            }
            if (p.Length >= 2 && p[1] == "DONE") { EndFistSweep("Done", true); continue; }
            if (p.Length == 4 && p[1] == "LIMIT") {
                EndFistSweep("Finished at the limit: index pot read " + p[3] + " at " + p[2] + " us (finger fully open or pot at its end)", true);
                continue;
            }
            int[] v = new int[5];
            bool ok = p.Length == 6;
            for (int i = 0; ok && i < 5; i++) ok = int.TryParse(p[i + 1], out v[i]);
            if (!ok || !fistSweeping) continue;
            fistNew[v[0]] = new int[] {v[1], v[2], v[3], v[4]};
            float curl = CalibratedCurl(1, v[1]);
            fistStatus = "Recording: " + v[0] + " us → index " + Pct(curl) + "  (" + fistNew.Count + " points)" +
                (v[4] == 1 ? "" : "  – unsteady, keep the fist still");
            if (lastFistCurl >= 0 && lastFistCurl - curl > FistJumpCurl) {
                fistNew.Remove(v[0]);
                StopFist("Stopped: your finger jumped open (relaxed?). Keep pushing and try again");
            }
            lastFistCurl = curl;
        }
        fistBuffer.Clear();
        if (fistMode && !Connected) { EndFistSweep("Stopped: ESP32 disconnected"); fistMode = false; }
        if (fistMode && fistReply.Length == 0 && Time.unscaledTime - fistCommandTime > 1.5f) {
            fistSweeping = false; fistMode = false;
            fistStatus = "No reply from the ESP32: upload the new firmware (CAL4)";
        }
    }

    void RefreshFistCurve()
    {
        if (fistCurveTime >= 0 && Time.unscaledTime - fistCurveTime < 0.5f) return;
        fistCurveTime = Time.unscaledTime;
        var us = new List<float>();
        var curl = new List<float>();
        foreach (var kv in fistTable) { us.Add(kv.Key); curl.Add(CalibratedCurl(1, kv.Value[0])); }
        fistCurveUs = us.ToArray();
        fistCurveCurl = curl.ToArray();
    }

    bool FistCurveReady()
    {
        RefreshFistCurve();
        return SensorCalibrationValid && fistCurveUs != null && fistCurveUs.Length >= 2;
    }

    // Pulse that stops the finger at this curl. Scans from the most open arm position,
    // so the first match is the least restrictive one.
    int FistTableUs(float target)
    {
        for (int i = fistCurveUs.Length - 1; i > 0; i--) {
            float c1 = fistCurveCurl[i], c2 = fistCurveCurl[i - 1];
            if (target > Mathf.Max(c1, c2) || target < Mathf.Min(c1, c2)) continue;
            if (Mathf.Approximately(c1, c2)) return Mathf.RoundToInt(fistCurveUs[i]);
            return Mathf.RoundToInt(Mathf.Lerp(fistCurveUs[i], fistCurveUs[i - 1], (target - c1) / (c2 - c1)));
        }
        return -1;
    }

    string FistRangeLabel()
    {
        RefreshFistCurve();
        if (fistCurveCurl == null || fistCurveCurl.Length == 0) return "none";
        float lo = 1, hi = 0;
        foreach (float c in fistCurveCurl) { lo = Mathf.Min(lo, c); hi = Mathf.Max(hi, c); }
        return Pct(lo) + "–" + Pct(hi);
    }

    bool SaveFistTable()
    {
        var csv = new StringBuilder("pulse_us,index_mean,index_min,index_max,stable\n");
        foreach (var kv in fistTable) csv.Append(kv.Key).Append(',').Append(string.Join(",", kv.Value)).Append('\n');
        try { File.WriteAllText(FistTablePath, csv.ToString()); return true; }
        catch (Exception e) { Debug.LogWarning("Fist table save failed: " + e.Message); return false; }
    }

    void LoadFistTable()
    {
        fistMinCurl = PlayerPrefs.GetFloat("HapticGlove.FistMinCurl", 0.7f);        fistTable = new SortedDictionary<int,int[]>();
        fistCurveTime = -1;
        try {
            if (!File.Exists(FistTablePath)) return;
            foreach (string text in File.ReadAllLines(FistTablePath)) {
                string[] p = text.Split(',');
                int us;
                if (p.Length != 5 || !int.TryParse(p[0], out us) || us < ServoMinUs || us > ServoMaxUs) continue;
                int[] row = new int[4];
                bool ok = true;
                for (int i = 0; ok && i < 4; i++) ok = int.TryParse(p[i + 1], out row[i]);
                if (ok) fistTable[us] = row;
            }
            if (fistTable.Count > 0) fistStatus = "Loaded your saved table (" + fistTable.Count + " points)";
        } catch (Exception e) { Debug.LogWarning("Fist table load failed: " + e.Message); }
    }

    // ================= window =================

    static string Pct(float curl) { return Mathf.RoundToInt(100f * curl) + "%"; }

    void EnsureStyles()
    {
        if (titleStyle != null) return;
        titleStyle = new GUIStyle(GUI.skin.label) { fontSize = 18, fontStyle = FontStyle.Bold };
        boldLabel = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = 14 };
        wrapLabel = new GUIStyle(GUI.skin.label) { wordWrap = true };
        bigButton = new GUIStyle(GUI.skin.button) { fontSize = 15, fontStyle = FontStyle.Bold, fixedHeight = 38 };
    }

    void Bar(float value, Color color, float marker)
    {
        Rect r = GUILayoutUtility.GetRect(100, 16, GUILayout.ExpandWidth(true));
        GUI.Box(r, GUIContent.none);
        Color old = GUI.color;
        GUI.color = color;
        GUI.DrawTexture(new Rect(r.x + 1, r.y + 1, (r.width - 2) * Mathf.Clamp01(value), r.height - 2), Texture2D.whiteTexture);
        if (marker >= 0) {
            GUI.color = new Color(1f, 0.45f, 0.1f);
            GUI.DrawTexture(new Rect(r.x + (r.width - 2) * Mathf.Clamp01(marker) - 1, r.y - 3, 4, r.height + 6), Texture2D.whiteTexture);
        }
        GUI.color = old;
    }

    bool ColoredButton(string text, Color color, bool enabled)
    {
        bool wasEnabled = GUI.enabled;
        Color old = GUI.backgroundColor;
        GUI.enabled = wasEnabled && enabled;
        GUI.backgroundColor = color;
        bool clicked = GUILayout.Button(text, bigButton);
        GUI.backgroundColor = old;
        GUI.enabled = wasEnabled;
        return clicked;
    }

    void DrawSimpleWindow()
    {
        EnsureStyles();
        float width = Mathf.Min(500, Screen.width - 24);
        GUILayout.BeginArea(new Rect(12, 12, width, Screen.height - 24), GUI.skin.box);
        GUILayout.Label("HAPTIC GLOVE — index finger", titleStyle);
        GUILayout.Label((Connected ? "● LIVE   " : "○ Waiting for ESP32   ") + message, wrapLabel);
        // Clicks that change which controls exist end this GUI event (ExitGUI) so the
        // layout pass and the click pass never disagree.
        if (HardwareActive && ColoredButton("■  STOP SERVO   (Space)", new Color(1f, 0.3f, 0.3f), true)) {
            StopAllServo("Stopped by user"); GUIUtility.ExitGUI();
        }
        int newTab = GUILayout.Toolbar(simpleTab, new string[] {"1 Tracking", "2 Ball + servo", "3 Calibration", "4 Manual servo"},
            GUILayout.Height(30));
        if (newTab != simpleTab) {
            // The ball servo only runs on tab 2 and manual control only on tab 4. Leaving
            // them releases the arm, so the finger is free for capture and the fist sweep.
            if (simpleServo) SetSimpleServo(false);
            if (degreeMode) StopDegree();
            simpleTab = newTab;
            GUIUtility.ExitGUI();
        }
        GUILayout.Space(6);
        simpleScroll = GUILayout.BeginScrollView(simpleScroll);
        if (simpleTab == 0) DrawTrackingTab();
        else if (simpleTab == 1) DrawBallTab();
        else if (simpleTab == 2) DrawFistTab();
        else DrawManualTab();
        GUILayout.EndScrollView();
        GUI.enabled = !HardwareActive;
        if (GUILayout.Button("Advanced / old panels")) ShowAdvancedPanels = true;
        GUI.enabled = true;
        GUILayout.EndArea();
    }

    // --- Tab 4: manual servo (firmware CAL3 degree mode, true degrees) ---
    // New mounting: 0° = arm just touching the pot reel (the real limit), 180° = furthest away.
    bool showManualTable;
    float manualSlider = 90;
    int ManualMin { get { return 0; } }
    int ManualMax { get { return 180; } }

    void ManualMove(int degree)
    {
        degree = Mathf.Clamp(degree, ManualMin, ManualMax);
        manualSlider = degree;
        MoveToDegree(degree);
    }

    void DrawManualTab()
    {
        GUILayout.Label("Pick the servo angle yourself.", boldLabel);
        GUILayout.Label("0° = 500 us, 180° = 2500 us. The arm moves there at about 45°/s and holds.\n" +
            "Leaving this tab, STOP or Space releases it.", wrapLabel);
        GUILayout.Label("0° = arm just touching the pot reel (its limit). Higher = further from the reel.", wrapLabel);
        GUILayout.Space(6);

        bool can = CanUseDegree && !degreeSweeping;
        string block = !Connected ? "Waiting for the ESP32 (must show LIVE)" :
            !can ? "Another servo mode is running: press STOP first" : "";
        GUI.enabled = can;
        GUILayout.Label(degreeTarget + "°   (" + DegreeToUs(degreeTarget) + " us)" +
            (degreeMode ? "" : "   – servo released"), titleStyle);
        GUILayout.BeginHorizontal();
        GUILayout.Label(ManualMin + "°", GUILayout.Width(32));
        float before = manualSlider;
        manualSlider = GUILayout.HorizontalSlider(manualSlider, ManualMin, ManualMax);
        GUILayout.Label(ManualMax + "°", GUILayout.Width(36));
        GUILayout.EndHorizontal();
        int dragged = Mathf.RoundToInt(manualSlider);
        // Dragging moves the servo right away (one command per whole degree).
        if (can && manualSlider != before && dragged != degreeTarget) ManualMove(dragged);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Type degree", GUILayout.Width(80));
        degreeText = GUILayout.TextField(degreeText, 3, GUILayout.Width(45));
        int typed;
        if (GUILayout.Button("Go", GUILayout.Width(50)) && int.TryParse(degreeText, out typed)) ManualMove(typed);
        if (GUILayout.Button("-10")) ManualMove(degreeTarget - 10);
        if (GUILayout.Button("-1")) ManualMove(degreeTarget - 1);
        if (GUILayout.Button("+1")) ManualMove(degreeTarget + 1);
        if (GUILayout.Button("+10")) ManualMove(degreeTarget + 10);
        GUILayout.EndHorizontal();
        GUI.enabled = true;
        GUILayout.Label(block.Length > 0 ? block : "ESP32: " + degreeReply, wrapLabel);
        GUILayout.Space(6);

        GUILayout.Label("Index pot now: raw " + Raw[1] + "   (" + Pct(MeasuredCurl[1]) + " curl)", boldLabel);
        Bar(MeasuredCurl[1], new Color(0.3f, 0.8f, 1f), -1);
        GUI.enabled = degreeMode;
        if (GUILayout.Button("Record pot at " + degreeTarget + "°")) { RecordCurrentDegree(); GUIUtility.ExitGUI(); }
        GUI.enabled = degreeMode;
        if (GUILayout.Button("Release servo")) { StopDegree(); GUIUtility.ExitGUI(); }
        GUI.enabled = true;
        GUILayout.Label(degreeStatus, wrapLabel);
        if (degreeTable.Count > 0) {
            showManualTable = GUILayout.Toggle(showManualTable, "Show recorded points (" + degreeTable.Count + ")");
            if (showManualTable) {
                GUILayout.Label("deg | pulse us | index pot");
                foreach (var kv in degreeTable)
                    GUILayout.Label(kv.Key + "° | " + kv.Value[0] + " | " + kv.Value[2]);
            }
        }
    }

    void DrawTrackingTab()
    {
        GUILayout.Label("Step 1   Open your hand fully, then press:", boldLabel);
        GUI.enabled = Connected;
        if (GUILayout.Button("Capture OPEN hand", bigButton)) Capture(false);
        GUILayout.Label("Step 2   Make a tight fist, then press:", boldLabel);
        if (GUILayout.Button("Capture CLOSED fist", bigButton)) Capture(true);
        GUI.enabled = true;
        GUILayout.Space(8);
        for (int i = 0; i < 5; i++) {
            GUILayout.BeginHorizontal();
            GUILayout.Label(names[i], GUILayout.Width(55));
            Bar(Curl[i], i == 1 ? new Color(0.3f, 0.8f, 1f) : new Color(0.6f, 0.6f, 0.6f), -1);
            GUILayout.Label(Pct(Curl[i]) + "  raw " + Raw[i], GUILayout.Width(110));
            GUILayout.EndHorizontal();
            if (Mathf.Abs(closedRaw[i] - openRaw[i]) < 50)
                GUILayout.Label("   ⚠ " + names[i] + ": range too small, capture OPEN and CLOSED again");
        }
        GUILayout.Space(8);
        showCurve = GUILayout.Toggle(showCurve, "Fine-tune the curve (optional, same as before)");
        if (showCurve) DrawCurveControls();
    }

    void DrawBallTab()
    {
        bool onPath = contactSimulation != null && contactSimulation.BallOnPath;
        float limit = onPath ? contactSimulation.SurfaceLimit : -1;
        GUILayout.Label("Index finger   (orange line = ball)", boldLabel);
        Bar(MeasuredCurl[1], new Color(0.3f, 0.8f, 1f), limit);
        GUILayout.Label("Finger " + Pct(MeasuredCurl[1]) + "    Ball " + (onPath ? Pct(limit) : "not on the finger path"));
        GUILayout.Space(6);

        string block = SimpleEnableBlock();
        if (simpleServo) {
            if (ColoredButton("SERVO ON   (click to turn off)", new Color(0.3f, 1f, 0.4f), true)) { SetSimpleServo(false); GUIUtility.ExitGUI(); }
        } else {
            if (ColoredButton("ENABLE SERVO", Color.white, block.Length == 0)) { SetSimpleServo(true); GUIUtility.ExitGUI(); }
        }
        GUILayout.Label(simpleServo ? simpleStatus : block.Length > 0 ? block :
            "Servo is OFF. The arm only moves while it is ON.", wrapLabel);
        GUILayout.Space(8);

        GUILayout.Label("Ball   (the arm follows immediately while the servo is ON)", boldLabel);
        Transform ball = contactSimulation != null ? contactSimulation.BallTransform : null;
        if (ball != null && !contactSimulation.BallHidden) {
            Vector3 p = ball.localPosition;
            Vector3 before = p;
            p.z = LabeledSlider("Along the finger", p.z, -2, 7);
            p.y = LabeledSlider("Up / down", p.y, -5, 3);
            p.x = LabeledSlider("Left / right", p.x, -4, 4);
            if (p != before) ball.localPosition = p;
            float size = ball.localScale.x;
            float nextSize = LabeledSlider("Size", size, 0.2f, 2.5f);
            if (!Mathf.Approximately(size, nextSize)) ball.localScale = Vector3.one * nextSize;
        }
        GUILayout.BeginHorizontal();
        if (contactSimulation != null) {
            if (GUILayout.Button(contactSimulation.BallHidden ? "Restore sphere" : "Move sphere away / release")) {
                contactSimulation.BallHidden = !contactSimulation.BallHidden; GUIUtility.ExitGUI();
            }
            if (GUILayout.Button("Reset ball")) { contactSimulation.BallHidden = false; contactSimulation.ResetBall(); GUIUtility.ExitGUI(); }
            if (GUILayout.Button("Camera: side view")) contactSimulation.ShowContactView();
        }
        GUILayout.EndHorizontal();
        GUILayout.Space(8);

        showSpeed = GUILayout.Toggle(showSpeed, "Servo speed");
        if (showSpeed) {
            GUILayout.Label("Move to ball: " + approachRate + " us/s");
            approachRate = QuantizeRate(Mathf.RoundToInt(GUILayout.HorizontalSlider(approachRate, 100, 2000)));
            GUILayout.Label("Release: " + releaseRate + " us/s");
            releaseRate = QuantizeRate(Mathf.RoundToInt(GUILayout.HorizontalSlider(releaseRate, 100, 2000)));
            if (GUILayout.Button("Save speed")) SaveSpeed();
        }
    }

    float LabeledSlider(string label, float value, float min, float max)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, GUILayout.Width(110));
        value = GUILayout.HorizontalSlider(value, min, max);
        GUILayout.Label(value.ToString("F2"), GUILayout.Width(40));
        GUILayout.EndHorizontal();
        return value;
    }

    void DrawFistTab()
    {
        GUILayout.Label("Measures where the arm stops YOUR finger. Wear the glove.", boldLabel);
        GUILayout.Label(
            "1. Do tab 1 Finger tracking first.\n" +
            "2. Make a fist and keep pushing gently against the arm.\n" +
            "3. Press START. The arm moves in until it touches your finger, then opens it\n" +
            "    step by step (down to " + fistToText + " us). At every step the pot is read 10 times.\n" +
            "4. It stops by itself if the pot reads 0 / max, your finger is fully open,\n" +
            "    or your finger jumps open (you relaxed). Space or STOP stops it any time.", wrapLabel);
        GUILayout.Space(4);
        Bar(MeasuredCurl[1], MeasuredCurl[1] >= fistMinCurl ? new Color(0.3f, 1f, 0.4f) : new Color(1f, 0.7f, 0.2f), fistMinCurl);
        GUILayout.Label("Fist check: index " + Pct(MeasuredCurl[1]) + (MeasuredCurl[1] >= fistMinCurl ? "  ✓" : "  (need " + Pct(fistMinCurl) + ")") +
            "    raw " + Raw[1]);
        GUI.enabled = !fistMode;
        GUILayout.BeginHorizontal();
        GUILayout.Label("Fist check at " + Pct(fistMinCurl), GUILayout.Width(120));
        float nextMin = Mathf.Round(GUILayout.HorizontalSlider(fistMinCurl, 0.3f, 0.9f) * 20f) / 20f;
        GUILayout.EndHorizontal();
        if (!Mathf.Approximately(nextMin, fistMinCurl)) {
            fistMinCurl = nextMin;
            PlayerPrefs.SetFloat("HapticGlove.FistMinCurl", fistMinCurl);
            PlayerPrefs.Save();
        }
        GUILayout.Label("Can't reach it? Lower it to the tightest fist you can make with the arm released.", wrapLabel);

        GUILayout.BeginHorizontal();
        GUILayout.Label("Step us", GUILayout.Width(52)); fistStepText = GUILayout.TextField(fistStepText, 3, GUILayout.Width(40));
        GUILayout.Label("Settle ms", GUILayout.Width(62)); fistSettleText = GUILayout.TextField(fistSettleText, 4, GUILayout.Width(45));
        GUILayout.Label("Stop at us", GUILayout.Width(66)); fistToText = GUILayout.TextField(fistToText, 4, GUILayout.Width(45));
        GUILayout.EndHorizontal();
        GUI.enabled = true;

        string block = FistBlock();
        if (!fistMode && ColoredButton("START SWEEP   (hold your fist)", Color.white, block.Length == 0)) { StartFistSweep(); GUIUtility.ExitGUI(); }
        GUILayout.Label(!fistMode && block.Length > 0 ? block + "\n" + fistStatus : fistStatus, wrapLabel);
        GUILayout.Space(8);

        SortedDictionary<int,int[]> shown = fistSweeping ? fistNew : fistTable;
        if (!fistSweeping) {
            GUILayout.Label(fistTable.Count >= 2
                ? "Saved table: " + fistTable.Count + " points, covers " + FistRangeLabel() + " curl. The ball uses it."
                : "No table for the new arm position yet: tab 2 needs one. Run the sweep.", wrapLabel);
        }
        if (shown.Count > 0) {
            showFistTable = GUILayout.Toggle(showFistTable, "Show readings (" + shown.Count + ")");
            if (showFistTable) {
                GUILayout.Label("pulse us | index raw (min–max) | curl | steady");
                foreach (var kv in shown) {
                    int[] r = kv.Value;
                    GUILayout.Label(kv.Key + " | " + r[0] + " (" + r[1] + "–" + r[2] + ") | " + Pct(CalibratedCurl(1, r[0])) +
                        " | " + (r[3] == 1 ? "yes" : "NO"));
                }
            }
            GUI.enabled = !fistMode && fistTable.Count > 0;
            if (GUILayout.Button("Delete saved table")) {
                fistTable = new SortedDictionary<int,int[]>();
                fistCurveTime = -1;
                SaveFistTable();
                fistStatus = "Table deleted";
                GUIUtility.ExitGUI();
            }
            GUI.enabled = true;
        }
    }
}
