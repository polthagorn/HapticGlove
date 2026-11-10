using UnityEngine;

// Simulation only. Never opens a serial port or writes a servo command.
// Geometry matches GloveDemo's index finger; not a general mesh collision solver.
public class IndexContactSimulation : MonoBehaviour
{
    public GloveHand hand;
    public bool mouseSlider;
    [Range(0, 1)] public float testCurl;
    public float releaseMargin = 0.03f;
    public float fingerRadius = 0.275f;
    [Tooltip("Index-only calibration. Hardware mode uses this table when explicitly enabled.")]
    public int openStopUs = 1000;
    public int halfStopUs = 1460;
    public int closedStopUs = 2000;
    public int releaseUs = 2050;
    public bool prepareAhead;
    public int preparationClearanceUs = 120;
    bool resolvedValid, pathFound;
    bool havePreviousSample, previousMouse;
    float previousMeasured, closingSpeed;
    public int CurrentPreparationGapUs {
        get { return FingerServoTargetPlanner.DynamicGap(measured,limit,closingSpeed,0.08f,preparationClearanceUs,80); }
    }
    public bool CanCheckClearance { get { return resolvedValid && pathFound && limit > 0 && CalibrationValid; } }
    public int SurfaceStopUs { get { return FingerServoTargetPlanner.StopAt(limit,openStopUs,halfStopUs,closedStopUs); } }
    // Validated on this index at this sphere boundary only, with the current calibration.
    public bool AtBenchPoint { get { return CanCheckClearance && Mathf.Abs(limit - 0.266f) <= 0.001f && SurfaceStopUs == 1245; } }
    public bool HasContact { get { return touching; } }
    public float SurfaceLimit { get { return limit; } }
    // Static bench measurements for this index assembly. Interpolate only within
    // the measured interval for the CONTACT check, pending physical verification.
    public int BenchContactTargetUs {
        get {
            if (AtBenchPoint) return FingerServoTargetPlanner.VerifiedIndexStopUs;
            const float lowerCurl = 0.072f, upperCurl = 0.291f;
            if (limit < lowerCurl || limit > upperCurl) return SurfaceStopUs;
            return Mathf.RoundToInt(Mathf.Lerp(1245f,1325f,Mathf.InverseLerp(lowerCurl,upperCurl,limit)));
        }
    }
    public int FirstTouchTestTarget {
        get { return FingerServoTargetPlanner.Plan(true,true,false,true,limit,limit,
            openStopUs,halfStopUs,closedStopUs,releaseUs,
            FingerServoTargetPlanner.DynamicGap(limit,limit,0,0,preparationClearanceUs,80)); }
    }
    public bool IsPreparing {
        get { return isActiveAndEnabled && resolvedValid && pathFound && prepareAhead && !touching &&
            CalibrationValid && HardwareTargetUs < releaseUs; }
    }
    readonly IndexSphereDrive sensorDrive = new IndexSphereDrive();
    int sensorTarget = 2050;
    public int HardwareTargetUs {
        get { if (!mouseSlider) return isActiveAndEnabled ? sensorTarget : releaseUs;
            if (isActiveAndEnabled && AtBenchPoint) return FingerServoTargetPlanner.VerifiedIndexTarget(touching,prepareAhead);
            return FingerServoTargetPlanner.Plan(isActiveAndEnabled && resolvedValid,
            pathFound, touching, prepareAhead, measured, limit,
            openStopUs,halfStopUs,closedStopUs,releaseUs,CurrentPreparationGapUs); }
    }
    public string TargetPhase {
        get { if (!mouseSlider) return sensorDrive.Status;
            return HardwareTargetUs == releaseUs ? "RELEASE" : touching ? "CONTACT" : "PREPARE (clearance, no virtual contact)"; }
    }
    const string ProfileKey = "HapticGlove.IndexServo.V1.";
    bool showCalibration;
    bool showPanel = true;
    bool showContactControls = true;
    bool showViewHelp;
    string openEdit, halfEdit, closedEdit;
    string profileMessage = "";
    Vector2 panelScroll;
    public bool CalibrationValid { get { return ValidTable(openStopUs, halfStopUs, closedStopUs, releaseUs); } }
    static bool ValidTable(int open, int half, int closed, int release) {
        return FingerServoTargetPlanner.ValidTable(open,half,closed,release);
    }
    void LoadCalibration() {
        int a = PlayerPrefs.GetInt(ProfileKey + "open", 1000);
        int b = PlayerPrefs.GetInt(ProfileKey + "half", 1460);
        int c = PlayerPrefs.GetInt(ProfileKey + "closed", 2000);
        if (ValidTable(a,b,c,2050)) { openStopUs = a; halfStopUs = b; closedStopUs = c; releaseUs = 2050; }
        openEdit = openStopUs.ToString(); halfEdit = halfStopUs.ToString(); closedEdit = closedStopUs.ToString();
    }
    void SaveCalibration() {
        int a,b,c;
        if (!int.TryParse(openEdit,out a) || !int.TryParse(halfEdit,out b) || !int.TryParse(closedEdit,out c) || !ValidTable(a,b,c,2050)) {
            profileMessage = "Needs 1000 <= open < half < closed <= 2000. Release fixed at 2050."; return;
        }
        openStopUs = a; halfStopUs = b; closedStopUs = c; releaseUs = 2050;
        PlayerPrefs.SetInt(ProfileKey + "open",a); PlayerPrefs.SetInt(ProfileKey + "half",b); PlayerPrefs.SetInt(ProfileKey + "closed",c);
        PlayerPrefs.Save(); profileMessage = "Saved on this computer";
    }
    public int PreviewServoUs {
        get {
            if (!isActiveAndEnabled || !touching || !CalibrationValid) return releaseUs;
            return AtBenchPoint ? FingerServoTargetPlanner.VerifiedIndexStopUs : FingerServoTargetPlanner.StopAt(limit,openStopUs,halfStopUs,closedStopUs);
        }
    }
    GameObject ball;
    Material ballMaterial;
    bool showBallControls;
    bool touching, hidden;
    bool placementPending = true;
    string benchResult = "";
    float measured, displayed, limit = 1;
    string command = "RELEASE";
    public string SimulatedCommand {
        get { return isActiveAndEnabled && touching ? "LIMIT," + limit.ToString("F3", System.Globalization.CultureInfo.InvariantCulture) : "RELEASE"; }
    }
    const float Step = 0.005f;

    void Start()
    {
        LoadCalibration();
        benchResult = PlayerPrefs.GetString(ProfileKey + "lastBenchTouch", "");
        if (!PlayerPrefs.HasKey(ProfileKey + "half")) SaveCalibration();
        ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ball.name = "Index contact test sphere (simulation only)";
        ball.transform.SetParent(transform, false);
        ball.transform.localScale = Vector3.one * 0.7f;
        // Built-in pipeline: Legacy Diffuse is in Always Included Shaders, so it exists in builds
        // (Standard/Unlit get stripped when no scene asset uses them -> pink).
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (UnityEngine.Rendering.GraphicsSettings.currentRenderPipeline == null || shader == null)
            shader = Shader.Find("Legacy Shaders/Diffuse");
        if (shader != null) {
            ballMaterial = new Material(shader);
            Color orange = new Color(1f, 0.35f, 0.03f);
            if (ballMaterial.HasProperty("_BaseColor")) ballMaterial.SetColor("_BaseColor", orange);
            if (ballMaterial.HasProperty("_Color")) ballMaterial.SetColor("_Color", orange);
            ball.GetComponent<Renderer>().sharedMaterial = ballMaterial;
        }
        SetView(true);
        PlaceBall();
    }

    void SetView(bool contactSide)
    {
        Camera camera = Camera.main;
        if (camera == null) return;
        Vector3 position = contactSide ? new Vector3(-8, -4, 7) : new Vector3(8, 11, 12);
        camera.transform.position = transform.TransformPoint(position);
        camera.transform.LookAt(transform.TransformPoint(new Vector3(-0.8f, -0.7f, 2.4f)), transform.up);
    }

    Vector3[] Points(float curl)
    {
        var f = hand.fingers[1];
        var points = new Vector3[4];
        Matrix4x4 matrix = f.joints[0].parent.localToWorldMatrix;
        for (int j = 0; j < 3; j++) {
            matrix *= Matrix4x4.TRS(f.joints[j].localPosition,
                f.rest[j] * Quaternion.AngleAxis(curl * f.maxAngles[j], f.localBendAxis.normalized),
                f.joints[j].localScale);
            points[j] = matrix.MultiplyPoint3x4(Vector3.zero);
        }
        points[3] = matrix.MultiplyPoint3x4(new Vector3(0, 0, 1.15f));
        return points;
    }

    void PlaceBall()
    {
        if (ball == null || hand.fingers[1].rest == null) return;
        var p = Points(0.5f);
        ball.transform.position = (p[1] + p[2]) * 0.5f;
        placementPending = false;
    }

    bool Intersects(float curl)
    {
        var p = Points(curl);
        Vector3 center = ball.transform.position;
        Vector3 scale = ball.transform.lossyScale;
        float radius = Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.y), Mathf.Abs(scale.z)) * 0.5f;
        Vector3 handScale = transform.lossyScale;
        radius += fingerRadius * Mathf.Max(Mathf.Abs(handScale.x), Mathf.Abs(handScale.y), Mathf.Abs(handScale.z));
        for (int j = 0; j < 3; j++) {
            Vector3 segment = p[j + 1] - p[j];
            float t = segment.sqrMagnitude > 0 ? Mathf.Clamp01(Vector3.Dot(center - p[j], segment) / segment.sqrMagnitude) : 0;
            if ((center - (p[j] + t * segment)).sqrMagnitude <= radius * radius) return true;
        }
        return false;
    }

    public float Resolve(float animatedCurl)
    {
        if (placementPending) PlaceBall();
        measured = mouseSlider ? testCurl : hand.receiver.MeasuredCurl[1];
        float visualRequest = mouseSlider ? testCurl : animatedCurl;
        float dt = Time.unscaledDeltaTime;
        if (havePreviousSample && previousMouse == mouseSlider && dt > 0 && dt < 0.25f) {
            float velocity = (measured - previousMeasured) / dt;
            closingSpeed = Mathf.Lerp(closingSpeed, velocity, 1f - Mathf.Exp(-25f * dt));
        } else closingSpeed = 0;
        previousMeasured = measured;
        previousMouse = mouseSlider;
        havePreviousSample = true;
        bool valid = mouseSlider || (hand.receiver != null && hand.receiver.Connected);
        limit = 1;
        bool found = false;
        if (valid && ball != null && !hidden) {
            // Scan the full bend path, independent of the clamped visible pose.
            // Substeps catch contact even when the sensor jumps across the sphere.
            for (int n = 0; n <= 200; n++) {
                float curl = n * Step;
                if (!Intersects(curl)) continue;
                float low = Mathf.Max(0, curl - Step), high = curl;
                for (int k = 0; k < 8; k++) {
                    float mid = (low + high) * 0.5f;
                    if (Intersects(mid)) high = mid; else low = mid;
                }
                limit = low;
                found = true;
                break;
            }
        }
        touching = FingerServoTargetPlanner.ContactState(touching,valid,found,measured,limit,releaseMargin);
        sensorTarget = sensorDrive.Resolve(valid && CalibrationValid,found,touching,prepareAhead,measured,limit);
        resolvedValid = valid;
        pathFound = found;
        // Touch is evaluated on the un-clamped, un-smoothed sensor sample.
        // Show the contact pose immediately; retain original smoothing when free.
        displayed = touching ? limit : found ? Mathf.Min(visualRequest,limit) : visualRequest;
        command = touching ? "LIMIT index @ " + limit.ToString("F3") : "RELEASE";
        return displayed;
    }

    // Used by the simple window in GloveSimpleUI.cs.
    public bool BallOnPath { get { return isActiveAndEnabled && pathFound && !hidden && ball != null; } }
    public Transform BallTransform { get { return ball != null ? ball.transform : null; } }
    public bool BallHidden {
        get { return hidden; }
        set { hidden = value; if (ball != null) ball.SetActive(!hidden); }
    }
    public void ResetBall() {
        if (ball != null) ball.transform.localScale = Vector3.one * 0.7f;
        PlaceBall();
    }
    public void ShowContactView() { SetView(true); }

    void OnGUI()
    {
        if (!GloveReceiver.ShowAdvancedPanels) return;
        GUILayout.BeginArea(new Rect(12, 330, 390, showPanel ? Mathf.Max(200, Screen.height - 342) : 34), GUI.skin.box);
        showPanel = GUILayout.Toggle(showPanel, showPanel ? "⌄ INDEX CONTACT / CALIBRATED TARGET" : "› INDEX CONTACT / CALIBRATED TARGET");
        if (!showPanel) { GUILayout.EndArea(); return; }
        panelScroll = GUILayout.BeginScrollView(panelScroll);
        showContactControls = GUILayout.Toggle(showContactControls, "Contact and sensor controls");
        if (showContactControls) {
        bool controlsWereEnabled = GUI.enabled;
        GUI.enabled = controlsWereEnabled && (hand.receiver == null || !hand.receiver.HardwareActive);
        mouseSlider = GUILayout.Toggle(mouseSlider, "Use mouse slider instead of index sensor");
        prepareAhead = GUILayout.Toggle(prepareAhead, "Prepare index stop early (table test)");
        if (prepareAhead) {
            GUILayout.Label("Dynamic gap: " + CurrentPreparationGapUs + " us | far gap " + preparationClearanceUs);
            preparationClearanceUs = Mathf.RoundToInt(GUILayout.HorizontalSlider(preparationClearanceUs,50,250));
        }
        GUI.enabled = controlsWereEnabled;
        if (mouseSlider) testCurl = GUILayout.HorizontalSlider(testCurl, 0, 1);
        GUILayout.Label("Sensor curl: " + measured.ToString("F3") + "  Visual: " + displayed.ToString("F3"));
        GUILayout.Label("Simulated command: " + command);
        if (mouseSlider) GUILayout.Label("Contact target: " + PreviewServoUs + " us");
        else if (pathFound) GUILayout.Label("Ball surface: " + (100f * limit).ToString("F1") + "% | index range 7.2–26.6%");
        GUILayout.Label("Hardware PREVIEW: " + HardwareTargetUs + " us | " + TargetPhase);
        GUILayout.Label("Applied only when sphere control is enabled");
        bool fixedCheck = hand.receiver.CheckingClearance;
        int markTarget = fixedCheck ? hand.receiver.ClearanceTarget : HardwareTargetUs;
        float markLimit = fixedCheck ? hand.receiver.ClearanceLimit : limit;
        GUI.enabled = controlsWereEnabled && (fixedCheck || IsPreparing) && mouseSlider &&
            hand.receiver.CanMarkBenchTouch(markTarget);
        if (GUILayout.Button("Mark first physical touch")) {
            float actual = hand.receiver.MeasuredCurl[1];
            float gap = 100f * (actual - markLimit);
            benchResult = string.Format("Ball {0:F1}% | Physical {1:F1}% | {2} ({3:F1} points)",
                100f * markLimit, 100f * actual, gap < 0 ? "EARLY" : gap > 0 ? "AFTER" : "AT SURFACE", Mathf.Abs(gap));
            if (hand.receiver.CheckingContactStop) {
                bool holds = FingerServoTargetPlanner.ContactState(true,true,true,actual,markLimit,releaseMargin);
                benchResult = "CONTACT | " + benchResult + (holds ? " | inside hold range" : " | BELOW RELEASE THRESHOLD");
            }
            PlayerPrefs.SetString(ProfileKey + "lastBenchTouch", benchResult + " @ " + markTarget + " us");
            PlayerPrefs.Save();
            if (fixedCheck) hand.receiver.FinishClearanceCheck();
        }
        GUI.enabled = controlsWereEnabled;
        if (benchResult.Length > 0) GUILayout.Label(benchResult);
        }
        if (GUILayout.Button(hidden ? "Restore sphere" : "Move sphere away / release")) {
            hidden = !hidden;
            if (ball != null) ball.SetActive(!hidden);
        }
        if (showBallControls && GUILayout.Button("Reset sphere at index bend path")) {
            if (ball != null) ball.transform.localScale = Vector3.one * 0.7f;
            PlaceBall();
        }
        showBallControls = GUILayout.Toggle(showBallControls, "Sphere position and size");
        if (showBallControls && ball != null) {
            Vector3 position = ball.transform.localPosition;
            Vector3 before = position;
            position.x = AxisSlider("X (across fingers)", position.x, -4, 4);
            position.y = AxisSlider("Y (above / below palm)", position.y, -5, 3);
            position.z = AxisSlider("Z (along fingers)", position.z, -2, 7);
            if (position != before) ball.transform.localPosition = position;
            float diameter = ball.transform.localScale.x;
            float nextDiameter = AxisSlider("Diameter", diameter, 0.2f, 2.5f);
            if (!Mathf.Approximately(nextDiameter, diameter))
                ball.transform.localScale = Vector3.one * nextDiameter;
        }
        showCalibration = GUILayout.Toggle(showCalibration, "Index servo calibration table");
        if (showCalibration) {
            bool previous = GUI.enabled;
            GUI.enabled = previous && (hand.receiver == null || !hand.receiver.HardwareActive);
            GUILayout.Label("0% / 50% / 100% stop targets (us)");
            GUILayout.BeginHorizontal();
            openEdit = GUILayout.TextField(openEdit); halfEdit = GUILayout.TextField(halfEdit); closedEdit = GUILayout.TextField(closedEdit);
            GUILayout.EndHorizontal();
            GUILayout.Label("Release: 2050 us | Index only");
            if (GUILayout.Button("Save index table")) SaveCalibration();
            GUI.enabled = previous;
            GUILayout.Label(profileMessage);
        }
        showViewHelp = GUILayout.Toggle(showViewHelp, "View and help");
        if (showViewHelp) {
            if (GUILayout.Button("Contact view (under / side)")) SetView(true);
            if (GUILayout.Button("Original overview")) SetView(false);
            GUILayout.Label("Bend index: visible pose stops at sphere.\nOpen index or remove sphere: release.\nOther fingers keep their original tracking.");
        }
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    float AxisSlider(string label, float value, float minValue, float maxValue)
    {
        GUILayout.Label(label + ": " + value.ToString("F2"));
        return GUILayout.HorizontalSlider(value, minValue, maxValue);
    }

    void OnDestroy() { if (ballMaterial != null) Destroy(ballMaterial); }

    void OnDisable() { touching = false; resolvedValid = false; pathFound = false; havePreviousSample = false; closingSpeed = 0; command = "RELEASE"; }
}
