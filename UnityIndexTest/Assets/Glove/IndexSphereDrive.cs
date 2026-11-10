using System;

// Local calibration for the index assembly. Targets between bench samples are estimates.
public sealed class IndexSphereDrive
{
    const float Low = 0.072f, Reference = 0.266f, UpperSample = 0.291f;
    const float Gap = UpperSample - Reference;
    bool waitingForClearance;
    float previousLimit = float.NaN;
    public string Status { get; private set; }

    // Bench (glove on table) table: valid only for a ball between 7.2% and 26.6% curl.
    public static int BenchPulse(float curl, out bool inRange) {
        inRange = curl >= Low && curl <= Reference + 0.0005f;
        return inRange ? Pulse(Math.Min(curl, Reference)) : 2050;
    }

    static int Pulse(float curl) {
        if (curl <= Reference)
            return (int)Math.Round(1245 + (1316-1245)*(curl-Low)/(Reference-Low));
        return (int)Math.Round(1316 + (1325-1316)*(Math.Min(curl,UpperSample)-Reference)/Gap);
    }

    public int Resolve(bool valid, bool found, bool contact, bool prepare, float measured, float limit) {
        // Allow rounding of the displayed 26.6% boundary from the recorded scene.
        if (Math.Abs(limit-Reference) < 0.0005f) limit = Reference;
        if (!valid || !found || !(limit >= Low && limit <= Reference) ||
            !(measured >= 0 && measured <= 1)) {
            waitingForClearance = true;
            previousLimit = float.NaN;
            Status = !found ? "Ball outside bend path" : "Outside measured index range";
            return 2050;
        }
        if ((float.IsNaN(previousLimit) || limit < previousLimit) && measured + Gap >= limit)
            waitingForClearance = true;
        previousLimit = limit;
        if (waitingForClearance && measured + Gap >= limit) {
            Status = "Open index to clear the new surface";
            return 2050;
        }
        waitingForClearance = false;
        Status = contact ? "CONTACT" : prepare ? "PREPARE" : "RELEASE";
        return contact ? Pulse(limit) : prepare ? Pulse(limit + Gap) : 2050;
    }
}
