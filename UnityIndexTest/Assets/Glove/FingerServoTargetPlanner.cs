using System;

// Pulse-width table for the current index assembly, not joint/servo angle equality.
// Pure calculation: no serial output, no hardware or Unity lifecycle calls.
public static class FingerServoTargetPlanner
{
    public const int VerifiedIndexStopUs = 1316;
    public const int VerifiedIndexPrepareUs = 1325;
    public static int VerifiedIndexTarget(bool touching, bool prepareAhead) {
        if (touching) return VerifiedIndexStopUs;
        return prepareAhead ? VerifiedIndexPrepareUs : 2050;
    }

    public static bool ValidTable(int open, int half, int closed, int release)
    {
        return open >= 1000 && open < half && half < closed && closed <= 2000 && release == 2050;
    }

    public static int StopAt(float curl, int open, int half, int closed)
    {
        curl = Math.Max(0, Math.Min(1, curl));
        double value = curl <= 0.5f ? open + (half - open) * (double)curl * 2 :
            half + (closed - half) * ((double)curl - 0.5) * 2;
        return (int)Math.Round(value);
    }

    public static bool ContactState(bool wasContact, bool valid, bool pathFound,
        float measured, float limit, float releaseMargin)
    {
        if (!valid || !pathFound || float.IsNaN(measured) || float.IsInfinity(measured) ||
            float.IsNaN(limit) || float.IsInfinity(limit)) return false;
        return wasContact ? measured >= limit - Math.Max(0, releaseMargin) : measured >= limit;
    }

    public static int DynamicGap(float measured, float limit, float closingSpeed,
        float lookAheadSeconds, int farGapUs, int nearGapUs)
    {
        int far = Math.Max(50, Math.Min(250, farGapUs));
        int near = Math.Max(50, Math.Min(far, nearGapUs));
        if (float.IsNaN(measured) || float.IsInfinity(measured) ||
            float.IsNaN(limit) || float.IsInfinity(limit) ||
            float.IsNaN(closingSpeed) || float.IsInfinity(closingSpeed) ||
            float.IsNaN(lookAheadSeconds) || float.IsInfinity(lookAheadSeconds)) return far;
        double ahead = Math.Max(0, Math.Min(0.15, lookAheadSeconds));
        double predictedCurl = Math.Max(0, Math.Min(1, measured + Math.Max(0, closingSpeed) * ahead));
        double distance = Math.Max(0, Math.Min(1, (limit - predictedCurl) / 0.25));
        return (int)Math.Round(near + (far - near) * distance);
    }

    public static int Plan(bool valid, bool pathFound, bool contact, bool prepare,
        float measured, float limit,
        int open, int half, int closed, int release, int clearanceUs)
    {
        if (!valid || !pathFound || !ValidTable(open,half,closed,release) ||
            float.IsNaN(measured) || float.IsInfinity(measured) ||
            float.IsNaN(limit) || float.IsInfinity(limit)) return release;
        if (contact) return StopAt(limit,open,half,closed);
        if (!prepare) return release;
        // This assembly opens with increasing pulses. Keep the prepared stop
        // beyond BOTH the predicted surface and the current measured pose.
        int surface = StopAt(limit,open,half,closed);
        int current = StopAt(measured,open,half,closed);
        int gap = Math.Max(50,Math.Min(250,clearanceUs));
        return Math.Min(release, Math.Max(surface,current) + gap);
    }
}
