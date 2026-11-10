using System;
class ServoPlannerChecks {
    static void Check(bool ok,string name) { if(!ok) throw new Exception(name); }
    static int Plan(bool valid,bool found,bool contact,bool prep,float raw,float limit) {
        return FingerServoTargetPlanner.Plan(valid,found,contact,prep,raw,limit,1000,1460,2000,2050,120);
    }
    static void Main() {
        Check(Plan(true,true,false,false,0,0.5f)==2050,"legacy release");
        Check(Plan(true,true,false,true,0,0.5f)==1580,"prepare beyond predicted stop");
        Check(Plan(true,true,true,true,0.5f,0.5f)==1460,"contact only at stop");
        Check(Plan(true,true,false,true,0.4f,0.5f)==1580,"small opening stays prepared");
        Check(Plan(true,false,false,true,0.5f,0.5f)==2050,"object removed");
        Check(Plan(false,true,true,true,0.5f,0.5f)==2050,"invalid tracking release");
        Check(Plan(true,true,false,true,float.NaN,0.5f)==2050,"bad sensor release");
        Check(Plan(true,true,false,true,0.95f,0.2f)>=FingerServoTargetPlanner.StopAt(0.95f,1000,1460,2000),"never prepare behind measured pose");
        Check(Plan(true,true,false,true,1,1)==2050,"clamp to released endpoint");
        for(int i=0;i<=100;i++) { float curl=i/100f; int t=Plan(true,true,false,true,curl,0.5f);Check(t>=1000&&t<=2050,"bounds"); }
        bool contact = false;
        float[] curlCycle = {0.4f,0.51f,0.49f,0.46f,0.48f,0.51f};
        int[] expected = {1580,1460,1460,1580,1580,1460};
        for(int i=0;i<curlCycle.Length;i++) {
            contact = FingerServoTargetPlanner.ContactState(contact,true,true,curlCycle[i],0.5f,0.03f);
            Check(Plan(true,true,contact,true,curlCycle[i],0.5f)==expected[i],"contact-open-close cycle avoids full release and jitter");
        }
        Check(!FingerServoTargetPlanner.ContactState(true,true,false,0.5f,0.5f,0.03f),"object removal clears contact");
        Check(Plan(true,false,false,true,0.5f,0.5f)==2050,"removed object fully releases");
        Check(FingerServoTargetPlanner.DynamicGap(0,0.5f,0,0.08f,120,80)==120,"far gap");
        Check(FingerServoTargetPlanner.DynamicGap(0.49f,0.5f,0,0.08f,120,80)<120,"near gap contracts");
        int slow = FingerServoTargetPlanner.DynamicGap(0.3f,0.5f,0,0.08f,120,80);
        int fast = FingerServoTargetPlanner.DynamicGap(0.3f,0.5f,3,0.08f,120,80);
        Check(fast < slow && fast >= 80,"fast closing anticipates surface without removing gap");
        Check(FingerServoTargetPlanner.DynamicGap(float.NaN,0.5f,3,0.08f,120,80)==120,"bad motion uses far gap");
        Check(!FingerServoTargetPlanner.ContactState(false,true,true,0.3f,0.5f,0.03f),"prediction never creates early contact");
        contact = false;
        for (int grasp = 0; grasp < 2; grasp++) {
            foreach (float curl in new float[] {0.22f,0.27f,0.26f,0.255f,0.26f,0.22f}) {
                contact = FingerServoTargetPlanner.ContactState(contact,true,true,curl,0.266f,0.03f);
                int expectedTarget = curl == 0.22f ? 1325 : 1316;
                Check(FingerServoTargetPlanner.VerifiedIndexTarget(contact,true)==expectedTarget,
                    "verified continuous index: hold through small opening, prepare on deliberate release, repeat");
            }
        }
        var movingBall = new IndexSphereDrive();
        Check(movingBall.Resolve(true,true,false,true,0,0.266f)==1325,"original prepare");
        Check(movingBall.Resolve(true,true,true,true,0.27f,0.266f)==1316,"original hold");
        Check(movingBall.Resolve(true,true,false,true,0,0.20f)<1325,"new surface changes target");
        Check(movingBall.Resolve(true,true,true,true,0.21f,0.20f)<1316,"hold at the new surface");
        Check(movingBall.Resolve(true,true,true,true,0.21f,0.10f)==2050,"surface moved across bent finger releases");
        Check(movingBall.Resolve(true,false,false,true,0,1)==2050,"ball moved out of reach");
        Check(movingBall.Resolve(true,true,false,true,0,0.266f)==1325,"returning ball resumes without enable");
        Check(movingBall.Resolve(true,true,true,true,0.6f,0.6f)==2050,"outside sampled interval releases");
        Console.WriteLine("PASS: preparation clearance, contact mapping, release, invalid data, measured-pose guard, bounds");
    }
}
