using System;
using System.IO.Ports;
using System.Threading;
using UnityEngine;

// Dedicated Uno port. Simulation only; never transmits a servo position.
public class UnoSimulationLink : MonoBehaviour
{
    public string portName = "";
    public IndexContactSimulation contact;
    readonly object gate = new object();
    string outgoing = "RELEASE", status = "Not connected";
    DateTime updated;
    Thread worker;
    volatile bool stop;
    string editPort = "";

    void Update() {
        lock (gate) {
            outgoing = contact != null ? contact.SimulatedCommand : "RELEASE";
            updated = DateTime.UtcNow;
        }
    }

    void Connect() {
        if (worker != null && worker.IsAlive) return;
        portName = editPort.Trim();
        if (portName.Length == 0) return;
        if (contact != null && contact.hand.receiver != null && portName == contact.hand.receiver.portName) {
            lock (gate) status = "Choose Uno port, not the ESP32 port";
            return;
        }
        string selected = portName;
        stop = false;
        worker = new Thread(() => Run(selected)) { IsBackground = true };
        worker.Start();
    }

    void Run(string selected) {
        try {
            using (var serial = new SerialPort(selected, 115200)) {
                serial.ReadTimeout = 100;
                serial.WriteTimeout = 100;
                serial.DtrEnable = false;
                serial.RtsEnable = false;
                serial.Open();
                while (!stop) {
                    string command;
                    lock (gate) command = (DateTime.UtcNow - updated).TotalSeconds < 0.5 ? outgoing : "RELEASE";
                    serial.WriteLine("SIM1," + command);
                    try {
                        string reply = serial.ReadLine().Trim();
                        lock (gate) status = reply.StartsWith("ACK1,") ? reply : "Waiting for SIM firmware: " + reply;
                    } catch (TimeoutException) {
                        lock (gate) status = "Waiting for Uno ACK";
                    }
                    Thread.Sleep(50);
                }
                serial.WriteLine("SIM1,RELEASE");
            }
        } catch (Exception e) { lock (gate) status = e.Message; }
    }

    void OnGUI() {
        if (!GloveReceiver.ShowAdvancedPanels) return;
        GUILayout.BeginArea(new Rect(410, 12, 380, 160), GUI.skin.box);
        GUILayout.Label("UNO LINK — SIMULATION ONLY");
        editPort = GUILayout.TextField(editPort);
        bool running = worker != null && worker.IsAlive;
        if (GUILayout.Button(running ? "Disconnect Uno" : "Connect Uno (enter its port above)")) {
            if (running) stop = true; else Connect();
        }
        string text;
        lock (gate) text = status;
        GUILayout.Label(text);
        GUILayout.EndArea();
    }

    void OnDisable() { stop = true; if (worker != null) worker.Join(500); }
}
