# Unity index contact test — simulation only

1. Keep the original ESP32 firmware. Disconnect servo power for this test.
2. Close Arduino Serial Monitor if it owns the ESP32 port. Uno is not needed.
3. Replace Assets/Glove/GloveDemo.cs and GloveHand.cs in your existing Unity project; add IndexContactSimulation.cs. Keep GloveReceiver.cs unchanged. Preserve existing .meta files.
4. Open SampleScene. Set GloveReceiver Port Name to your actual ESP32 port. Press Play.
5. The original calibration panel remains. Below it is an index contact simulation panel.
6. First enable Use mouse slider. Sweep 0 to 1: index should stop at the sphere while Measured continues increasing. Other fingers keep tracking.
7. Hold slider past contact: LIMIT should remain stable. Open the slider more than the release margin: RELEASE.
8. At contact, click Move sphere away: index should reach the measured pose and show RELEASE. Restore sphere to test again.
9. Disable mouse slider and repeat with your glove. Open/closed calibration works as before.
10. Disconnect ESP32 with mouse mode OFF: command must show RELEASE; original last measured pose is held.

No serial writes, servo pulses, or servo angle mapping are implemented here. The simulated limit is a normalized finger curl, NOT a servo position. Uno bench firmware remains separate.

The sphere/segment solver matches only the generated index geometry. It samples the entire curl path in 0.005 steps and refines first contact. It does not support arbitrary object meshes, dynamic object physics, or all five fingers yet. Moving the generated hand or sphere is accounted for each frame. It is not a force controller. A sphere initially overlapping the open finger blocks at zero; reposition it if needed.

Source was reviewed locally. Unity Editor and physical-device validation have not been run in this workspace.
