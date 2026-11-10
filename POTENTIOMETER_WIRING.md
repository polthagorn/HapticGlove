# Current potentiometer mapping

Thumb VP/GPIO36, index VN/GPIO39, middle D34/GPIO34, ring D35/GPIO35, little D32/GPIO32. These five headers are physically adjacent on the photographed classic ESP32 30-pin board, starting from VP away from EN. D33 remains unused. Each sensor wiper has its own signal wire. Outer terminals remain 3V3 and GND. Servo index remains TX2/GPIO17.

Active sketches ESP32GloveSimulation, ESP32IndexCalibration and ESP32IndexBench use this mapping. Original ESP32GloveUSB and earlier IndexServoBench are preserved with the old mapping; do not upload them with the new wiring. Recalibrate OPEN/CLOSED in Unity after rewiring. Current ESP32IndexBench must be uploaded again for mapping to take effect.
