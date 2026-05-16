# Vision-Driven Autonomous PCB Assembly Prototype

A low-cost, computer vision–guided PCB assembly prototype designed for educational, research, and small-scale manufacturing environments. This project combines OpenCV-based vision processing, ESP8266 embedded control, and an XY gantry motion system to automate PCB component placement with improved accuracy and reduced manual effort.

---

## 🚀 Project Overview

Industrial PCB pick-and-place systems are expensive and complex, making them difficult to access for students, research labs, and small manufacturers. This project presents a modular and affordable alternative capable of:

- Detecting PCB placement positions using computer vision
- Extracting coordinates automatically
- Controlling an XY gantry using stepper motors
- Performing semi-automated component placement
- Reducing manual placement errors

### 📌 System Performance

- 📍 Placement Accuracy: ±2 mm
- 🎯 Detection Rate: 92%
- 🔁 Repeatability: 90%
- 💰 Total Hardware Cost: Under ₹20,000

---

# 🛠️ System Architecture

## Main Modules

### 1. Vision Processing Unit
- Python
- OpenCV
- USB 8MP Camera

### 2. Embedded Motion Controller
- ESP8266 NodeMCU
- UART Communication

### 3. Motion System
- NEMA17 Stepper Motors
- TMC2209 / A4988 Drivers
- T8 Lead Screw XY Gantry

### 4. Mechanical Unit
- Servo-based Gripper
- Aluminum Frame Structure

---

# 📷 Features

- PCB image capture using fixed camera setup
- Automatic contour-based coordinate extraction
- Adaptive thresholding and image preprocessing
- Real-world coordinate mapping through calibration
- UART-based communication with ESP8266
- XY gantry movement control
- Modular and upgradeable architecture
- Low-cost educational automation platform

---

# 🧠 Technologies Used

## Software
- Python
- OpenCV
- Arduino IDE

## Hardware
- ESP8266 NodeMCU
- NEMA17 Stepper Motors
- TMC2209 / A4988 Drivers
- USB 8MP Camera
- T8 Lead Screws
- Servo Gripper
- SMPS Power Supply

---

# ⚙️ Vision Processing Pipeline

1. Image Acquisition  
2. Grayscale Conversion  
3. Gaussian Blur  
4. Adaptive Thresholding  
5. Morphological Operations  
6. Contour Detection  
7. Feature Filtering  
8. Centroid Extraction  

---

# 🔌 Communication Protocol

Coordinate data is transmitted from the Python vision system to the ESP8266 using UART serial communication.

### Example
```text
X45.2,Y32.7
```

The ESP8266 converts the coordinates into motor steps and drives the XY gantry accordingly.

---

# 📐 Motion Control Formula

```text
Steps = (Distance × Steps_per_Revolution × Microstepping) / Lead_Screw_Pitch
```

---

# 📦 Hardware Components

| Component | Specification |
|---|---|
| ESP8266 NodeMCU | Wi-Fi Microcontroller |
| NEMA17 Motors | 200 Steps/Rev |
| TMC2209 / A4988 | Stepper Driver |
| USB Camera | 8MP |
| T8 Lead Screw | 2 mm Pitch |
| Servo Motor | Gripper Control |
| SMPS | 12V 5A |

---

# 📊 Results

| Parameter | Result |
|---|---|
| Placement Accuracy | ±2 mm |
| Detection Rate | 92% |
| Repeatability | 90% |
| Estimated Cost | < ₹20,000 |

---

# 🎯 Applications

- Educational automation projects
- PCB prototyping labs
- Embedded systems research
- Vision-guided robotics
- Small-scale PCB assembly

---

# 🔮 Future Improvements

- SMD component handling
- Automatic component feeder
- AI/ML-based component recognition
- Closed-loop feedback control
- Multi-camera alignment system
- Industrial-grade precision enhancement

---

# 📁 Project Structure

```text
├── Python_OpenCV/
│   ├── image_processing.py
│   ├── coordinate_detection.py
│
├── ESP8266_Firmware/
│   ├── motion_control.ino
│
├── Hardware/
│   ├── circuit_diagram
│   ├── gantry_design
│
├── Documentation/
│   ├── Project_Report.pdf
│
└── README.md
```

---

# 👨‍💻 Authors

- Gowtham B
- Akash M
- Nivesh S
- Arjun V

Department of Electronics and Communication Engineering  
Kathir College of Engineering

---

# 📚 References

- OpenCV Documentation
- ESP8266 Documentation
- Embedded Systems and Computer Vision Research Papers

---

# ⭐ Conclusion

This project demonstrates that a low-cost, vision-guided PCB assembly system can be developed using accessible hardware and open-source software. The prototype bridges the gap between manual PCB assembly and expensive industrial SMT systems while providing a practical platform for learning embedded systems, robotics, and computer vision.
