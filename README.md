# **Car Prototype for Self-Parking System** 🚗

![Project Banner](https://via.placeholder.com/800x200?text=Car+Prototype+for+Self-Parking+System) *(Replace with actual project image)*

## **Table of Contents**
- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Tools](#software-tools)
- [Installation & Setup](#installation--setup)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## **Project Overview** 📌
This repository contains the firmware and documentation for a **car prototype** designed as the foundation for a **self-parking system**. The prototype demonstrates basic autonomous functionalities, including:
- **Motor control** (forward/backward movement)
- **Servo-based steering**
- **Bluetooth remote control**

The project is built using **STM32 microcontrollers** and serves as a stepping stone toward a fully autonomous self-parking vehicle.

---

## **Features** ✨
✅ **Motor Control**  
- PWM-based speed control  
- Direction control (forward/backward)  

✅ **Servo Steering**  
- Angle control (0°–180°)  
- Smooth turning mechanism  

✅ **Bluetooth Remote Control**  
- UART communication  
- Supports commands:  
  - `W` = Forward  
  - `S` = Backward  
  - `A` = Left  
  - `D` = Right  
  - `X` = Stop  

✅ **Modular & Extensible**  
- Easy integration with additional sensors (ultrasonic, IR, etc.)  
- Scalable for future self-parking algorithms  

---

## **Hardware Components** 🛠️
| Component | Description |
|-----------|-------------|
| **STM32 Microcontroller** | Main control unit (STM32F103C8T6) |
| **DC Motors** | 2x for wheel movement |
| **Servo Motor** | Steering control (SG90) |
| **Motor Driver (L298N)** | H-bridge for motor control |
| **Bluetooth Module (HC-05)** | Wireless communication |
| **Power Supply** | 5V-12V battery pack |
| **Breadboard & Jumper Wires** | Circuit connections |

*(Optional: Add a wiring diagram image)*

---

## **Software Tools** 💻
- **STM32CubeMX** (For STM32 configuration & code generation)  
- **Keil uVision / STM32 IDE** (For firmware development)  
- **Proteus** (For simulation & testing)  
- **HAL Library** (STM32 Hardware Abstraction Layer)  

---

## **Installation & Setup** ⚙️
### **1. Prerequisites**
- STM32CubeMX installed  
- STM32 IDE / Keil uVision  
- USB-TTL converter (for flashing)  

### **2. Flashing the Firmware**
1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/car-prototype-self-parking.git
   ```
2. Open the project in **STM32CubeIDE** or **Keil uVision**.
3. Build and flash the firmware to your STM32 board.

### **3. Hardware Setup**
- Connect motors to the motor driver (L298N).  
- Wire the servo motor to a PWM-enabled pin.  
- Pair the Bluetooth module (HC-05) with your phone/PC.  

*(Include a connection diagram if possible)*

---

## **Usage** 🚀
### **Basic Commands**
| Command | Action |
|---------|--------|
| `W` | Move Forward |
| `S` | Move Backward |
| `A` | Turn Left |
| `D` | Turn Right |
| `X` | Stop |

### **Bluetooth Control**
1. Power on the car prototype.  
2. Connect via Bluetooth (default HC-05 PIN: `1234`).  
3. Send commands using a serial terminal app (e.g., **Arduino Serial Monitor** or **Bluetooth Terminal**).  

*(Optional: Add a demo GIF/video)*

---

## **Project Structure** 📂
```
car-prototype-self-parking/
├── Docs/               # Project documentation
├── Firmware/           # STM32 source code
│   ├── Core/           # HAL drivers & main.c
│   ├── Drivers/        # STM32 HAL libraries
│   └── STM32CubeMX/    # Configuration files
├── Simulation/         # Proteus simulation files
└── README.md           # This file
```

---

## **Contributing** 🤝
Contributions are welcome! If you'd like to improve this project:
1. Fork the repository.  
2. Create a new branch (`git checkout -b feature/new-feature`).  
3. Commit your changes (`git commit -m "Add new feature"`).  
4. Push to the branch (`git push origin feature/new-feature`).  
5. Open a **Pull Request**.  

---

## **License** 📜
This project is licensed under the **MIT License** - see [LICENSE](LICENSE) for details.

---

## **Acknowledgements** 🙏
- **STM32CubeMX** for easy peripheral configuration.  
- **Proteus** for simulation support.  
- **HC-05 Bluetooth Module** docs for UART communication.  

---

**🌟 Happy Building! Let’s make self-parking cars a reality!** 🚘💡  

*(Replace placeholders with actual images, diagrams, and links to your repository)*