# Scan-4-Less  
*A parody of budget-brand naming (because precision shouldn't cost a fortune!)*  

![Optional: Add a GIF/demo image of the scanner here]  

## 📌 Overview  
3D spatial scanner using **TI MSP-EXP432E401Y (Cortex-M4)** + **VL53L1X ToF sensor**, with MATLAB 3D reconstruction.  

## 🔥 Key Features  
- **Millimeter-precision depth data** via VL53L1X sensor + automated YZ-plane stepper motor control  
- **Bare-metal firmware** in C/Assembly (I2C/UART comms, real-time motor control)  
- **MATLAB 3D room geometry maps** from raw sensor data  

## 🤔 Why "Scan-4-Less"?  
A playful riff on *Fit 4 Less* branding, emphasizing **low-cost hardware for high-precision scanning**.  

---

➡ **Sequel Alert!** Check out **[Scan-4-Even-Less](https://github.com/yourusername/Scan-4-Even-Less)** for optimizations!  

## 🛠️ Hardware/Software  
| Hardware              | Software/Protocols  |  
|-----------------------|---------------------|  
| TI MSP-EXP432E401Y    | C, Assembly         |  
| VL53L1X ToF Sensor    | MATLAB              |  
| Stepper Motors (YZ)   | I2C, UART           |  

## 📜 License  
MIT License - See [LICENSE](LICENSE) for details.  
