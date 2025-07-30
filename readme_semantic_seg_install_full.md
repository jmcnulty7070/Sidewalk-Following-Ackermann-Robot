
# 🧠 Semantic Segmentation Installation Guide for Jetson Nano

Here is a detailed, step-by-step setup guide to run the segmav semantic navigation project on a Jetson Nano Developer Kit — written clearly and thoroughly for practical use.

---

## ✅ OVERVIEW
This project uses:
• Jetson Nano with JetPack SDK  
• Camera (like Raspberry Pi V2)  
• Flight controller (like Pixhawk) using MAVLink  
• Jetson Inference tools with a pretrained segmentation model

---

## 📁 FOLDER STRUCTURE (after setup)
```
~/segmav/
├── mavsegmav.py         ← main robot control code
├── segmav.py            ← segmentation + steering logic
├── segmav.service       ← systemd auto-start file (optional)
├── README.md
├── screenshot.png
└── jetson-inference/    ← cloned Jetson inference library with models
```

---

## 🧠 PREREQUISITES

### ⚙️ 1. Flash JetPack on Jetson Nano
Use NVIDIA SDK Manager or download JetPack SD image:  
🔗 [JetPack for Jetson Nano (SD Card)](https://developer.nvidia.com/embedded/jetpack)

• Flash to SD card using Balena Etcher  
• Boot Nano with monitor, keyboard, or via SSH  
• Connect to Wi-Fi or Ethernet

---

## 🧰 2. Install system packages

Open terminal on Nano:
```bash
sudo apt update
sudo apt install -y git cmake libpython3-dev python3-pip python3-numpy                     python3-opencv python3-pil libjpeg-dev libfreetype6-dev                     libcanberra-gtk-module python3-pyserial                     libopenblas-dev libopenmpi-dev gfortran                     libjpeg-dev zlib1g-dev libjpeg8-dev libpng-dev                     libavcodec-dev libavformat-dev libswscale-dev
```

---

## 📦 INSTALL jetson-inference (FROM SOURCE)

### 🔁 3. Clone and build
```bash
cd ~
git clone --recursive https://github.com/dusty-nv/jetson-inference
cd jetson-inference

# Configure build and models
mkdir build
cd build
cmake ../
```

### ✅ Choose "Pretrained models" → segmentation → `fcn-resnet18-cityscapes-1024x512`

Use arrow keys to move.  
Press SPACEBAR to check:
```
[*] segmentation--fcn-resnet18-cityscapes-1024x512
```
Then press:
• Enter to accept  
• `c` to configure  
• `g` to generate  
• Exit the menu

---

### 🛠️ 2. Build and Install
```bash
make -j4         # Compiles everything
sudo make install
sudo ldconfig    # Update shared library cache
```

---

## ✅ Confirm install
```bash
cd ~/jetson-inference/build/aarch64/bin
./segnet-console.py --help
```

---

## 🤖 INSTALL segmav PROJECT

### 📥 4. Clone your segmav code
```bash
cd ~
git clone https://github.com/YOUR-REPO/segmav.git
cd segmav
```

Or unzip the one you already have and rename:
```bash
unzip segmav-main\ \(1\).zip
mv segmav-main segmav
```

---

## 🧩 5. Install Python dependencies
```bash
cd ~/segmav
pip3 install numpy opencv-python pymavlink jetson-inference
```

---

## 🎥 6. Plug in Camera and Pixhawk

• Use the Raspberry Pi V2 camera or USB webcam  
• Connect Jetson UART (or USB) to Pixhawk via MAVLink  
• Make sure MAVLink is routed to `udp:14552`  
Use mavproxy or mavlink-router if needed.

---

## ▶️ 7. Run it!
```bash
cd ~/segmav
python3 mavsegmav.py
```
If it works, you should see:
• The camera stream  
• MAVLink messages about yaw and speed being sent  
• The robot start steering based on sidewalk segmentation

---

## 🔄 OPTIONAL: Run on Boot

To auto-start at boot, enable the systemd service:
```bash
sudo cp segmav.service /etc/systemd/system/
sudo systemctl daemon-reexec
sudo systemctl enable segmav.service
sudo systemctl start segmav.service
```

Make sure the service ExecStart path matches your install location.

---

## 🧪 TROUBLESHOOTING

| Problem           | Solution                                                   |
|-------------------|------------------------------------------------------------|
| Jetson overheating| Add a fan                                                  |
| No camera detected| Try `ls /dev/video*`, use cheese or opencv to test         |
| MAVLink timeout   | Ensure correct port and connection on flight controller    |
| SegNet error      | Make sure jetson-inference was installed from source, not apt |

---

## ✅ STEP-BY-STEP: Install fcn-resnet18-cityscapes-1024x512

During the CMake setup of jetson-inference, a GUI menu lets you choose which pre-trained models to download. Here’s what to do:

---

### 🔁 1. CMake Menu (Model Downloader)

```bash
cd ~/jetson-inference/build
cmake ../
```

✅ This will open a menu that looks like this:

```
BUILD_PYTHON       ON
INSTALL_PREFIX     /usr/local
...

[ ] segmentation--fcn-resnet18-cityscapes-1024x512
```

👉 USE THE ARROW KEYS to move.  
✅ Press SPACEBAR on:

```
[*] segmentation--fcn-resnet18-cityscapes-1024x512
```

💾 Then press:  
• Enter to accept  
• `c` to configure  
• `g` to generate  
• Then exit

---

### 🛠️ 2. Build and Install
```bash
make -j4         # Compiles everything
sudo make install
sudo ldconfig    # Update shared library cache
```

---

## 📁 FINAL FOLDER STRUCTURE (Jetson)

Here’s what your Jetson Nano home directory should look like when done:
```
~/                 ← Your Jetson Nano home
├── jetson-inference/
│   ├── build/
│   │   ├── aarch64/bin/
│   │   │   └── segnet.py  ← segmentation executable
│   │   ├── CMakeFiles/
│   ├── data/
│   │   └── networks/
│   │       └── fcn-resnet18-cityscapes-1024x512/
│   │           ├── labels.txt
│   │           ├── fcn_resnet18.onnx
│   │           └── other model files
│   └── python/
│       └── jetson_inference/
│       └── jetson_utils/
│
├── segmav/
│   ├── mavsegmav.py
│   ├── segmav.py
│   ├── segmav.service
│   ├── screenshot.png
│   └── README.md
│
└── catkin_ws/           ← You’ll create this later for ROS
    ├── src/
    └── build/
```

---

## 💡 How to Test the Model

You can test the installed model with a sample image or live camera:

```bash
cd ~/jetson-inference/build/aarch64/bin
./segnet.py --network=fcn-resnet18-cityscapes-1024x512 /dev/video0
```

If your camera works, you should see real-time segmentation with purple sidewalks.

---

## 🛠️ READY TO ADD ROS?

You’re now ready to add a ROS Catkin workspace in `~/catkin_ws/` without affecting the `segmav` or `jetson-inference` setups.
