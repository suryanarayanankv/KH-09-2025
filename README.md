# SignWav: Sign Language Recognition System

## Overview

**SignWav** is a **Sign Language Recognition System** using **MPU6050 sensors** integrated with an **ESP32** through a **TCA9548A multiplexer**. The system captures motion data from **five fingers and the backside of the palm** and transmits it over **WiFi using UDP** to a Python script. A **K-Nearest Neighbors (KNN) model** is trained to recognize sign language letters from the collected sensor data.

Additionally, we are currently developing a **Conversational Model for Sign Language**, which will enhance real-time communication by translating sign gestures into meaningful sentences instead of just letters.

## Features

- **MPU6050 Sensor Integration**: Motion data collection from fingers and palm.
- **ESP32 Communication**: Data transmission via UDP over WiFi.
- **KNN-based Machine Learning Model**: Used to classify sign language letters.
- **Real-Time Prediction**: Receives sensor data and predicts corresponding letters.
- **Conversational Model Development**: Translating sequences of detected letters into full sentences.
- **Scalable Dataset Handling**: Collects and stores gesture data for training and testing.

## Technologies Used

- **Hardware**:
  - MPU6050 Accelerometer & Gyroscope Sensors
  - ESP32 Microcontroller
  - TCA9548A I2C Multiplexer
- **Software**:
  - **Python** (Data processing, training, and prediction)
  - **Scikit-Learn** (KNN model for classification)
  - **Pandas & NumPy** (Data handling)
  - **Socket Programming** (UDP communication between ESP32 and Python script)
  - **C++ (Arduino/ESP32)** (Firmware for ESP32)

## Installation

### Prerequisites

- **Python 3.x** installed
- **ESP32 Board with Arduino IDE** setup
- Required Python libraries:
  ```sh
  pip install pandas numpy scikit-learn
  ```

### Steps to Run Locally

1. **Clone the repository**:
   ```sh
   git clone https://github.com/rixprog/signwav.git
   cd signwav
   ```
2. **Upload the ESP32 firmware**:
   - Open `esp32_udp.ino` in the Arduino IDE.
   - Set up the correct WiFi credentials.
   - Flash the code to your ESP32 board.
3. **Run the Python receiver script**:
   ```sh
   python reciever.py
   ```
4. **Record gesture data**:
   ```sh
   python data_csv.py
   ```
5. **Train the model**:
   ```sh
   python data_csv.py --train
   ```
6. **Test sign recognition**:
   ```sh
   python reciever.py --predict
   ```

## Usage

- **Recording Gestures**: Collect and label sign language motion data.
- **Training the Model**: Uses recorded gestures to build the KNN classifier.
- **Real-Time Prediction**: Streams motion data from ESP32 and predicts letters in real time.
- **Conversational Model**: Ongoing development to enhance real-time sentence formation from recognized signs.

## Contributing

1. **Fork the repository**.
2. **Create a new branch**:
   ```sh
   git checkout -b feature-branch
   ```
3. **Make changes and commit**:
   ```sh
   git commit -m "Added new feature"
   ```
4. **Push to your branch**:
   ```sh
   git push origin feature-branch
   ```
5. **Open a pull request**.

## License

This project is licensed under the MIT License.
