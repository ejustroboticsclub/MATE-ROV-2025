# ROS 2 ZED Open Capture JSON Integration

### 1. `json_data_topic`

* **Type** : `std_msgs/String`
* **Purpose** : This topic is used to publish JSON data containing image and IMU information (e.g., pitch, roll, yaw, etc.) from the RPI (connected to the ZED2i) to the laptop.
* **Message Format** :
* The JSON message contains metadata about the captured photo and IMU data (roll, pitch, yaw).
* Example:

  ```json
  {
    "angleViewX": 5.6447997,
    "angleViewY": 4.2447996,
    "focalLength": 1294.56427,
    "pictures": [
      {
        "degrees": 22,
        "ring": 1,
        "sensors": {
          "fileUri": "img-r1-22.jpg",
          "roll_pitch_yaw": {
            "pitch": 0.1,
            "roll": 0.2,
            "yaw": 1.0
          }
        }
      }
    ]
  }
  ```

## Nodes Overview

### 1. **ZedOCJsonCapture (RPI)**

* **Purpose** : This node runs on the RPI where the ZED2i camera is connected. It captures image and IMU data, formats the data into JSON, and publishes it to the `json_data_topic`.

#### How to Use:

1. Build the ROS 2 workspace and source it:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
2. Launch the `ZedOCJsonCapture` node on the RPI:

   ```bash
   ros2 run your_package zed_oc_json_capture
   ```
3. Press `x` or `X` in the terminal to capture and publish data. Each press will:

   * Capture the IMU data (pitch, roll, yaw).
   * Capture an image.
   * Append this information into the `data.json` file on the RPI.
   * Publish the captured data to the `json_data_topic` topic.
4. The data will be published in the format shown in the **Topics Overview** section.

---

### 2. **JsonSaver (Laptop)**

* **Purpose** : This node subscribes to the `json_data_topic` topic, receives the JSON data from the RPI, and saves it to a JSON file on the laptop.

#### How to Use:

1. Build the ROS 2 workspace and source it on the  **laptop** :

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
2. Launch the `JsonSaver` node on the  **laptop** :

   ```bash
   ros2 run your_package json_saver
   ```
3. The `JsonSaver` node will subscribe to the `json_data_topic` and automatically save the incoming JSON data to a file (`data_on_laptop.json`) on the laptop.
4. The saved file on the laptop will contain all the captured data from the RPI, including the image file names, roll, pitch, yaw, and other metadata.

---

## How to Set Up the System

### 1.  **On the RPI (ZED2i Camera)** :

* Make sure the ZED2i camera is connected to the RPI.
* Install the necessary dependencies and configure the system as per the ZED Open Capture SDK.
* Run the `ZedOCJsonCapture` node on the RPI to start capturing and publishing data.

### 2.  **On the Laptop** :

* Install ROS 2 and set up the workspace on the laptop.
* Run the `JsonSaver` node on the laptop to receive and save the JSON data.

---

## File Locations:

* **RPI** :
* `data.json`: The JSON file storing the captured data (e.g., roll, pitch, yaw, image filenames).
* **Laptop** :
* `data.json`: The JSON file where the received data is saved. (will be changed)
