# Test 1  : ROSWebTools


## RTT and Transmission Time Test in ROS 2 Iron using ROSWebTools Interface

### 1. **Start the Apache Web Server or any other hosting server**
   - Start a web hosting server like Apache to host your web interface, making it accessible to the tablet or other devices.
   - **Command:** 
     ```bash
     sudo systemctl start apache2
     ```
   - **Verify Server is Running:**
     - After starting Apache, check if it's running properly:
     ```bash
     sudo systemctl status apache2
     ```
     You should see the service as "active (running)".

### 2. **Make the test ros workspace and build it and export it's location**
#### 2.1 **On Loki**
    - Create a new ROS 2 workspace for the test and build the workspace.
    - add the two packages `interface` and `loki` to the `src` folder of the workspace which are located in the `IHM/tools` folder.
    - **Command:**
      ```bash
      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws
      cp -r ~/IHM/tools/interface ~/ros2_ws/src
      cp -r ~/IHM/tools/loki ~/ros2_ws/src
      colcon build
      ```
    - **Export the Workspace Location:**
      ```bash
      export test_ws=~/ros2_ws
      ```
      This will export the workspace location to the environment variable `ROS2_WS`.

#### 2.2 **On Robot**
    - Copy the robot script /IHM/tools/robot/robot_server.py to the robot.
    - **Command:**
      ```bash
      scp ~/IHM/tools/robot/robot_server.py <robot_ip>:~/
      ```
    - Run the robot script on the robot.
    - **Command:**
      ```bash
      python robot_server.py
      ```
    - **Note:**
      - The script works in both Python 2 and Python 3.
#### 2.3 **On Yunobo**
    - Create a new ROS 2 workspace for the test and build the workspace.
    - add the two packages `interface` and `yunobo` to the `src` folder of the workspace which are located in the `IHM/tools` folder.
    - **Command:**
      ```bash
      mkdir -p ~/ros2_ws/src
      cd ~/ros2_ws
      cp -r ~/IHM/tools/interface ~/ros2_ws/src
      cp -r ~/IHM/tools/yunobo ~/ros2_ws/src
      colcon build
      ```
    - **Export the Workspace Location:**
      ```bash
      export test_ws=~/ros2_ws
      ```
      This will export the workspace location to the environment variable `ROS2_WS`.


### 3. **Run ROS 2 Iron Nodes**
#### 3.1 **On Loki**
    - Find the Loki.sh file in the `IHM/tools/scripts` folder and run it.
    - You may need to change the permissions of the file to make it executable.
    - **Command:**
      ```bash
      chmod +x Loki.sh
      ./Loki.sh <middleware>
      ```
#### 3.2 **On Yunobo**
    - The same as Loki, find the Yunobo.sh file in the `IHM/tools/scripts` folder and run it.
    - **Command:**
      ```bash
      chmod +x Yunobo.sh
      ./Yunobo.sh <middleware> <ip_address>
      ```
#### 3.3 **For zenoh**
    - For zenoh, you need to export the zenoh workspace location to the environment variable `zenoh_ws`.
    - **Command:**
      ```bash
      export zenoh_ws=~/zenoh_ws
      ```
    - Then run the zenoh nodes.
    - **Command:**
      ```bash
      ./Loki.sh zenoh
      ./Yunobo.sh zenoh <ip_address>
      ```
    - **Note:**
      - Zenoh routers should be running on both devices and one of them should be running the zenoh router with the IP address of the other device.
      - You can find more information on how to install and run the middleware in the zenoh documentation. [Zenoh Documentation](https://github.com/ros2/rmw_zenoh)
### 4. **Open the Web Interface**
   - Open the web interface on the tablet or any other device by entering the IP address of the server in the browser.
   - **URL:** 
     ```
     http://<ip_address>
     ```
   - Go to the end of the page were you will find the buttons to start and download the test.
   - **Note:** 
     - The test will start after pressing the start button.
     - The button will change color to indicate the test is running.
     - The test will take 30 minutes to complete.






