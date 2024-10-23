# Test 1  : ROSWebTools


## RTT and Transmission Time Test in ROS 2 Iron

### 1. **Launch a Web Server**
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

### 2. **Start the Webridge Server on Loki**
   - Launch the ROS 2 Webridge server on the Loki machine. This server acts as the bridge between the ROS 2 nodes and the web interface.
   - **Command:**
     ```bash
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
     ```
   - **Troubleshooting:**
     - Ensure that `webridge_server` is installed and sourced correctly. If needed, source your workspace:
     ```bash
     source /opt/ros/iron/setup.bash
     ```

### 3. **Run ROS 2 Iron Nodes**
   - Launch the required ROS 2 nodes for the Loki, Yunobo, and Robot. These nodes should be located in the `ros2-nodes` folder on their respective machines.
   - **Command:**
     ```bash
     ros2 run <package_name> <node_name>
     ```
     Replace `<package_name>` and `<node_name>` with the appropriate package and node names for each machine (Loki, Yunobo, and Robot).
   - **Examples:**
     ```bash
     ros2 run my_robot_package loki_node
     ros2 run my_robot_package yunobo_node
     ros2 run my_robot_package robot_node
     ```

   - **Troubleshooting:**
     - Make sure each node is set up correctly and communicating. You can use the following to confirm that the nodes are running:
     ```bash
     ros2 node list
     ```
     - Ensure that they are using the same ROS 2 network and Domain ID.

### 4. **Access the Web Interface**
   - From the tablet or any device on the same network, open a browser and access the web interface served by your Apache server.
   - **URL:**
     ```
     http://<your-server-ip>
     ```
     Replace `<your-server-ip>` with the IP address of the machine running the Apache web server. For example, `http://192.168.1.100`.

   - **Troubleshooting:**
     - If the page doesn't load, verify network connectivity and make sure the Apache server is running. Use `ifconfig` or `ip a` to check your machine's IP address.

### 5. **Simulate Button Click Test**
   - Once the web interface is loaded on the tablet, look for the "Simulate Button Click Test" button.
   - **Action:**
     - Click on the "Simulate Button Click Test" button to begin the RTT and Transmission Time Test.

### 6. **Wait for Test Completion**
   - Allow the test to run for a minimum of **5 minutes** without interruption.
   - This test will measure the Round-Trip Time (RTT) and transmission times between the web interface and the ROS 2 system, simulating real-time interaction.

### 7. **Download Test Results**
   - Once the test is complete, a "Download Results" button will appear.
   - **Action:**
     - Click the "Download Results" button to download the log file or dataset containing the results of the RTT and transmission time test.



