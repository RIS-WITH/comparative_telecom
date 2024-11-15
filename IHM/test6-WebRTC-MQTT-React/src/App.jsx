import React, { useState, useEffect, useRef, useCallback } from "react";
import mqtt from 'mqtt';
import mqttSettings from './settings';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { pack, unpack } from 'msgpackr';

const TeleopInterface = () => {
  const [messages, setMessages] = useState([]);
  const [messageInput, setMessageInput] = useState('');
  const [cameraVisible, setCameraVisible] = useState(true);
  const [driveVisible, setDriveVisible] = useState(true);
  const canvasRef = useRef(null);
  const TOPIC_DRIVE = "cmd_vel/geometry_msgs/TwistStamped";
  const TOPIC_CHAT = "chatter/std_msgs/String";
  const TOPIC_TIMESTAMP = "cmd_vel_time_stamp/interfaces/CommandTimestamp";
  const TOPIC_COMMAND_COMPLETION = "temp_completion/std_msgs/Int64";
  const MAX_COMMANDS = 100000;
  const commandTimestamps = useRef(new Map());
  const [client, setClient] = useState(null);

  useEffect(() => {
    const mqttClient = mqtt.connect(mqttSettings.url, mqttSettings.options);
    setClient(mqttClient);

    mqttClient.on('connect', async () => {
      console.log('connected to MQTT broker');
      try {
        if (mqttClient.connected) {
          await mqttClient.subscribeAsync([TOPIC_CHAT, TOPIC_TIMESTAMP, TOPIC_COMMAND_COMPLETION], { qos: 0 });
          console.log('subscribed to topics');
        }
      } catch (err) {
        console.error('Failed to subscribe to topics:', err);
      }
    });

    const handleChatMessage = async (message) => {
      setMessages((prevMessages) => [...prevMessages, `ROS2: ${message.toString()}`]);
    };

    const handleTimestampMessage = async (message) => {
      try {
      const receivedTimestamp = unpack(message);
      const commandId = receivedTimestamp.command_id;
      const commandTimestampIndex = receivedTimestamp.timestamp_index;
      const commandTimestamp = receivedTimestamp.timestamp;

      if (!commandTimestamps.current.has(commandId)) {
        commandTimestamps.current.set(commandId, { T1: null, T2: null, T3: null, T4: null, T5: null, T6: null });
      }
      commandTimestamps.current.get(commandId)['T' + commandTimestampIndex] = commandTimestamp;
      } catch (error) {
      console.error('Failed to parse timestamp message:', error);
      }
    };

    const handleCommandCompletionMessage = async (message, currentTimeNs) => {
      const receivedCommandId = unpack(message);

      if (commandTimestamps.current.has(receivedCommandId)) {
      commandTimestamps.current.get(receivedCommandId).T6 = currentTimeNs;
      }
      trimCommandTimestamps();
    };

    mqttClient.on('message', async (topic, message) => {
      const currentTimeNs = Date.now() * 1000000; // Register time early
      const handlers = {
      [TOPIC_CHAT]: handleChatMessage,
      [TOPIC_TIMESTAMP]: handleTimestampMessage,
      [TOPIC_COMMAND_COMPLETION]: (msg) => handleCommandCompletionMessage(msg, currentTimeNs),
      };

      const handler = handlers[topic];
      if (handler) {
      await handler(message);
      } else {
      console.warn(`Unhandled topic: ${topic}`);
      }
    });

    return () => {
      mqttClient.end();
    };
  }, []);
    

  const sendMessage = () => {
    if (client) {
      const packedMessage = pack(messageInput);
      client.publish(TOPIC_CHAT, packedMessage, { qos: 0 });
      setMessages((prevMessages) => [...prevMessages, `You: ${messageInput}`]);
      setMessageInput('');
    }
  };

  const trimCommandTimestamps = () => {
    if (commandTimestamps.current.size > MAX_COMMANDS) {
      const oldestCommandId = commandTimestamps.current.keys().next().value;
      commandTimestamps.current.delete(oldestCommandId);
    }
  };

  const downloadCommandTimestamps = () => {
    let csv = 'T0,T1,T2,T3,T4,T5,T6\n';
    commandTimestamps.current.forEach((timestamps, commandId) => {
      csv += `${commandId},${commandId},${timestamps.T2},${timestamps.T3},${timestamps.T4},${timestamps.T6},${timestamps.T6}\n`;
    });

    const hiddenElement = document.createElement('a');
    hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);
    hiddenElement.target = '_blank';
    hiddenElement.download = 'commandTimestamps.csv';
    hiddenElement.click();
    hiddenElement.remove();
  };

  const testSimulateCommandClicks = () => {
    const commands = [
      { linear: 1.0, angular: 0.0 },
      { linear: 0.0, angular: 1.0 },
      { linear: -1.0, angular: 0.0 },
      { linear: 0.0, angular: -1.0 },
      { linear: 0.0, angular: 0.0 }
    ];

    const startTime = Date.now();
    const testButton = document.querySelector('.command-timestamps-section .header button');

    if (testButton) {
      testButton.style.backgroundColor = 'green';
    }

    const simulateCommand = () => {
      const elapsedTime = Date.now() - startTime;

      if (elapsedTime > 1800000) {
        if (testButton) {
          testButton.style.backgroundColor = '';
        }
        return;
      }

      const randomIndex = Math.floor(Math.random() * commands.length);
      const command = commands[randomIndex];
      pubTwistStamped(command.linear, command.angular);

      let interval = 60;
      if (elapsedTime >= 1200000) interval = 10000;
      else if (elapsedTime >= 600000) interval = 1000;

      setTimeout(simulateCommand, interval);
    };

    simulateCommand();
  };

  async function publishMessage(topic, message) {
    try {
      const packet = await client.publishAsync(topic, message, { qos: 1 });
      console.log('Message published:', packet);
    } catch (err) {
      console.error('Failed to publish message:', err);
    }
  }

  const pubTwistStamped = useCallback((linear, angular) => {
    const currentTime = Date.now();
    if (client) {
      const twistStamped = {
        header: {
          stamp: {
            sec: Math.floor(currentTime / 1000),
            nanosec: (currentTime % 1000) * 1000000
          },
          frame_id: "base_link"
        },
        twist: {
          linear: { x: linear, y: 0.0, z: 0.0 },
          angular: { x: 0.0, y: 0.0, z: angular }
        }
      };
  
      publishMessage(TOPIC_DRIVE, pack(twistStamped));
    }
  }, [client]);

  return (
    <div className="container">
      <div className="chat-section">
        <header className="header">
          <h5>Chat</h5>
        </header>
        <div className="message-box">
          {messages.map((msg, index) => (
            <div key={index} className="message">{msg}</div>
          ))}
        </div>
        <div className="input-section">
          <input
            type="text"
            placeholder="Type your message here..."
            value={messageInput}
            onChange={(e) => setMessageInput(e.target.value)}
          />
          <button onClick={sendMessage}>Send</button>
        </div>
      </div>

      <div className="camera-section">
        <header className="header" onClick={() => setCameraVisible(!cameraVisible)}>
          <h5>Camera ROS</h5>
        </header>
        {cameraVisible && (
          <div className="camera-container">
            <canvas ref={canvasRef} width="300" height="300"></canvas>
          </div>
        )}
      </div>

      <div className="drive-section">
        <header className="header" onClick={() => setDriveVisible(!driveVisible)}>
          <h5>Drive</h5>
          <FontAwesomeIcon icon="fa-solid fa-gamepad" />
        </header>
        {driveVisible && (
          <div className="drive-controls">
            <button className="drive-button" onMouseDown={() => pubTwistStamped(1.0, 0.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
              <FontAwesomeIcon icon="fa-solid fa-caret-up" />
            </button>
            <div className="drive-middle-row">
              <button className="drive-button" onMouseDown={() => pubTwistStamped(0.0, 1.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
                <FontAwesomeIcon icon="fa-solid fa-caret-left" />
              </button>
              <button className="drive-button stop" onClick={() => pubTwistStamped(0.0, 0.0)}>
                <FontAwesomeIcon icon="fa-solid fa-stop-circle" />
              </button>
              <button className="drive-button" onMouseDown={() => pubTwistStamped(0.0, -1.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
                <FontAwesomeIcon icon="fa-solid fa-caret-right" />
              </button>
            </div>
            <button className="drive-button" onMouseDown={() => pubTwistStamped(-1.0, 0.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
              <FontAwesomeIcon icon="fa-solid fa-caret-down" />
            </button>
          </div>
        )}
      </div>

      <div className="command-timestamps-section">
        <header className="header">
          <h5>Command Timestamps</h5>
          <button onClick={testSimulateCommandClicks}>Test</button>
          <button onClick={downloadCommandTimestamps}>Download</button>
        </header>
      </div>
    </div>
  );
};

export default TeleopInterface;