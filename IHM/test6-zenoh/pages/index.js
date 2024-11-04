import React, { useState, useEffect, useRef } from 'react';
import { CdrReader, CdrWriter } from "@foxglove/cdr";

const TeleopInterface = () => {
  const [messages, setMessages] = useState([]);
  const [messageInput, setMessageInput] = useState('');
  const [cameraVisible, setCameraVisible] = useState(true);
  const [driveVisible, setDriveVisible] = useState(true);
  const canvasRef = useRef(null);
  const rest_api = "http://140.93.6.33:8800/";
  const scope = "3/";
  const TOPIC_DRIVE = "cmd_vel/geometry_msgs::msg::dds_::TwistStamped_/RIHS01_5f0fcd4f81d5d06ad9b4c4c63e3ea51b82d6ae4d0558f1d475229b1121db6f64";
  const TOPIC_CHAT = "chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
  const TOPIC_TIMESTAMP = "cmd_vel_time_stamp/interfaces::msg::dds_::CommandTimestamp_/RIHS01_ff56d63f170881d1222138d1dc80184879835e4160323aff37c33a885c6eaa84";
  const TOPIC_COMMAND_COMPLETION = "temp_completion/std_msgs::msg::dds_::UInt64_/RIHS01_fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943"
  const MAX_COMMANDS = 100000;
  const [commandTimestamps, setCommandTimestamps] = useState(new Map());

  class String {
    constructor(data) {
      this.data = data;
    }
    static decode(cdrReader) {
      const data = cdrReader.string();
      return new String(data);
    }
    encode(cdrWriter) {
      cdrWriter.string(this.data);
    }
  }

  class Time {
    constructor(sec, nanosec) {
      this.sec = sec;
      this.nanosec = nanosec;
    }
    static decode(cdrReader) {
      const sec = cdrReader.uint32();
      const nanosec = cdrReader.uint32();
      return new Time(sec, nanosec);
    }
    encode(cdrWriter) {
      cdrWriter.uint32(this.sec);
      cdrWriter.uint32(this.nanosec);
    }
  }

  class Header {
    constructor(stamp, frame_id) {
      this.stamp = stamp;
      this.frame_id = frame_id;
    }
    static decode(cdrReader) {
      const stamp = Time.decode(cdrReader);
      const frame_id = cdrReader.string();
      return new Header(stamp, frame_id);
    }
    encode(cdrWriter) {
      this.stamp.encode(cdrWriter);
      cdrWriter.string(this.frame_id);
    }
  }

  class Vector3 {
    constructor(x, y, z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
    static decode(cdrReader) {
      const x = cdrReader.float64();
      const y = cdrReader.float64();
      const z = cdrReader.float64();
      return new Vector3(x, y, z);
    }
    encode(cdrWriter) {
      cdrWriter.float64(this.x);
      cdrWriter.float64(this.y);
      cdrWriter.float64(this.z);
    }
  }

  class Twist {
    constructor(linear, angular) {
      this.linear = linear;
      this.angular = angular;
    }
    static decode(cdrReader) {
      const linear = Vector3.decode(cdrReader);
      const angular = Vector3.decode(cdrReader);
      return new Twist(linear, angular);
    }
    encode(cdrWriter) {
      this.linear.encode(cdrWriter);
      this.angular.encode(cdrWriter);
    }
  }

  class TwistStamped {
    constructor(header, twist) {
      this.header = header;
      this.twist = twist;
    }
    static decode(cdrReader) {
      const header = Header.decode(cdrReader);
      const twist = Twist.decode(cdrReader);
      return new TwistStamped(header, twist);
    }
    encode(cdrWriter) {
      this.header.encode(cdrWriter);
      this.twist.encode(cdrWriter);
    }
  }

  class UInt64 {
    constructor(data) {
      this.data = data;
    }
    static decode(cdrReader) {
      const data = cdrReader.uint64();
      return new UInt64(data);
    }
    encode(cdrWriter) {
      cdrWriter.uint64(this.data);
    }
  }

  class CommandTimestamp {
    constructor(command_id, timestamp_index, timestamp) {
      this.command_id = command_id;
      this.timestamp_index = timestamp_index;
      this.timestamp = timestamp;
    }
    static decode(cdrReader) {
      const command_id = cdrReader.uint64();
      const timestamp_index = cdrReader.uint64();
      const timestamp = cdrReader.uint64();
      return new CommandTimestamp(command_id, timestamp_index, timestamp);
    }
    encode(cdrWriter) {
      cdrWriter.uint64(this.command_id);
      cdrWriter.uint64(this.timestamp_index);
      cdrWriter.uint64(this.timestamp);
    }
  }

  useEffect(() => {
    const chatUrl = rest_api + scope + TOPIC_CHAT;
    const chatEventSource = new EventSource(chatUrl);

    chatEventSource.addEventListener("PUT", (e) => {
      const sample = JSON.parse(e.data);
      const reader = new CdrReader(Uint8Array.from(atob(sample['value']), c => c.charCodeAt(0)));
      const receivedMessage = String.decode(reader).data;
      setMessages((prevMessages) => [...prevMessages, `ROS2: ${receivedMessage}`]);
    });

    return () => {
      chatEventSource.close();
    };
  }, []);

  useEffect(() => {
    const timestampUrl = rest_api + scope + TOPIC_TIMESTAMP;
    const timestampEventSource = new EventSource(timestampUrl);

    timestampEventSource.addEventListener("PUT", (e) => {
      const sample = JSON.parse(e.data);
      console.log('Received timestamp: ' + sample['value']);
      const buffer = Buffer.from(sample["value"], "base64");
      const reader = new CdrReader(buffer);
      const receivedTimestamp = CommandTimestamp.decode(reader);
      const commandId = receivedTimestamp.command_id;
      const commandTimestampIndex = receivedTimestamp.timestamp_index;
      const commandTimestamp = receivedTimestamp.timestamp;
      console.log('Received command ID: ' + commandId + ', timestamp index: ' + commandTimestampIndex + ', timestamp: ' + commandTimestamp);
      setCommandTimestamps(prev => {
        const updated = new Map(prev);
        if (!updated.has(commandId)) {
          updated.set(commandId, { T1: null, T2: null, T3: null, T4: null, T5: null, T6: null });
        }
        updated.get(commandId)['T' + commandTimestampIndex] = commandTimestamp;
        return updated;
      });
    });

    return () => {
      timestampEventSource.close();
    }
  }, []);

  useEffect(() => {
    const commandCompletionUrl = rest_api + scope + TOPIC_COMMAND_COMPLETION;
    const commandCompletionEventSource = new EventSource(commandCompletionUrl);

    commandCompletionEventSource.addEventListener("PUT", (e) => {
      const sample = JSON.parse(e.data);
      console.log('Received command completion: ' + sample['value']);
      const buffer = Buffer.from(sample["value"], "base64");
      const reader = new CdrReader(buffer);
      const receivedCommandId = UInt64.decode(reader).data;
      console.log('Received command completion ID: ' + receivedCommandId);
      setCommandTimestamps(prev => {
        const updated = new Map(prev);
        if (updated.has(receivedCommandId)) {
          updated.get(receivedCommandId).T6 = Date.now() * 1000000; // Current time in nanoseconds
          console.log(`Added T6 to command ${receivedCommandId}: ${updated.get(receivedCommandId).T6}`);
        }
        trimCommandTimestamps();
        return updated;
      });
    });

    return () => {
      commandCompletionEventSource.close();
    }
  }, []);


  const sendMessage = () => {
    const url = rest_api + scope + TOPIC_CHAT;
    const writer = new CdrWriter();
    const message = new String(messageInput);
    message.encode(writer);
    const encodedMessage = writer.data;

    fetch(url, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/octet-stream'
      },
      body: encodedMessage
    })
    .then(response => {
      if (response.ok) {
        setMessages((prevMessages) => [...prevMessages, `You: ${messageInput}`]);
        setMessageInput('');
      }
    })
    .catch(error => console.error("Error sending message:", error));
  };

  // Function to trim commandTimestamps if size exceeds MAX_COMMANDS
  const trimCommandTimestamps = () => {
    if (commandTimestamps.size > MAX_COMMANDS) {
      const oldestCommandId = commandTimestamps.keys().next().value;
      setCommandTimestamps(prev => {
        const updated = new Map(prev);
        updated.delete(oldestCommandId);
        return updated;
      });
      console.log('Deleted oldest command ID: ' + oldestCommandId);
    }
  };

  // Function to download the commandTimestamps as CSV
  const downloadCommandTimestamps = () => {
    let csv = 'T0,T1,T2,T3,T4,T5,T6\n';
    commandTimestamps.forEach((timestamps, commandId) => {
      csv += `${commandId},${timestamps.T1},${timestamps.T2},${timestamps.T3},${timestamps.T4},${timestamps.T5},${timestamps.T6}\n`;
    });

    const hiddenElement = document.createElement('a');
    hiddenElement.href = 'data:text/csv;charset=utf-8,' + encodeURI(csv);
    hiddenElement.target = '_blank';
    hiddenElement.download = 'commandTimestamps.csv';
    hiddenElement.click();
  };

  // Simulate command clicks
  const testSimulateCommandClicks = () => {
    const commands = [
      { linear: 1.0, angular: 0.0 },
      { linear: 0.0, angular: 1.0 },
      { linear: -1.0, angular: 0.0 },
      { linear: 0.0, angular: -1.0 },
      { linear: 0.0, angular: 0.0 }
    ];

    const startTime =  Date.now();
    const interval = setInterval(() => {
      const currentTime = Date.now();
      // Stop after 5min (300000ms)
      if (currentTime - startTime > 300000) {
        clearInterval(interval);
        return;
      }

      const randomIndex = Math.floor(Math.random() * commands.length);
      const command = commands[randomIndex];
      pubTwistStamped(command.linear, command.angular);
    }, 1000); // Interval set for Chrome click recording
  };

  const pubTwistStamped = (linear, angular) => {
    const url = rest_api + scope + TOPIC_DRIVE;
    const writer = new CdrWriter();
    const now = new Date();
    const header = new Header(new Time(Math.floor(now.getTime() / 1000), now.getMilliseconds() * 1000000), "base_link");
    const twist = new Twist(new Vector3(linear, 0.0, 0.0), new Vector3(0.0, 0.0, angular));
    const twistStamped = new TwistStamped(header, twist);
    twistStamped.encode(writer);
    const encodedTwistStamped = writer.data;

    fetch(url, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/octet-stream'
      },
      body: encodedTwistStamped
    }).then(response => {
      if (!response.ok) {
        console.error("Error sending TwistStamped:", response);
      }else{
        console.log("TwistStamped sent successfully", response);
      }
    }).catch(error => console.error("Error sending TwistStamped:", error));
  };

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

      {/* Camera ROS Section */}
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

      {/* Drive Controls Section */}
      <div className="drive-section">
        <header className="header" onClick={() => setDriveVisible(!driveVisible)}>
          <h5>Drive</h5>
          <i className='fas fa-gamepad'></i>
        </header>
        {driveVisible && (
          <div className="drive-controls">
            <button className="drive-button" onMouseDown={() => pubTwistStamped(1.0, 0.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
              <i className='fas fa-caret-up'></i>
            </button>
            <div className="drive-middle-row">
              <button className="drive-button" onMouseDown={() => pubTwistStamped(0.0, 1.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
                <i className='fas fa-caret-left'></i>
              </button>
              <button className="drive-button stop" onClick={() => pubTwistStamped(0.0, 0.0)}>
                <i className='fas fa-stop-circle'></i>
              </button>
              <button className="drive-button" onMouseDown={() => pubTwistStamped(0.0, -1.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
                <i className='fas fa-caret-right'></i>
              </button>
            </div>
            <button className="drive-button" onMouseDown={() => pubTwistStamped(-1.0, 0.0)} onMouseUp={() => pubTwistStamped(0.0, 0.0)}>
              <i className='fas fa-caret-down'></i>
            </button>
          </div>
        )}
      </div>
      {/* Command Timestamps Section */}
      <div className="command-timestamps-section">
        <header className="header">
          <h5>Command Timestamps</h5>
          <button onClick={testSimulateCommandClicks}>Test</button>
          <button onClick={downloadCommandTimestamps}>Download</button>
        </header>
        <div className="command-timestamps">
          <table>
            <thead>
              <tr>
                <th>T0</th>
                <th>T1</th>
                <th>T2</th>
                <th>T3</th>
                <th>T4</th>
                <th>T5</th>
                <th>T6</th>
              </tr>
            </thead>
            <tbody>
              {Array.from(commandTimestamps).map(([commandId, timestamps]) => (
                <tr key={commandId}>
                  <td>{commandId}</td>
                  <td>{timestamps.T1}</td>
                  <td>{timestamps.T2}</td>
                  <td>{timestamps.T3}</td>
                  <td>{timestamps.T4}</td>
                  <td>{timestamps.T5}</td>
                  <td>{timestamps.T6}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default TeleopInterface;
