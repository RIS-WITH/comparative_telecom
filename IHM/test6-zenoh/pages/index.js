import React, { useState, useEffect } from 'react';
import { CdrReader, CdrWriter } from "@foxglove/cdr";


const ChatInterface = () => {
  const [messages, setMessages] = useState([]);
  const [messageInput, setMessageInput] = useState('');
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
  
  useEffect(() => {
    // Set up the EventSource for message subscription
    const url = "http://140.93.6.33:8800/3/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    const key = "3/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    const eventSource = new EventSource(url);

    
    eventSource.addEventListener("PUT", (e) => {
      console.log("Received message: " + e.data);
      const sample = JSON.parse(e.data);
      if (sample['key'] == key) {
        const reader = new CdrReader(Uint8Array.from(atob(sample['value']), c => c.charCodeAt(0)));
        const receivedMessage = String.decode(reader).data;
        setMessages((prevMessages) => [...prevMessages, `ROS2: ${receivedMessage}`]);
      }
    });

    // Cleanup function to close the connection when the component unmounts
    return () => {
      eventSource.close();
    };
  }, []);

  const sendMessage = () => {
    console.log("Send message: " + messageInput);
    const url = "http://140.93.6.33:8800/3/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

    const writer = new CdrWriter();
    const message = new String(messageInput);
    message.encode(writer);
    const encodedMessage = writer.data;

    fetch(url, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/octet-stream'
      },
      body: encodedMessage // Send the encoded message
    })
    .then(response => {
      if (response.ok) {
        setMessages((prevMessages) => [...prevMessages, `You: ${messageInput}`]);
        setMessageInput('');
      }
    })
    .catch(error => console.error("Error sending message:", error));
  };

  return (
    <div className="w3-card-4 w3-margin-bottom">
      <header className="w3-bar w3-green">
        <h5 className="w3-bar-item" style={{ margin: 0 }}>Chat</h5>
      </header>
      <div className="w3-container w3-padding">
        <div id="messages" style={{ height: '200px', overflowY: 'auto', border: '1px solid #ccc', padding: '10px' }}>
          {messages.map((msg, index) => (
            <div key={index}>{msg}</div>
          ))}
        </div>
        <input
          type="text"
          placeholder="Type your message here..."
          value={messageInput}
          onChange={(e) => setMessageInput(e.target.value)}
          style={{ width: 'calc(100% - 22px)', padding: '10px', marginTop: '10px' }}
        />
        <button
          onClick={sendMessage}
          style={{
            width: '100%',
            padding: '10px',
            backgroundColor: '#4CAF50',
            color: 'white',
            border: 'none',
            cursor: 'pointer',
            marginTop: '10px'
          }}
        >
          Send
        </button>
      </div>
    </div>
  );
};

export default ChatInterface;
