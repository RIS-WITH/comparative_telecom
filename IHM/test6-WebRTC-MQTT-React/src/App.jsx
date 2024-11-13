import React, { useState, useEffect } from 'react';
import mqtt from 'mqtt';
import mqttSettings from './settings';

const App = () => {
  const [connected, setConnected] = useState(false);
  const [messages, setMessages] = useState([]);
  const [client, setClient] = useState(null);

  useEffect(() => {
    const mqttClient = mqtt.connect(mqttSettings.url, mqttSettings.options);
    setClient(mqttClient);

    mqttClient.on('connect', () => {
      console.log('connected');
      setConnected(true);
      mqttClient.subscribe('presence', (err) => {
        if (!err) {
          console.log('subscribed');
          mqttClient.publish('presence', 'Hello mqtt');
        }
      });
    });

    mqttClient.on('message', (topic, message) => {
      console.log('message', topic, message.toString());
      setMessages((prevMessages) => [...prevMessages, message.toString()]);
    });

    mqttClient.on('close', () => {
      console.log('close');
      setConnected(false);
    });

    // Cleanup on component unmount
    return () => {
      mqttClient.end();
    };
  }, []);

  const handlePublish = () => {
    if (client) {
      client.publish('presence', 'Hello mqtt');
    }
  };

  const handleDisconnect = () => {
    if (client) {
      client.end();
    }
  };

  return (
    <main>
      <h1>MQTTjs VITE Example</h1>
      <p>
        This is a simple example of using the MQTT.js library with Vite and
        React.
      </p>
      <p>
        The client connects to a public MQTT broker at{' '}
        <code>ws://yunobo:9001</code> using the <code>mummer</code> username and
        password.
      </p>
      <p>Status: {connected ? 'Connected' : 'Disconnected'}</p>
      <p>
        <button onClick={handlePublish}>Publish</button>
        <button onClick={handleDisconnect}>Disconnect</button>
      </p>
      <p>Messages:</p>
      <ul>
        {messages.map((message, index) => (
          <li key={index}>{message}</li>
        ))}
      </ul>
    </main>
  );
};

export default App;
