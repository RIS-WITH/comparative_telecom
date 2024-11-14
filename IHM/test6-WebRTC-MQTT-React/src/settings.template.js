// settings.template.js
const mqttSettings = {
  url: 'ws://localhost:9001', // URL of the MQTT broker
  options: {
    username: 'username',
    password: 'password',
    keepalive: 10, // Keep-alive interval in seconds
    will: {
      topic: 'chatter/std_msgs/String',
      payload: 'Goodbye!',
      qos: 0,
      retain: false,
    },
  },
};

export default mqttSettings;