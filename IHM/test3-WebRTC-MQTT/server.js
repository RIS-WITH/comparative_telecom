const Janode = require('janode');
const WebSocket = require('ws');
const mqtt = require('mqtt');
const wss = new WebSocket.Server({ port: 9090 });

const janode = new Janode();

// Configurez le client MQTT
const mqttClient = mqtt.connect('mqtt://localhost:1883');

mqttClient.on('connect', () => {
    console.log('Connected to MQTT broker');
    mqttClient.subscribe('chat');
});

mqttClient.on('message', (topic, message) => {
    if (topic === 'chat') {
        const chatMessage = message.toString();
        wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify({ chat: chatMessage }));
            }
        });
    }
});

wss.on('connection', (ws) => {
    console.log('Client connected');

    ws.on('message', async (message) => {
        try {
            const data = JSON.parse(message);
            if (data.offer) {
                janode.receiveOffer(ws, data.offer);
            } else if (data.answer) {
                janode.receiveAnswer(ws, data.answer);
            } else if (data.candidate) {
                janode.receiveCandidate(ws, data.candidate);
            } else if (data.chat) {
                mqttClient.publish('chat', data.chat);
            }
        } catch (error) {
            console.error('Error processing message:', error);
        }
    });

    janode.on('offer', (offer) => {
        ws.send(JSON.stringify({ offer }));
    });

    janode.on('answer', (answer) => {
        ws.send(JSON.stringify({ answer }));
    });

    janode.on('candidate', (candidate) => {
        ws.send(JSON.stringify({ candidate }));
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

console.log('WebSocket server running on ws://localhost:9090');
