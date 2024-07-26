const Janode = require('janode');
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 9090 });

const janode = new Janode();

wss.on('connection', (ws) => {
    console.log('Client connected');

    ws.on('message', async (message) => {
        try {
            const data = JSON.parse(message);
            if (data.offer) {
                // Transmet l'offre au client distant
                janode.receiveOffer(ws, data.offer);
            } else if (data.answer) {
                // Transmet la réponse au client distant
                janode.receiveAnswer(ws, data.answer);
            } else if (data.candidate) {
                // Transmet le candidat ICE au client distant
                janode.receiveCandidate(ws, data.candidate);
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
