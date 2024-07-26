const WebSocket = require('ws');
const server = new WebSocket.Server({ port: 9090 });

server.on('connection', socket => {
    socket.on('message', message => {
        server.clients.forEach(client => {
            if (client !== socket && client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });
});

console.log('WebSocket signaling server running on ws://localhost:9090');
