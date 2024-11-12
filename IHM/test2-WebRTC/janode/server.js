import Janode from 'janode';
import WebSocket from 'ws';
import EchoTestPlugin from 'janode/plugins/echotest';
const { Logger } = Janode;

// Create a WebSocket server
const wss = new WebSocket.Server({ port: 9090 });

wss.on('connection', async (ws) => {
    console.log('Client connected');

    // Connect to the Janode server
    try {
        const connection = await Janode.connect({
            is_admin: false,
            address: {
                url: 'ws://yunobo:8188/',  // The URL of your Janode server
                apisecret: 'secret'           // API secret
            }
        });

        // Create a new session
        const session = await connection.create();

        // Attach EchoTestPlugin to the session
        const echoHandle = await session.attach(EchoTestPlugin);

        // Handle Janode events using the plugin
        echoHandle.on(Janode.EVENT.HANDLE_WEBRTCUP, () => Logger.info('WebRTC is up.'));
        echoHandle.on(Janode.EVENT.HANDLE_MEDIA, (evtdata) => Logger.info('Media event:', evtdata));
        echoHandle.on(Janode.EVENT.HANDLE_SLOWLINK, (evtdata) => Logger.info('Slow link event:', evtdata));
        echoHandle.on(Janode.EVENT.HANDLE_HANGUP, (evtdata) => Logger.info('Hangup event:', evtdata));
        echoHandle.on(Janode.EVENT.HANDLE_DETACHED, (evtdata) => Logger.info('Detached event:', evtdata));

        // Handle EchoTest plugin specific events
        echoHandle.on(EchoTestPlugin.EVENT.ECHOTEST_RESULT, (evtdata) => Logger.info('EchoTest result:', evtdata));

        // When an offer is received from the WebSocket client, send a response
        ws.on('message', async (message) => {
            try {
                const data = JSON.parse(message);

                if (data.offer) {
                    // Handle the offer received from the client (for example, starting a test)
                    const { jsep: answer } = await echoHandle.start({ video: true, jsep: data.offer });

                    // Send the answer back to the client
                    ws.send(JSON.stringify({ answer }));
                } else if (data.answer) {
                    // Handle the answer received from the client if necessary
                    // This would depend on your WebRTC flow
                    echoHandle.receiveAnswer(data.answer);
                } else if (data.candidate) {
                    // Handle ICE candidate if necessary
                    echoHandle.receiveCandidate(data.candidate);
                }
            } catch (error) {
                console.error('Error processing WebSocket message:', error);
            }
        });

        // Handle WebSocket closure (clean up if needed)
        ws.on('close', async () => {
            console.log('Client disconnected');
            await echoHandle.detach();
        });

    } catch (error) {
        console.error('Error connecting to Janode server:', error);
    }
});

console.log('WebSocket server running on ws://localhost:9090');
