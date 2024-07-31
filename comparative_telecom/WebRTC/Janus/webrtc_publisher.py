import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription,VideoStreamTrack
from .janus_session import JanusPlugin

pcs = set()

async def publish(plugin: JanusPlugin, video_track):
    pc = RTCPeerConnection()
    pcs.add(pc)

    media = {"audio": False, "video": True}
    if video_track:
        pc.addTrack(video_track)
    else:
        pc.addTrack(VideoStreamTrack())

    await pc.setLocalDescription(await pc.createOffer())
    print("local description: ",pc.localDescription)
    request = {"request": "configure"}
    request.update(media)
    print("request: ",request)
    response = await plugin.send(
        {
            "body": request,
            "jsep": {
                "sdp": pc.localDescription.sdp,
                "trickle": False,
                "type": pc.localDescription.type,
            },
        }
    )
    print("response: ",response)
    await pc.setRemoteDescription(
        RTCSessionDescription(
            sdp=response["jsep"]["sdp"], type=response["jsep"]["type"]
        )
    
    )

async def subscribe(session, room, feed, recorder):
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("track")
    async def on_track(track):
        print("Track %s received" % track.kind)
        if track.kind == "video":
            recorder.addTrack(track)
        if track.kind == "audio":
            recorder.addTrack(track)

    plugin = await session.attach("janus.plugin.videoroom")
    response = await plugin.send(
        {"body": {"request": "join", "ptype": "subscriber", "room": room, "feed": feed}}
    )

    await pc.setRemoteDescription(
        RTCSessionDescription(
            sdp=response["jsep"]["sdp"], type=response["jsep"]["type"]
        )
    )

    await pc.setLocalDescription(await pc.createAnswer())
    response = await plugin.send(
        {
            "body": {"request": "start"},
            "jsep": {
                "sdp": pc.localDescription.sdp,
                "trickle": False,
                "type": pc.localDescription.type,
            },
        }
    )
    await recorder.start()
