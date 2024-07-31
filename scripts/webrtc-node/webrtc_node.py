#!/usr/bin/env python3

import argparse
import asyncio
import logging

import rclpy
from rclpy.executors import MultiThreadedExecutor
from aiortc.contrib.media import MediaRecorder

from comparative_telecom.WebRTC.Janus.janus_session import JanusSession
from comparative_telecom.WebRTC.Janus.webrtc_publisher import publish, subscribe
from comparative_telecom.WebRTC.ROS_link.ros_video_stream import VideoStreamTrackFromROS, ROSVideoStream

pcs = set()

async def run(video_track, recorder, room, session):
    await session.create()

    plugin = await session.attach("janus.plugin.videoroom")
    response = await plugin.send(
        {
            "body": {
                "display": "Robot",
                "ptype": "publisher",
                "request": "join",
                "room": room,
            }
        }
    )
    publishers = response["plugindata"]["data"]["publishers"]
    for publisher in publishers:
        print("id: %(id)s, display: %(display)s" % publisher)

    await publish(plugin=plugin, video_track=video_track)

    if recorder is not None and publishers:
        await subscribe(
            session=session, room=room, feed=publishers[0]["id"], recorder=recorder
        )

    print("Exchanging media")
    await asyncio.sleep(600)

def main(args):
    rclpy.init(args=None)
    video_track = VideoStreamTrackFromROS()
    node = ROSVideoStream(video_track)

    session = JanusSession(args.url)
    recorder = MediaRecorder(args.record_to) if args.record_to else None

    # Utiliser un exécuteur multithreadé pour permettre à rclpy de fonctionner de manière asynchrone
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()
    try:
        loop.run_in_executor(None, executor.spin)
        loop.run_until_complete(
            run(video_track=video_track, recorder=recorder, room=args.room, session=session)
        )
    except KeyboardInterrupt:
        pass
    finally:
        if recorder is not None:
            loop.run_until_complete(recorder.stop())
        loop.run_until_complete(session.destroy())

        coros = [pc.close() for pc in pcs]
        loop.run_until_complete(asyncio.gather(*coros))
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Janus")
    parser.add_argument("--url", help="Janus root URL, e.g. http://raikou:8088/janus", default="http://raikou.laas.fr:8088/janus")
    parser.add_argument(
        "--room",
        type=int,
        default=1234,
        help="The video room ID to join (default: 1234).",
    )
    parser.add_argument("--record-to", help="Write received media to a file.")
    parser.add_argument("--verbose", "-v", action="count")
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    main(args)
