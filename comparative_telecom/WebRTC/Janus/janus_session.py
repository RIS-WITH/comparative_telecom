import random
import string
import asyncio
import aiohttp
import logging
import time

# Configurer la journalisation
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)

def transaction_id():
    return "".join(random.choice(string.ascii_letters) for x in range(12))

class JanusPlugin:
    def __init__(self, session, url):
        self._queue = asyncio.Queue()
        self._session = session
        self._url = url

    async def send(self, payload):
        message = {"janus": "message", "transaction": transaction_id()}
        message.update(payload)

        # logger.debug(f"Plugin send message: {message}")
        async with self._session._http.post(self._url, json=message) as response:
            data = await response.json()
            # logger.debug(f"Received response: {data}")
            assert data["janus"] == "ack"

        response = await self._queue.get()
        # logger.debug(f"Plugin queue response: {response}")
        assert response["transaction"] == message["transaction"]
        return response

class JanusSession:
    def __init__(self, url):
        self._http = None
        self._poll_task = None
        self._plugins = {}
        self._root_url = url
        self._session_url = None

    async def attach(self, plugin_name: str) -> JanusPlugin:
        message = {
            "janus": "attach",
            "plugin": plugin_name,
            "transaction": transaction_id(),
        }
        # logger.debug(f"Attaching plugin with message: {message}")
        async with self._http.post(self._session_url, json=message) as response:
            data = await response.json()
            # logger.debug(f"Attach response: {data}")
            assert data["janus"] == "success"
            plugin_id = data["data"]["id"]
            plugin = JanusPlugin(self, self._session_url + "/" + str(plugin_id))
            self._plugins[plugin_id] = plugin
            return plugin

    async def create(self):
        self._http = aiohttp.ClientSession()
        message = {"janus": "create", "transaction": transaction_id()}
        # logger.debug(f"Creating session with message: {message}")
        async with self._http.post(self._root_url, json=message) as response:
            data = await response.json()
            # logger.debug(f"Create session response: {data}")
            assert data["janus"] == "success"
            session_id = data["data"]["id"]
            self._session_url = self._root_url + "/" + str(session_id)

        self._poll_task = asyncio.ensure_future(self._poll())

    async def destroy(self):
        if self._poll_task:
            self._poll_task.cancel()
            self._poll_task = None

        if self._session_url:
            message = {"janus": "destroy", "transaction": transaction_id()}
            # logger.debug(f"Destroying session with message: {message}")
            async with self._http.post(self._session_url, json=message) as response:
                data = await response.json()
                # logger.debug(f"Destroy session response: {data}")
                assert data["janus"] == "success"
            self._session_url = None

        if self._http:
            await self._http.close()
            self._http = None

    async def _poll(self):
        while True:
            try:
                params = {"maxev": 1, "rid": int(time.time() * 1000)}
                async with self._http.get(self._session_url, params=params) as response:
                    data = await response.json()
                    # logger.debug(f"Polling response: {data}")
                    if data["janus"] == "event":
                        plugin = self._plugins.get(data["sender"], None)
                        if plugin:
                            await plugin._queue.put(data)
                        else:
                            logger.warning(f"Received event for unknown plugin: {data}")
                    else:
                        logger.warning(f"Unexpected polling data: {data}")
            except Exception as e:
                logger.error(f"Polling error: {e}")
                await asyncio.sleep(1)  # Wait a bit before retrying
