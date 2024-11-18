# React + Vite

This template provides a minimal setup to get React working in Vite with HMR and some ESLint rules.

Currently, two official plugins are available:

- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react/README.md) uses [Babel](https://babeljs.io/) for Fast Refresh
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh

## Usage

### How to start

```bash
npm install
npm run dev
```

### How to build

```bash
npm run build
```

### How to preview the build output

```bash
npm run preview -- --host
```

## What changes from roswebtools architecture
- We don't need to run nodes anymore, we just need to run the server and the client of the MQTT.
- Make sure mosquitto is running on the server with the port websocket enabled and add a user and password to the server.
- change the settings template in the `src` folder to your server settings and rename it to `settings.js`.
- Copy the mqtt script to Yunobo and run it (no need to run anything else on Loki other than the host server).
- You will need the robot_server.py to also be running on the robot.