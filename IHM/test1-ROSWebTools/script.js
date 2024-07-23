// script.js

import mqttComponent_ from "./MqttComponent";

document.addEventListener('DOMContentLoaded', function() {
    const video = document.getElementById('backgroundVideo');
    const canvas = document.getElementById('videoCanvas');
    const context = canvas.getContext('2d');

    function resizeCanvas() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }

    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    video.addEventListener('play', function() {
        function drawVideo() {
            if (video.paused || video.ended) return;
            context.drawImage(video, 0, 0, canvas.width, canvas.height);
            requestAnimationFrame(drawVideo);
        }
        drawVideo();
    });
});



