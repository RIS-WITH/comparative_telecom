// Sélectionner les éléments video et canvas
const video = document.getElementById('video');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// Fonction pour démarrer le stream de la webcam
async function startStream() {
    try {
        // Demander l'accès à la webcam
        const stream = await navigator.mediaDevices.getUserMedia({ video: true });
        // Assigner le stream à l'élément video
        video.srcObject = stream;
        
        // Démarrer la fonction d'affichage sur canvas
        requestAnimationFrame(updateCanvas);
    } catch (error) {
        console.error('Erreur lors de l\'accès à la webcam :', error);
    }
}

// Fonction pour dessiner la vidéo sur le canvas
function updateCanvas() {
    // Définir les dimensions du canvas en fonction de la vidéo
    canvas.width = video.videoWidth;
    canvas.height = video.videoHeight;
    
    // Dessiner l'image actuelle de la vidéo sur le canvas
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    
    // Continuer l'animation
    requestAnimationFrame(updateCanvas);
}

// Démarrer le stream lorsque la page est chargée
window.addEventListener('load', startStream);
