# Tests Réseau

Cette section contient les tests réseau effectués sur le projet. Actuellement, elle inclut des tests de latence sur le réseau local en utilisant les middlewares suivants :
- DDS de ROS2 (FastRTPS)
- FastDDS
- CycloneDDS
- Zenoh

Pour la communication avec le site web :
- ROSWebTools rosbridge_webserver - roslibjs et apache2
- RestAPI - zenoh, react, et react nextjs server

## Prérequis
- apache2
- ROS2 Iron (Packages : CycloneDDS, FastDDS, Rosbridge)
- Zenoh
- MQTT mosquitto

### Tests
- Pour savoir comment lancer les tests du serveur Rosbridge avec les différents middlewares, consultez [Test 1 : ROSWebTools](test1-ROSWebTools/ReadMe.md)
- Pour savoir comment lancer les tests du serveur RestAPI avec Zenoh, consultez [Test 2 : Zenoh](test5-Zenoh-React/ReadMe.md)
- Pour savoir comment lancer les tests du MQTT, consultez [Test 3 : MQTT](test6-WebRTC-MQTT-React/ReadMe.md)