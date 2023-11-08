# Project1-Build3

## Fire detector with camera for in the woods

In this project, I connected a waterproof temperature sensor and a OV2640 camera to my ESP32-S3.

The purpose of this project is that the system would be in the woods and if the temperatures rises above 25 °C, it would show a live feed of the view of the camera. 

All the data is send to Node Red over MQTT. I made a dashboard in Node Red to visualise the temperature and a link to the live stream. 
In Node Red, I added a button to stop the live stream so the esp32 would exit its while loop and go into deepsleep.
![Schermafbeelding 2023-11-06 om 00 27 04](https://github.com/waut10000/Project1-Build3/assets/133114632/1ee227ea-453d-4e74-b392-6f4a146b255e)


Code explenation
In the loop(): The temperature is measured, send to Node Red with MQTT. Then it checks IF the temperature is above 25°C. 
- yes : activate live stream and go in a while loop that measures and sends the temperature every minute. When the buttin is pressed on the dashboard, false is send to the esp32 and exits the while loop. It ends the stream and go into deepsleep.
- no : Wait 5 seconds and go into deepsleep.
