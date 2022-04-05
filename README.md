# System Identification and Kalman Filter for the closed loop dynamics of a quadrotor

Given and input output mapping I use System Identification theory to estimate the lateral dynamics of a quadrotor. I use a Linear Grey-Box Model.
![image](https://user-images.githubusercontent.com/36279027/161842600-e83f041d-a22a-42c7-96c9-16fb660eb788.png)

I write a discrete time Linear Kalman Filter to estimate the vehicles lateral velocity, fusing MoCap with IMU.

![image](https://user-images.githubusercontent.com/36279027/161842971-627cdc8c-422f-4301-bd8c-fb9d72d5191b.png)
