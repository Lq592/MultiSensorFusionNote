



其中：$\Omega$ 可以参考四元数微分　ref. <<Indirect Kalman Filter for 3D Attitude Estimation>>　

![](/home/pi-sz/Desktop/msckf_note/img/Screenshot%20from%202019-03-20%2014-44-39.png)













![1553680544348](/home/pi-sz/Desktop/msckf_note/images/1553680544348.png)

Jacobian的计算有很多方法。这里使用链式法则  ![1553681104224](/home/pi-sz/Desktop/msckf_note/images/1553681104224.png)

- $H_x$是regular EKF中的H，与观测矩阵有关
- $X_{\delta X}$ 是真实状态相对于误差状态的jacobian ，与ESKF的状态组成有关

![1553681410338](/home/pi-sz/Desktop/msckf_note/images/1553681410338.png)

- ![1553681482256](/home/pi-sz/Desktop/msckf_note/images/1553681482256.png)
- ![1553681601939](/home/pi-sz/Desktop/msckf_note/images/1553681601939.png)

![1553681783318](/home/pi-sz/Desktop/msckf_note/images/1553681783318.png)

