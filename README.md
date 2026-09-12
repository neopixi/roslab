# EKF-SLAM в ROS 2 и Gazebo

Учебная симуляция четырёхколёсной переднеприводной тележки. Робот движется по
прямой, обнаруживает препятствие лидаром, объезжает его и возвращается на
исходную траекторию.

Реализованы ROS 2 Jazzy, Gazebo Harmonic, EKF-SLAM, управление поворотом через
подтормаживание передних колёс и визуализация в RViz.

## Запуск

```bash
docker build -t aovp-ekf-slam:jazzy .
docker run --rm -it --name aovp-ekf-slam \
  --shm-size=1g -p 127.0.0.1:6080:6080 aovp-ekf-slam:jazzy
```

Открыть: [http://localhost:6080/vnc.html?autoconnect=1](http://localhost:6080/vnc.html?autoconnect=1).

## Отчёт

- [Готовый PDF](output/pdf/report.pdf)
- [Исходник LaTeX](report/report.tex)
