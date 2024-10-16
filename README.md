# AHRS: Attitude Heading Reference System 
AHRS (Attitude Heading Reference System) 을 위한 자세 추정 알고리즘을 다루고 있습니다.  
AHRS를 위한 다양한 추정 알고리즘이 존재 하지만, 
자세추정 수식이 실제 Python으로 구현되는 내용을 이해함으로써 정밀한 알고리즘 개발을 위한 인싸이트를 제공하는 것을 목표로 합니다.

여기서 다루고 있는 AHRS는 다음과 같습니다.
1) Accumulation기반 자세 추정   
   - Euler angular rate accumulation
   
   - Quaternion rate accumulation
3) Vector observation기반 자세 추정
   
   - Madgwich filter
5) Complementary filter
6) Quaternion기반 Kalman filter
7) Euler angle 기반 Extende Kalman filter
8) Euler angle 기반 Unscented Kalman filter
9) Quaternion 기반 Unscented Kalman filter 
