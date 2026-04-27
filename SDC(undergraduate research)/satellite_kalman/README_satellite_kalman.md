# satellite_kalman

`satellite_6dof` 시뮬레이션 결과를 입력으로 받아 **확장 칼만 필터(EKF)**로 위성의 자세(쿼터니언)와 자이로 바이어스를 추정하는 알고리즘.

---

## 주요 기능

- **센서 노이즈 모델링**: 자이로 ARW(각도 랜덤 워크) + RRW(레이트 랜덤 워크) 바이어스
- **EKF 예측(Prediction)**: 쿼터니언 운동학 기반 상태 전파, 선형화 상태 전이 행렬 F 계산
- **EKF 보정(Update)**: 태양 벡터 / Nadir 벡터 / 지상국 방향 벡터 관측 활용
- **모드 전환 대응**: Sun ↔ GS 전환 시 Q 행렬 및 이상치 기각(Outlier Rejection) 임계값 동적 조정
- **수치 안정성**: Joseph Form 공분산 업데이트
- **성능 평가**: 전체 / Sun 모드 / GS 모드별 RMS·평균 자세 오차(deg) 출력

---

## 📁 구조

```
src/
├── main.m                      # 메인 EKF 실행 스크립트
├── my_satellite_data.csv       # satellite_6dof 출력 데이터 (입력값)
├── ekf_function/
│   ├── compute_F.m             # 상태 전이 행렬 F (쿼터니언 선형화)
│   └── jacobian_h_q.m         # 측정 모델 야코비안 H
├── dynamics/                   # 동역학 보조 함수
├── quaternion_math/            # 쿼터니언 연산
├── dcm/                        # DCM 변환
├── ode/                        # Runge-Kutta 적분기
└── sun_function/               # 태양 위치 계산
```

---

## ▶️ 실행 방법

1. `satellite_6dof/src/main.m` 실행 후 생성된 `my_satellite_data.csv`를 `src/`에 위치
2. MATLAB에서 `src/main.m` 실행

---

## 상태 벡터 정의

$$x = [q_x,\ q_y,\ q_z,\ q_w,\ \beta_x,\ \beta_y,\ \beta_z]^T$$

- $q$: 자세 쿼터니언 (JPL Notation)  
- $\beta$: 자이로 바이어스 [rad/s]
