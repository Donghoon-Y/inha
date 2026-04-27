# satellite_6dof

인공위성의 궤도 및 자세 운동을 MATLAB으로 구현한 **6자유도(6-DoF) 시뮬레이션**.  
인하대학교 항공우주산학융합원(위도 37.23°, 경도 126.39°)을 지상국으로 설정하여 실제 운용 시나리오를 모델링.

---

## 주요 기능

- **궤도 전파**: Two-Body 모델 + Runge-Kutta 4차 적분기
- **좌표 변환**: ECI ↔ ECEF ↔ PQW, DCM 및 쿼터니언(JPL Notation)
- **자세 제어 (PD 제어기)**
  - `Sun-pointing 모드`: 태양 방향 지향 (충전)
  - `Ground-pointing 모드`: 지상국 가시 구간 안테나 추적
  - `Detumbling`: 초기 각속도 수렴
- **시각화**: 3D 궤도, Ground Track, 자세 오차, Quaternion 이력, MATLAB Satellite Scenario Viewer
- **CSV 출력**: EKF 입력용 `my_satellite_data.csv` 저장 (21열)

---

## 구조

```
src/
├── main.m                      # 메인 실행 스크립트
├── dynamics/                   # 궤도·자세 동역학
├── quaternion_math/            # 쿼터니언 연산
├── dcm/                        # DCM 변환
├── ode/                        # Runge-Kutta 적분기
├── sun_function/               # UTC 기반 태양 위치 계산
└── plus/                       # 관성 모멘트 등 보조 함수

report/
├── 결과보고서/                  # 동계 학부연구생 최종 보고서
└── 발표자료/                   # 1~4차 발표 PPT
```

---

## 실행 방법

MATLAB에서 `src/main.m` 실행.  
Aerospace Toolbox 필요 (`eci2ecef`, `ecef2lla`, `satelliteScenario` 등 사용).

---

## CSV 출력 형식

| 열 | 내용 |
|----|------|
| 1 | 시간 [s] |
| 2–5 | 쿼터니언 [qx qy qz qw] |
| 6–8 | 각속도 [rad/s] |
| 9–11 | 태양 벡터 ECI (단위벡터) |
| 12–14 | 위성 위치 ECI [km] |
| 15–17 | 지상국 위치 ECI [m] |
| 18 | 제어 모드 (0=Sun, 1=GS) |
| 19–21 | 목표 각속도 ECI [rad/s] |
