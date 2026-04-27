# optimal_traj

**Clohessy-Wiltshire(CW) 방정식** 기반 위성 근접 랑데부(Rendezvous) 최소 추력 최적 궤적 계산.  
Deputy 위성이 Chief 위성에 도달하는 연료 최소화 문제를 수치적으로 풀이.

---

## 주요 내용

- **좌표 변환**: ECI → RTN(Hill) 프레임 상대 상태 변환
- **CW 방정식**: 근거리 상대 운동 선형 모델 (`scipy.linalg.expm` 활용)
- **최적 제어**: 이산 선형 최적 제어 문제 수식화 — 추력 최소화 목적함수 수치 풀이

---

## 📄 파일

| 파일 | 내용 |
|------|------|
| `CW_Rendezvous.ipynb` | ECI 초기 조건 입력 → RTN 변환 → CW 기반 최적 궤적 계산 및 시각화 |

---

## ▶️ 실행 방법

```bash
pip install numpy scipy matplotlib
jupyter notebook CW_Rendezvous.ipynb
```

---

## 상태 벡터 정의

$$x = [x,\ y,\ z,\ \dot{x},\ \dot{y},\ \dot{z}]^T \quad \text{(RTN 프레임)}$$

- $(x, y, z)$: Deputy의 Chief에 대한 상대 위치 [m]  
- $x$축: Radial, $y$축: Transverse, $z$축: Normal
