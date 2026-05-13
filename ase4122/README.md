# Multirotor Sizing Methodology

멀티로터 드론의 **프로펠러 · 모터 · ESC · 배터리**를 자동으로 선정하고,  
공력 해석 · 암 후류 간섭 · 최적 암 길이 설계 · 지면효과까지 통합 분석하는 MATLAB 설계 도구입니다.

Biczyski et al. (2020) *"Multirotor Sizing Methodology with Flight Time Estimation"* 의 방법론을 기반으로 하며,  
전진비행 해석(Pollet et al., ICAS 2021), Leng et al. 비축 입사각 모델, 렘니스케이트(역물방울) 단면 기반 암 설계 모듈이 추가되었습니다.

---

## 임무별 설계 방향

세 가지 미션 목표를 기반으로 설계 방향을 도출합니다.

| 미션 | 목표 | 설계 전략 |
|------|------|-----------|
| **Heavy Lift** | 탑재중량 증가에 따른 충분한 총 추력 및 여유 추력 확보 | 허용 Disk Loading 범위 내 최적 고추력 모터/프로펠러 선정 |
| **Endurance** | 동일 전력 대비 추력 극대화로 장시간 체공 실현 | 모터-프로펠러 조합 비추력(g/W)을 주요 지표로 소모 전력 최소화 |
| **Maneuverability** | 급격한 가속·감속 및 자세 제어를 위한 고응답성 확보 | 최대 추력 외 모터 응답성, 프로펠러 관성, 추중비 종합 고려 |

로터 수 Trade-off 분석(비행 안정성 vs 경량화)을 통해 **4개 로터 기반 쿼드콥터**를 기초 형상으로 선정하였습니다.

### 설계 1안 vs 2안

| 구분 | 목표 | 최적화 기준 |
|------|------|-------------|
| **설계 1안 (Endurance)** | 체공시간 최대화 | 호버링 비추력 (g/W) 최대 |
| **설계 2안 (Maneuverability)** | 왕복비행 시간 최소화 | 여유추력(T_surplus) 최대 → 가속도 최대 |

두 안 모두 동일한 후보 프롭 풀(`propList_considered`)에서 출발하며, 각자 독립적으로 최적 prop / motor / ESC를 선정합니다.

---

## 데이터베이스 구성

설계 코드는 세 가지 DB를 참조합니다.

### APC 프로펠러 DB (`APC_Prop/`)

| 파일 | 내용 |
|------|------|
| `WEBLIST.xlsx` | APC 프롭 전체 목록 (기종명, 파일명). 6행부터 데이터 |
| `PROP-DATA-FILE_202602.xlsx` | 직경·피치·무게 데이터 (2026년 2월 기준). `PRODUCT LIST` 시트 사용 |
| `PER2_STATIC-2.DAT` | 전 기종 정적(V=0) 성능 통합 데이터 (~300종). 추력(LBF)·전력(HP)·토크(IN-LBF) → 코드 내에서 g·W·Nm 변환 수행 |
| `PERFILES2/` | 기종별 개별 동적 성능 파일 (~300개). 전진비행 advance ratio J 기반 CT/CP 수록 |
| `PER2_RPMRANGE.DAT` | 기종별 데이터 수록 RPM 하한/상한 |

`PER2_STATIC-2.DAT` 구조:

```
====== APC Propeller Static Thrust VS. RPM Predictions =======

               105x45.dat                     ← 기종 구분자

    RPM       THRUST      POWER      TORQUE        Cp        Ct
               (LBF)       (HP)     (IN-LBF)

   1000         0.03        0.00       0.02       0.03      0.08
   ...
                                                           ← 빈 줄 = 다음 기종 시작
```

### 모터 DB (`motor_list/Motor_list.xlsx`)

**80% 쓰로틀 조건에서 기체 총 중량 4kg 이상을 들 수 있는 모터**를 사전 필터링한 리스트 (총 52종)입니다.  
모터명, kV, 권선 저항(Rm), 최대 전류(I_max), 질량, 무부하 전류(I0) 포함.

### ESC DB (`ESC_list/ESC_data.xlsx`)

ESC명, 정격전류, 질량 포함.

---

## 전체 코드 파이프라인

```
├ main.m                    : 설계 1안 — Endurance 중점 추진 시스템 선정
├ forward_flight_analysis.m : 설계 2안 — Maneuverability 중점 추진 시스템 선정
├ aero_analysis.m           : 공력 해석 (Disk Loading, Figure of Merit, 비추력)
├ arm_wake_analysis.m       : arm 형상·두께·배치 조건에 따른 추력 손실률 분석
├ planar_analysis.m         : 선정된 프롭·모터 기준 최적 arm 길이 설계
├ ground_effect_analysis.m  : 지면효과 분석 및 최적 랜딩기어 높이 설계
│
├ src/
│  ├ load_propList.m        : APC 프롭 직경·피치·질량·RPM 제한값 DB 구성
│  ├ load_propPerf.m        : PER2_STATIC-2.DAT에서 RPM·추력·토크·출력 추출
│  ├ load_motorList.m       : 요구 RPM·토크·전류·질량 조건 기반 모터 필터링 및 점수 정렬
│  ├ load_escList.m         : 요구 전류 조건을 만족하는 ESC 선정
│  ├ compare_hoverTime.m    : 1안 vs 2안 호버 시간 비교 출력
│  └ validate_propPerf.m    : 프롭 성능 데이터 유효성 검증
│
├ plot/
│  ├ plot_propPerf.m        : 프롭 성능 그래프 출력
│  └ plot_motorPerf.m       : 모터 성능 그래프 출력
│
└ results_arm_wake/
   └ arm_wake_analysis_result.xlsx  : arm_wake_analysis 결과 자동 저장
```

### 실행 순서

**설계 1안 (Endurance)**

```
main → aero_analysis → arm_wake_analysis → planar_analysis → ground_effect_analysis
```

**설계 2안 (Maneuverability)**

```
main → forward_flight_analysis → arm_wake_analysis → planar_analysis → ground_effect_analysis
```

`aero_analysis.m`은 설계 1안 전용입니다. 설계 2안은 전진비행 성능 기반으로 프롭을 선정하므로 호버 공력 해석 단계를 수행하지 않습니다.

`arm_wake_analysis` → `planar_analysis` → `ground_effect_analysis` 순서는 반드시 지켜야 합니다.  
각 파일은 이전 단계의 workspace 변수(`bestPracticalCandidate`, `best_arm_len_p`, `final_aum_p`)를 필수 입력으로 사용합니다.

`arm_wake_analysis.m`, `planar_analysis.m`, `ground_effect_analysis.m`은  
workspace에 `prop_ff` / `mass_Total_ff`가 존재하는지 자동으로 감지하여 설계안별 입력값을 전환합니다.

| 실행 순서 | `prop_ff` 존재 | 사용 기준 |
|-----------|:--------------:|-----------|
| `main` → downstream | 없음 | `propSpecification` · `mass_Total` (1안) |
| `main` → `forward_flight` → downstream | 있음 | `prop_ff` · `mass_Total_ff` (2안 수렴값) |

---

## 설계 1안: Endurance 추진 시스템 선정 (`main.m`)

S500 프레임을 기초 형상으로, 호버링 비추력을 기준으로 최적 prop / motor / ESC / 배터리를 선정합니다.

```
① 총 질량 추정 (mass_Total_Est)
   frame + FC/GPS/센서류 + motor_Est×N + ESC_Est×N + prop_Est×N + batt + payload
    ↓
② 후보 프롭 로드 (load_propList)
   직경·피치·질량·RPM 제한값 필터링 (AcceptedTypes, PropDiameter_Min/Max)
    ↓
③ 성능 데이터 파싱 (load_propPerf)
   PER2_STATIC-2.DAT → RPM별 추력·토크·출력 행렬
    ↓
④ 운용점 계산 (operatingPoints)
   hover / WOT / RPM limit 기준 3개 운용점 도출
    ↓
⑤ 프롭 선정 — APC 실측 파워(col2) 기반 비추력 최대
   → propSpecification (1안 선정 프롭)
    ↓
⑥ 모터 선정 (load_motorList)
   하드 탈락: disc<0 / I_req>I_max / kV_margin<0.7
   소프트 페널티: 질량 초과 / kV_margin 0.7~1.0
   → motorChosen
    ↓
⑦ ESC 선정 (load_escList) — 요구 전류 × 1.2 여유 기준
   → best_esc
    ↓
⑧ 실측 질량 확정 (mass_Total)
   frame + FC/GPS/센서류 + 선정 motor×N + 선정 ESC×N + 선정 prop×N + batt + payload
    ↓
⑨ 체공시간 계산 (Peukert 배터리 모델)
결과 출력 + 그래프 (plot_propPerf, plot_motorPerf)
```

**최종 선정 결과 (설계 1안)**

| 항목 | 결과 |
|------|------|
| 프로펠러 | APC 12x5.5MR |
| 모터 | KDE3510 475kV |
| ESC | KISS ESC 3-6S 32A |

---

## 설계 2안: Maneuverability 추진 시스템 선정 (`forward_flight_analysis.m`)

설계 1안의 호버 후보 프롭(`propList_considered`)을 그대로 계승하여 전진비행 성능을 평가합니다.  
선정된 motor/ESC/prop 무게 변화를 반영하는 **질량 수렴 루프**를 포함합니다.

적용 이론: Pollet et al. (ICAS 2021) 힘 평형 모델 + Leng et al. 비축 입사각 CT/CP 보정

```
설계 1안 호버 후보 프롭 (propList_considered)
    ↓
┌── 질량 수렴 루프 (max 10회, 허용 오차 1g) ─────────────────────────────┐
│                                                                         │
│  ① 전진비행 힘 평형 (Pollet Eq. 3, 수치 반복)                          │
│     W_N = mass_Total_ff × g 기반 기체 경사각 α 수렴 계산               │
│      ↓                                                                  │
│  ② CT / CP 전진비행 보정 (Pollet Eq. 8-9)                              │
│     Leng et al. 비축 입사각 모델 적용                                  │
│     C_{T,P}(α, J) = η_{T,P}(α) · C_{T,P}^{axial}(β, J_{axial})        │
│      ↓                                                                  │
│  ③ 기동성 지표 계산 — 10회 왕복 시간 추정                              │
│     T_surplus → a_max → t_lap (삼각형 속도 프로파일, L=2m)            │
│      ↓                                                                  │
│  ④ 최적 prop 선정 — 왕복 시간 최소화 (= 여유추력 최대화)               │
│      ↓                                                                  │
│  ⑤ 모터 선정 (load_motorList)                                           │
│     호버 + 전진비행 운용점 중 더 혹독한 조건 기준                      │
│      ↓                                                                  │
│  ⑥ ESC 선정 — 전류 여유 20%                                             │
│      ↓                                                                  │
│  ⑦ 질량 갱신                                                            │
│     mass_Total_ff = mass_Total                                          │
│                   + (motor_ff - motor_1안) × N                         │
│                   + (ESC_ff  - ESC_1안)   × N                          │
│                   + (prop_ff - prop_1안)  × N                          │
│      ↓                                                                  │
│  |Δmass| < 1g → 수렴 종료                                              │
└─────────────────────────────────────────────────────────────────────────┘
    ↓
mass_Total_ff 확정 → workspace 저장
2안 호버 시간 계산 (Peukert 배터리 모델)
설계 1안 vs 2안 비교 출력 (compare_hoverTime)
    ↓
그래프 1: 전진속도 vs 여유추력 곡선 (1안 vs 2안)
그래프 2: t_lap vs V_max 산점도 (후보 프롭 기동성 비교)
그래프 3: T_surplus vs t_lap 산점도 (여유추력 vs 왕복시간 상관관계)
```

**기동성 지표 수식**

$$T_{total} = \sqrt{W^2 + D_f^2}, \quad D_f = \frac{1}{2}C_d \rho V^2 S_{ref}$$

$$a_{max} = \frac{\sqrt{T_{max}^2 - W^2}}{m}, \quad t_{lap} = 20 \times 2\sqrt{\frac{L}{a_{max}}} \quad (L=2\,\text{m},\ \text{왕복 10회})$$

**최종 선정 결과 (설계 2안)**

| 항목 | 결과 |
|------|------|
| 프로펠러 | APC 11x7E-3 |
| 모터 | MN3110 700kV |
| ESC | KISS ESC 3-6S 32A |
| Hover Time | 18 min |
| t_lap (10회) | 5.3s |

1안 대비 호버 시간 3분 감소, 왕복시간 30% 감소, 최대속도 53% 향상

---

## 공력 해석 (`aero_analysis.m`)

항상 **1안(호버 최적 프롭)** 기준으로, 후보 프롭 전체의 공력 성능을 비교·시각화합니다.

```
후보 프롭 전체 (propList_considered)
    ↓
① Disk Loading 계산
   DL = T / A  [g/cm²]  (A = π·(D/2)²)
    ↓
② Figure of Merit 계산
   FM = (T^1.5 / sqrt(2ρA)) / P_mech
    ↓
③ 비추력 vs RPM 곡선
   선정 프롭 강조 + 후보 전체 비교
    ↓
그래프 1: DL vs FM 산점도 + 비추력 등고선
그래프 2: 호버링 비추력 막대그래프 (통과 프롭 내림차순)
```

---

## Arm 형상에 따른 추력 손실 분석 (`arm_wake_analysis.m`)

선정된 프롭 기준으로 암 형상·두께·배치별 후류 공력 손실을 계산합니다.  
허용 손실률(1%) 이내 **최소 두께** 후보(`bestPracticalCandidate`)를 도출하며, 이 값은 이후 `planar_analysis.m`의 필수 입력으로 사용됩니다.  
결과는 `results_arm_wake/arm_wake_analysis_result.xlsx`에 자동 저장됩니다.

```
선정 프롭 (1안: propSpecification / 2안: prop_ff 자동 전환)
    ↓
① 후류 속도 계산 (Actuator disk theory)
   v_i    = sqrt(T / (2·ρ·A))
   v_wake = k · v_i  (k = 1 + min(z/R, 1): 암 수직 위치에 따른 계수)
    ↓
② 암 형상별 항력 및 추력 손실률 계산
   D_arm = 0.5·ρ·v_wake²·Cd·A_proj
   형상: circular(Cd=1.10) / square(1.80) / elliptic(0.50) / streamlined(0.15)
   두께 후보: 11~20 mm (1mm 단위)
   y/R 오프셋: 0.00~0.20 (0.05 단위)
   z/R 수직 위치: 0.272~1.00 (0.05 단위)
    ↓
③ 허용 손실률(1%) 이내 최소 두께 후보 도출
   선정 우선순위: 허용 이내 > arm 두께 최소 > 추력 손실 최소 > 추가출력 최소
   → bestPracticalCandidate (암 형상·두께·위치·손실률 포함)
    ↓
④ 와류 방출 주파수(Strouhal) vs 로터/BPF 공명 검토
   f_shed = St · v_wake / d  (St=0.20, 3블레이드 기준)
    ↓
그래프: 형상별 두께에 따른 최소 추력 손실률
결과 저장: results_arm_wake/arm_wake_analysis_result.xlsx
           (All Results / Best by Shape / Max Thickness / Min Offset / Z Effect)
```

**설계 파라미터**

| 파라미터 | 값 | 설명 |
|---|---|---|
| `targetLossPct` | 1.0 % | 허용 추력 손실 상한 |
| `armThickness_mm_list` | 11 ~ 20 mm | arm 두께 후보 범위 |
| `yOverR_list` | 0.00 ~ 0.20 | 로터 중심 대비 횡방향 오프셋 |
| `zOverR_list` | 0.272 ~ 1.00 | 로터 디스크 대비 수직 위치 |
| `wakeCrossMode` | `"radius"` | 후류 통과 길이 산출 기준 |

**선정 결과 (설계 2안 기준 예시)**

| 형상 | 최소 허용 두께 [mm] | 손실률 [%] |
|------|:-----------:|:----------:|
| Streamlined | 11 | ≤ 1.00 |

---

## 최적 Arm 길이 설계 (`planar_analysis.m`)

`arm_wake_analysis.m`의 `bestPracticalCandidate`를 입력으로 받아,  
**선정된 프롭과 모터에 최적화된 암 길이를 설계**합니다.  
초기 기준인 S500 프레임(암 길이 250mm)에서 출발하여, 렘니스케이트(역물방울) 단면 기반 암 질량과 추력 여유를 함께 고려한 최적 spacing을 도출합니다.

```
bestPracticalCandidate (arm_wake_analysis 출력 — ArmThickness_mm: 암 두께 w [mm])
선정 프롭 반경 (1안: propSpecification{2}/2 / 2안: prop_ff{3}/2 자동 전환)
기체 총 질량 (1안: mass_Total / 2안: mass_Total_ff 자동 전환)
    ↓
[1단계] 렘니스케이트(역물방울) 단면 치수 계산
   US11305881 특허 단면: (y²+x²)² = 2a²(y²-x²) 상단 루프
   ArmThickness_mm = w (단면 최대 폭) → a_outer = w → h = w·√2
   A_outer = a_outer², A_shell = a_outer² − a_inner²
   단위 길이당 무게 = A_shell × ρ_carbon  (ρ_carbon = 1.80×10⁻³ g/mm³, tw = 1mm)
    ↓
[2단계] 암 길이 계산 (L/R = 3.0 기준)
   정의: L/R = L / R  (L: arm 1개 길이, R: 프롭 반경)
   정사각형 X자 배치: 인접 로터 간 거리 = L·√2
   → L = spacing × R_mm / √2 = spacing × d_prop[inch] × 25.4 / √2
    ↓
[3단계] Planar — L/R spacing별 추력 여유 비교
   대상 spacing: 2.2 / 2.6 / 3.0 / 3.2 / 3.6 / 4.0
   RPM 구간별 추력 증가율 적용 (논문 Figure 4 기반, 8구간 평균)
   렘니스케이트 단면 기반 암 추가 질량 반영
   추력 손실 계수(bestPracticalCandidate.ThrustLoss_pct) 적용
   → 추력 여유 기준 최적 spacing 선정 (L/R = 3.0 고정)
    ↓
[4단계] 최종 암 길이·질량 확정
   best_arm_len_p [mm] → ground_effect_analysis로 export
   final_aum_p    [g]  → ground_effect_analysis로 export
그래프 출력 (spacing별 발생 vs 필요 추력 막대그래프)
```

**암 길이 정의 명확화**

| 변수 | 정의 | 비고 |
|------|------|------|
| `L` (= `arm_len_p`) | arm 1개 길이, 중심→프롭 [mm] | 슬라이드 "전체 arm 길이"와 동일 |
| 인접 로터 간 거리 | L × √2 | 정사각형 X자 배치 기준 |
| 대각 로터 간 거리 | L × 2 | ground_effect `b_m` 기준 |

**최종 결과 요약**

| 설계 구분 | 설계 1안 | 설계 2안 |
|-----------|:--------:|:--------:|
| 최적 L/R | 3.0 | 3.0 |
| 암 길이 (중심→프롭) [mm] | 323 | 296 |
| 암 질량 증가분 [g] | +11.1 | +7.0 |
| 최종 기체 질량 [g] | 2140.1 | 2052.0 |

---

## 지면효과 분석 (`ground_effect_analysis.m`)

멀티로터 전용 지면효과 모델(로터 간 간섭 + fountain effect)을 적용하여 최적 랜딩기어 높이를 산출합니다.  
`planar_analysis.m` 실행 후 `best_arm_len_p`와 `final_aum_p`가 workspace에 존재하는 것을 필수 조건으로 확인하며, 이 값들을 기반으로 정확한 지면효과 해석을 수행합니다.

```
best_arm_len_p (arm 1개 길이 [mm]) + final_aum_p (암 포함 최종 AUM [g])
선정 프롭 (1안: propSpecification / 2안: prop_ff 자동 전환)
    ↓
[기하 계산 — planar_analysis 연동]
   ArmLength_m = best_arm_len_p / 1000
   d_m (인접 로터 간 거리) = √2 × ArmLength_m
   b_m (대각 로터 간 거리) = 2  × ArmLength_m
   Kb (fountain effect 계수) = 0.5
    ↓
① 유도속도 계산 (final_aum_p 기준)
   T_total = final_aum_p × g,  T_per_rotor = T_total / RotorNo
   v_i = sqrt(T_per_rotor / (2·ρ·A))
    ↓
② 멀티로터 지면효과 모델 (computeQuadGroundEffect)
   단일 로터 Cheeseman-Bennett
   + 인접/대각 로터 간섭 + fountain effect (Kb=0.5)
   탐색 범위: z = 10 ~ 25 cm (로터 디스크~지면)
    ↓
③ 랜딩기어 높이 설계 기준 도출
   Gradient stop: |dfGE/dz| ≤ 0.9 [1/m] 를 처음 만족하는 높이
   Upper-limit crossing: fGE ≤ 1.10 을 처음 만족하는 높이
   두 후보 중 낮은(작은) 값을 최종 랜딩기어 높이로 선정
    ↓
그래프 1: fGE vs 로터 높이 (hover, 최종 선정 높이 표시)
그래프 2: fGE vs 전진속도 (전진비행 최적 프롭 분석 시만 출력, z=2.0m 기준)
```

**지면효과 모델 수식**

$$f_{GE} = \frac{1}{1 - \chi \cdot \Sigma_{GE}}, \quad \chi = \frac{1}{1 + (V/v_i)^2}$$

$$\Sigma_{GE} = \left(\frac{R}{4z}\right)^2 + \frac{R^2 z}{(d^2+4z^2)^{3/2}} + \frac{R^2 z/2}{(2d^2+4z^2)^{3/2}} + \frac{2K_b R^2 z}{(b^2+4z^2)^{3/2}}$$

**설계 파라미터**

| 파라미터 | 값 | 설명 |
|---|---|---|
| `fGE_upper_limit` | 1.10 | 허용 지면효과 상한 |
| `fGE_stable_limit` | 1.05 | 안정 호버 기준 |
| `gradient_stop_threshold` | 0.9 [1/m] | fGE 민감도 허용 상한 |
| `z_min_m` / `z_max_m` | 0.10 / 0.25 m | 랜딩기어 높이 탐색 범위 |
| `Kb` | 0.5 | Fountain effect 계수 |

**최종 선정 결과**

| 항목 | 1안 결과 | 2안 결과 |
|------|:--------:|:--------:|
| 프로펠러 | APC 12x5.5MR | APC 11x7E-3 |
| 선정 기준 | Upper-Limit Crossing | Upper-Limit Crossing |
| 랜딩기어 높이 | — | — |

---

## 주요 파라미터 설정 (`main.m`)

```matlab
RotorNo           = 4;           % 로터 수
OptimisationGoal  = 'hover';     % 'hover' | 'max' | 'utilisation'
ThrustWeightRatio = 2;           % 추력-중량비 (2=최소, 3=수송, 5+=곡예)
PropDiameter_Min  = 0;           % 프롭 직경 하한 [inch]
PropDiameter_Max  = 14;          % 프롭 직경 상한 [inch]
SafetyFactor      = 1.1;         % 안전 계수
AcceptedTypes     = {'MR' 'E-3' 'E-4'};  % 허용 프롭 시리즈

BattCellNo        = 4;           % 배터리 셀 수 (S)
BattCellVoltage   = 3.7;         % 셀 공칭 전압 [V]
BattCapacity      = 5000;        % 배터리 용량 [mAh]
BattPeukertConstant = 1.3;       % Peukert 상수 (LiPo 기준)

%% 질량 파라미터 [g]
mass_Frame            = 454;     % S500 프레임 + 랜딩기어 질량 (실측값)
mass_FC               = 34.6;    % 비행 컨트롤러
mass_FC_GPS           = 7;       % GPS 모듈
mass_FC_CurrentSensor = 15;      % 전류 센서
mass_Receiver         = 2;       % 수신기
mass_optical          = 1.5;     % 광류 센서
mass_Other_Est        = 20;     % 케이블, 스트랩 등
mass_Payload          = 500;     % 탑재물
mass_Battery          = 500;     % 4S 5000mAh (학과 지급)
```

---

## 주요 Workspace 변수

### main.m 출력 (설계 1안)

| 변수 | 설명 |
|------|------|
| `propList_considered` | 호버 후보 프롭 전체 (`consideredNo × 6` cell) |
| `propSpecification` | 1안 선정 프롭 사양 `{name, diam_in, pitch_in}` |
| `operatingPoints` | 후보 프롭별 운용점 (hover/WOT/RPM limit) |
| `temp_propChosen_pos` | 1안 선정 프롭 인덱스 (arm_wake_analysis 참조) |
| `motorChosen` | 1안 선정 모터 (`1×12` cell) |
| `best_esc` | 1안 선정 ESC |
| `mass_Total` | 1안 실측 기체 질량 [g] |
| `ThrustWeightRatio` | 추력-중량비 (planar_analysis 참조) |
| `time_hover` | 1안 호버 비행시간 벡터 [h] |

### forward_flight_analysis.m 출력 (설계 2안)

| 변수 | 설명 |
|------|------|
| `prop_ff` | 2안 선정 프롭 (`1×6` cell) |
| `idx_ff_best` | 2안 프롭 인덱스 (arm_wake_analysis 참조) |
| `V_max_best` | 최대 전진속도 [m/s] |
| `motorChosen_ff` | 2안 선정 모터 (`1×12` cell) |
| `best_esc_ff` | 2안 선정 ESC |
| `mass_Total_ff` | 2안 수렴 후 최종 기체 질량 [g] ← downstream 전체 참조 |

### arm_wake_analysis.m 출력

| 변수 | 설명 |
|------|------|
| `bestPracticalCandidate` | 허용 손실률(1%) 이내 최소 두께 후보 (형상·두께·위치·손실률 포함) |
| `resultTable` | 전체 형상·두께·위치 조합 계산 결과 테이블 |

### planar_analysis.m 출력

| 변수 | 설명 |
|------|------|
| `best_arm_len_p` | 선정된 arm 1개 길이 (중심→프롭) [mm] ← ground_effect_analysis 필수 입력 |
| `final_aum_p` | 암 무게 포함 최종 AUM [g] ← ground_effect_analysis 필수 입력 |

---

## 적용 프레임 (S500)

초기 해석은 S500 프레임 기준 파라미터에서 출발합니다.  
암 길이는 `planar_analysis.m`에서 선정된 프롭과 모터에 맞춰 최적값으로 재설계되며,  
`ground_effect_analysis.m`은 해당 설계 결과를 자동으로 반영하여 지면효과 해석을 수행합니다.

| 항목 | 값 |
|------|----|
| 휠베이스 | 500 mm |
| 프레임 질량 | 454 g (프레임 + 랜딩기어 실측값) |
| 초기 암 길이 기준 | 250 mm (S500 기본값) |
| 상면 투영면적 `Stop` | 0.059 m² |
| 정면 투영면적 `Sfront` | 0.021 m² |
| 항력계수 `Cd_frame` | 1.0 (일반 쿼드콥터 오픈 프레임 기준) |
| 허용 프롭 시리즈 | MR / E-3 / E-4, 최대 14 inch |
| 탑재 배터리 | 4S 5000mAh (500 g, 학과 지급) |
| 암 재질 | 렘니스케이트(역물방울) 단면 카본 파이프 (ρ = 1.80×10⁻³ g/mm³, tw = 1mm) |

---

## 참고문헌

- Biczyski et al. (2020). *Multirotor Sizing Methodology with Flight Time Estimation.* Journal of Advanced Transportation.
- Pollet et al. (2021). *Design Optimization of Multirotor Drones in Forward Flight.* ICAS 2021.
- Leng et al. (2019). *An Analytical Model for Propeller Aerodynamic Efforts at High Incidence.*
- Lei et al. (2020). *Aerodynamic Interference Effect on Multi-Rotor UAV.* (L/R spacing 기준 참조)
- Peng, U.S. Patent US11305881B2 (2022). *Lemniscate (Inverted Teardrop) Cross-Section Arm Design.*

---

## 요구 환경

- MATLAB R2017b 이상
- 추가 툴박스 불필요
