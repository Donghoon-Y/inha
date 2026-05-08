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
│  └ load_escList.m         : 요구 전류 조건을 만족하는 ESC 선정
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
각 파일은 이전 단계의 workspace 변수(`bestPracticalCandidate`, `best_arm_len_p`)를 필수 입력으로 사용합니다.

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
결과 출력 + 그래프
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
설계 1안 vs 2안 비교 출력
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
허용 손실률(3%) 이내 최적 암 후보(`bestPracticalCandidate`)를 도출하며, 이 값은 이후 `planar_analysis.m`의 필수 입력으로 사용됩니다.

```
선정 프롭 (1안: propSpecification / 2안: prop_ff 자동 전환)
    ↓
① 후류 속도 계산 (Actuator disk theory)
   v_i    = sqrt(T / (2·ρ·A))
   v_wake = k · v_i  (k: 암 위치에 따른 계수)
    ↓
② 암 형상별 항력 및 추력 손실률 계산
   D_arm = 0.5·ρ·v_wake²·Cd·A_proj
   형상: circular(Cd=1.10) / square(1.80) / elliptic(0.50) / streamlined(0.15)
    ↓
③ 허용 손실률(3%) 이내 최대 두께 도출
   → bestPracticalCandidate (암 형상·두께·손실률 포함)
    ↓
그래프 출력 (손실률 vs 암 두께/형상)
```

**결과 요약 (설계 2안 기준)**

| 형상 | 최대 두께 [mm] | 후류 속도 [m/s] | 항력 [N] | 손실률 [%] |
|------|:--------------:|:---------------:|:--------:|:----------:|
| Circular | 30 | 27.94 | 1.638 | 2.98 |
| Square | 18 | 27.94 | 1.608 | 2.93 |
| Elliptic | 40 | 27.94 | 0.990 | 1.81 |
| Streamlined | 40 | 27.94 | 0.296 | 0.54 |

---

## 최적 Arm 길이 설계 (`planar_analysis.m`)

`arm_wake_analysis.m`의 `bestPracticalCandidate`를 입력으로 받아,  
**선정된 프롭과 모터에 최적화된 암 길이를 설계**합니다.  
초기 기준인 S500 프레임(암 길이 250mm)에서 출발하여, 렘니스케이트(역물방울) 단면 기반 암 질량과 추력 여유를 함께 고려한 최적 spacing을 도출합니다.

```
bestPracticalCandidate (arm_wake_analysis 출력 — 암 단면 치수)
선정 프롭 반경 (1안: propSpecification / 2안: prop_ff 자동 전환)
기체 총 질량 (1안: mass_Total / 2안: mass_Total_ff 자동 전환)
    ↓
렘니스케이트(역물방울) 단면 치수 계산
   US11305881 특허 단면: (y²+x²)² = 2a²(y²-x²) 상단 루프
   a_outer = h/√2,  단면적 A = a²,  단위 길이당 무게 계산
    ↓
L/R spacing별 암 길이 산출
   S500 기준 암 길이(250mm)에서 출발
   arm_length = spacing × d_prop × 25.4 / √2
    ↓
Planar — L/R spacing별 추력 여유 비교
   RPM 구간별 추력 증가율 적용
   렘니스케이트 단면 기반 암 추가 질량 반영
   → 추력 여유 기준 최적 spacing 선정 (L/R = 3.0)
    ↓
최적 암 길이 및 질량 확정 → best_arm_len_p workspace 저장
그래프 출력
```

**최종 결과 요약**

| 설계 구분 | 설계 1안 | 설계 2안 |
|-----------|:--------:|:--------:|
| 최적 L/R | 3.0 | 3.0 |
| 최적 암 길이 [mm] | 323 | 296 |
| 암 질량 증가분 [g] | +35.4 | +22.2 |
| 최종 기체 질량 [g] | 1906.4 | 1893.2 |

---

## 지면효과 분석 (`ground_effect_analysis.m`)

멀티로터 전용 지면효과 모델(로터 간 간섭 + fountain effect)을 적용하여 최적 랜딩기어 높이를 산출합니다.  
`planar_analysis.m` 실행 후 `best_arm_len_p`가 workspace에 존재하면, S500 기본값(0.25m) 대신 **설계된 암 길이 기반 기하값**을 사용하여 보다 정확한 지면효과 해석을 수행합니다.

```
선정 프롭 + 설계된 암 길이 (planar_analysis 결과 자동 반영)
    ↓
① 유도속도 계산
   v_i = sqrt(T / (2·ρ·A))
    ↓
② 멀티로터 지면효과 모델 (computeQuadGroundEffect)
   단일 로터 Cheeseman-Bennett
   + 인접/대각 로터 간섭 + fountain effect
    ↓
③ 고도별 추력 증가율 곡선
    ↓
④ 랜딩기어 높이 설계 기준 도출
   fGE 기울기 기준(gradient_stop)으로 실용 상한 고도 결정
    ↓
그래프 출력
```

**최종 선정 결과**

| 항목 | 1안 결과 | 2안 결과 |
|------|:--------:|:--------:|
| 프로펠러 | APC 12x5.5MR | APC 11x7E-3 |
| Gradient Stop | 1.107 | 1.103 |
| 판정 기준 | Upper-Limit Crossing | Upper-Limit Crossing |

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
BattCapacity      = 5200;        % 배터리 용량 [mAh]
BattPeukertConstant = 1.3;       % Peukert 상수 (LiPo 기준)

%% 질량 파라미터 [g]
mass_Frame            = 222.5;   % S500 프레임 질량 (실측값)
mass_FC               = 34.6;    % 비행 컨트롤러
mass_FC_GPS           = 7;       % GPS 모듈
mass_FC_CurrentSensor = 15;      % 전류 센서
mass_Receiver         = 2;       % 수신기
mass_optical          = 1.5;     % 광류 센서
mass_Payload          = 500;     % 탑재물
mass_Battery          = 473;     % Lumenier 5200mAh 4S 35C
```

---

## 주요 Workspace 변수

### main.m 출력 (설계 1안)

| 변수 | 설명 |
|------|------|
| `propList_considered` | 호버 후보 프롭 전체 (`consideredNo × 6` cell) |
| `propSpecification` | 1안 선정 프롭 사양 `{name, diam_in, pitch_in}` |
| `motorChosen` | 1안 선정 모터 (`1×12` cell) |
| `best_esc` | 1안 선정 ESC |
| `mass_Total` | 1안 실측 기체 질량 [g] |
| `time_hover` | 1안 호버 비행시간 벡터 [h] |

### forward_flight_analysis.m 출력 (설계 2안)

| 변수 | 설명 |
|------|------|
| `prop_ff` | 2안 선정 프롭 (`1×6` cell) |
| `V_max_best` | 최대 전진속도 [m/s] |
| `motorChosen_ff` | 2안 선정 모터 (`1×12` cell) |
| `best_esc_ff` | 2안 선정 ESC |
| `mass_Total_ff` | 2안 수렴 후 최종 기체 질량 [g] ← downstream 전체 참조 |

### arm_wake_analysis.m 출력

| 변수 | 설명 |
|------|------|
| `bestPracticalCandidate` | 허용 손실률(3%) 이내 최적 암 후보 (형상·두께·손실률 포함) |

### planar_analysis.m 출력

| 변수 | 설명 |
|------|------|
| `best_arm_len_p` | Planar 기준 최적 암 길이 [mm] ← ground_effect_analysis 자동 참조 |

---

## 적용 프레임 (S500)

초기 해석은 S500 프레임 기준 파라미터에서 출발합니다.  
암 길이는 `planar_analysis.m`에서 선정된 프롭과 모터에 맞춰 최적값으로 재설계되며,  
`ground_effect_analysis.m`은 해당 설계 결과를 자동으로 반영하여 지면효과 해석을 수행합니다.

| 항목 | 값 |
|------|----|
| 휠베이스 | 500 mm |
| 프레임 질량 | 222.5 g (실측값) |
| 초기 암 길이 기준 | 250 mm (S500 기본값) |
| 상면 투영면적 `Stop` | 0.059 m² |
| 정면 투영면적 `Sfront` | 0.021 m² |
| 항력계수 `Cd_frame` | 1.0 (일반 쿼드콥터 오픈 프레임 기준) |
| 허용 프롭 시리즈 | MR / E-3 / E-4, 최대 14 inch |
| 탑재 배터리 | Lumenier 5200mAh 4S 35C (473 g) |
| 암 재질 (기준) | 렘니스케이트(역물방울) 단면 카본 파이프 |

---

## 참고문헌

- Biczyski et al. (2020). *Multirotor Sizing Methodology with Flight Time Estimation.* Journal of Advanced Transportation.
- Pollet et al. (2021). *Design Optimization of Multirotor Drones in Forward Flight.* ICAS 2021.
- Leng et al. (2019). *An Analytical Model for Propeller Aerodynamic Efforts at High Incidence.*
- US11305881 특허. *Lemniscate (Inverted Teardrop) Cross-Section Arm Design.*

---

## 요구 환경

- MATLAB R2017b 이상
- 추가 툴박스 불필요
