# Multirotor Sizing Methodology

멀티로터 드론의 **프로펠러 · 모터 · ESC · 배터리**를 자동으로 선정하고,  
체공시간 · 전진비행 성능 · 암 길이 · 지면효과 · 암 후류 간섭까지 통합 분석하는 MATLAB 설계 도구입니다.

Biczyski et al. (2020) *"Multirotor Sizing Methodology with Flight Time Estimation"* 의 방법론을 기반으로 하며,  
전진비행 해석(Pollet et al., ICAS 2021), 지면효과, 암 형상 공력 해석 모듈이 추가되었습니다.

---

## 설계 방향

두 가지 설계안을 병렬로 도출합니다.

| 구분 | 목표 | 최적화 기준 |
|------|------|-------------|
| **설계 1안 (Endurance)** | 체공시간 최대화 | 호버링 비추력 (g/W) 최대 |
| **설계 2안 (Maneuverability)** | 왕복비행 시간 최소화 | 여유추력(T_surplus) 최대 → 가속도 최대 |

> **두 안 모두** 동일한 후보 프롭 풀(`propList_considered`)에서 출발합니다.  
> 1안은 호버 효율, 2안은 전진비행 여유추력을 기준으로 각자 최적 prop / motor / ESC를 독립 선정합니다.

---

## 전체 실행 흐름

### 설계 1안 (Endurance — 체공시간 최적화)

```
main.m
  │  호버 최적 prop / motor / ESC 선정
  │  mass_Total (실측 부품 기반 기체 질량) 확정
  │
  ├─► aero_analysis.m
  │     Disk Loading · Figure of Merit · 비추력 곡선
  │     (1안 프롭 기준, 후보 전체 비교 포함)
  │
  ├─► ground_effect_analysis.m
  │     멀티로터 지면효과 모델 (로터 간 간섭 + fountain effect)
  │     랜딩기어 높이 설계 기준 도출
  │
  ├─► arm_wake_analysis.m
  │     암 형상/두께별 후류 공력 손실 계산
  │     허용 손실률 이내 최대 암 두께 도출
  │
  └─► planar_analysis.m
        L/R spacing별 발생 추력 vs 필요 추력 비교
        암 길이 및 암 무게 증분 반영
        → 암 길이·형상 설계 기준 도출
```

### 설계 2안 (Maneuverability — 기동성 최적화)

```
main.m
  │  (1안과 동일 실행, 호버 후보 프롭 풀 생성)
  │
  └─► forward_flight_analysis.m
        ┌─ 질량 수렴 루프 (최대 10회, 허용 오차 1g) ─────────────────┐
        │  W_N = mass_Total_ff 기반 전진비행 힘 평형                  │
        │  CT/CP 비축 입사각 보정 (Leng et al.)                       │
        │  왕복 시간 기준 최적 prop 선정                              │
        │  2안 motor / ESC 선정                                       │
        │  mass_Total_ff = mass_Total                                  │
        │                  + Δmotor × RotorNo                         │
        │                  + ΔESC   × RotorNo                         │
        │                  + Δprop  × RotorNo   → 재수렴              │
        └──────────────────────────────────────────────────────────────┘
        mass_Total_ff 확정 → workspace 저장
              │
              ├─► ground_effect_analysis.m
              │     (prop_ff · mass_Total_ff 기준으로 자동 전환)
              │
              ├─► arm_wake_analysis.m
              │     (prop_ff · motorChosen_ff 기준으로 자동 전환)
              │
              └─► planar_analysis.m
                    (prop_ff · mass_Total_ff 기준으로 자동 전환)
                    L/R spacing별 암 길이 · 추력 여유 분석
```

> **질량 수렴 루프**: 2안에서 선정된 motor/ESC/prop의 무게가 1안과 다를 경우  
> `mass_Total_ff`를 반복 갱신하여 전진비행 힘 평형과 부품 선정을 수렴시킵니다.  
> 수렴 후 `mass_Total_ff`가 workspace에 저장되고, 이후 모든 downstream 파일이 이 값을 참조합니다.

---

## downstream 자동 분기 동작

`ground_effect_analysis.m`, `arm_wake_analysis.m`, `planar_analysis.m`은  
workspace에 `prop_ff` / `mass_Total_ff`가 존재하는지 자동으로 감지하여  
설계안별 입력값을 전환합니다.

| 실행 순서 | `prop_ff` | `mass_Total_ff` | 사용 기준 |
|-----------|:---------:|:---------------:|-----------|
| `main` → downstream | 없음 | 없음 | `propSpecification` · `mass_Total` (1안) |
| `main` → `forward_flight` → downstream | 있음 | 있음 | `prop_ff` · `mass_Total_ff` (2안 수렴값) |
| `forward_flight` 실패 → downstream | 있음 | 없음 | `mass_Total` 폴백 + 경고 출력 |

---

## 파일 구조

```
.
├── main.m                        # 설계 1안: 호버 최적 prop / motor / ESC / 배터리
├── forward_flight_analysis.m     # 설계 2안: 전진비행 기동성 최적화 + 질량 수렴 루프
├── aero_analysis.m               # 공력 해석 — Disk Loading, Figure of Merit (1안 기준)
├── ground_effect_analysis.m      # 지면효과 해석 — 랜딩기어 높이 설계
├── arm_wake_analysis.m           # 암 후류 공력 손실 해석 — 암 두께 설계
├── planar_analysis.m             # 암 길이 해석 — L/R spacing · 발생 추력 · 필요 추력
│
├── src/
│   ├── load_propList.m           # APC 프롭 목록 로딩 (WEBLIST.xlsx + PROP-DATA-FILE)
│   ├── load_propPerf.m           # APC 정적 성능 데이터 파싱 (PER2_STATIC-2.DAT)
│   ├── load_motorList.m          # 모터 필터링 · 하드/소프트 조건 적용 · 점수 정렬
│   ├── load_escList.m            # ESC 목록 로딩 (ESC_data.xlsx)
│   ├── compare_hoverTime.m       # 설계안 간 체공시간 비교
│   └── validate_propPerf.m       # 프롭 성능 데이터 유효성 검증
│
├── plot/
│   ├── plot_propPerf.m           # 프롭 성능 곡선 + 배터리 시뮬레이션 그래프
│   ├── plot_motorPerf.m          # 모터 성능 그래프
│   └── addaxis*.m                # 다중 Y축 플롯 유틸리티 (외부 라이브러리)
│
├── APC_Prop/
│   ├── PER2_STATIC-2.DAT         # APC 전 기종 정적 성능 통합 데이터 (~300종)
│   ├── WEBLIST.xlsx              # APC 프롭 전체 목록
│   ├── PROP-DATA-FILE_202602.xlsx # APC 치수/무게 데이터 (2026년 2월 기준)
│   ├── PROP-DATA-EXPERIMENTAL.xlsx # 실험 측정 데이터
│   └── PERFILES2/                # 기종별 개별 동적 성능 .dat 파일 (~300개)
│
├── UIUC_Prop/                    # UIUC 프롭 데이터베이스 (예비 / 비교용)
│   ├── propdata_uiuc_filtered.xlsx
│   ├── static2_uiuc_filtered.xlsx
│   └── weblist_uiuc_filtered.xlsx
│
├── motor_list/
│   └── Motor_list.xlsx           # 모터 DB (이름, kV, Rm, I_max, 질량, I0)
│
└── ESC_list/
    └── ESC_data.xlsx             # ESC DB (이름, 정격전류, 질량)
```

---

## 주요 파라미터 설정 (`main.m`)

```matlab
RotorNo           = 4;           % 로터 수
OptimisationGoal  = 'hover';     % 'hover' | 'max' | 'utilisation'
ThrustWeightRatio = 2;           % 추력-중량비 (2=최소, 3=수송, 5+=곡예)
PropDiameter_Min  = 9;           % 프롭 직경 하한 [inch]
PropDiameter_Max  = 12;          % 프롭 직경 상한 [inch]
SafetyFactor      = 1.1;         % 안전 계수
AcceptedTypes     = {'MR' 'E' 'E-3'};  % 허용 프롭 시리즈

BattCellNo        = 4;           % 배터리 셀 수 (S)
BattCellVoltage   = 3.7;         % 셀 공칭 전압 [V]
BattCapacity      = 5000;        % 배터리 용량 [mAh]

mass_Frame        = 405;         % 프레임 질량 [g]  (S500 기준)
mass_Motor_Est    = 93;          % 모터 질량 추정값 [g]  (부품 선정 초기값)
mass_Payload      = 500;         % 페이로드 [g]
```

> `mass_Motor_Est`는 모터 필터링의 초기 질량 상한으로만 사용됩니다.  
> 실제 선정 후 `mass_Total`(1안) / `mass_Total_ff`(2안 수렴값)로 갱신됩니다.

---

## 설계 1안 흐름 (`main.m`)

```
사용자 파라미터 입력
    ↓
① 프롭 후보 필터링
   직경 범위 + 허용 타입 조건 → propList_considered
    ↓
② APC 정적 성능 데이터 로딩
   PER2_STATIC-2.DAT + PERFILES2/ → RPM vs Thrust / Power / Torque / Ct / Cp
    ↓
③ 운용점 계산 (보간)
   호버 운용점 / WOT 운용점 / RPM 한계 운용점 → operatingPoints
    ↓
④ 최적 프롭 선정
   비추력(g/W) 최대화 → propSpecification, temp_propChosen_pos
    ↓
⑤ 모터 선정 (load_motorList)
   전압·토크·전류 조건 + 호버 소비전력 최소화 → motorChosen
    ↓
⑥ 배터리 시뮬레이션 (Peukert 방정식)
   호버 / WOT 비행시간 추정 → time_hover, time_max
    ↓
⑦ ESC 선정 (전류 여유 20%)
   → best_esc
    ↓
⑧ 실측 질량 확정
   mass_Total = frame + motor×N + ESC×N + prop×N + batt + payload + misc
    ↓
결과 출력 + 그래프
```

---

## 설계 2안 흐름 (`forward_flight_analysis.m`)

설계 1안의 호버 후보 프롭(`propList_considered`)을 그대로 계승하여 전진비행 성능을 평가합니다.

```
설계 1안 호버 후보 프롭 (propList_considered)
    ↓
┌── 질량 수렴 루프 (max 10회, 허용 오차 1g) ──────────────────────────┐
│                                                                      │
│  W_N = mass_Total_ff × g  (매 iter 갱신)                            │
│      ↓                                                               │
│  ① 전진비행 힘 평형 (Pollet Eq. 3, 수치 반복)                       │
│     기체 경사각 α 수렴 계산                                          │
│      ↓                                                               │
│  ② CT / CP 전진비행 보정 (Pollet Eq. 8-9)                           │
│     Leng et al. 비축 입사각 모델 적용                               │
│      ↓                                                               │
│  ③ 기동성 점수 계산 (왕복 시간 추정)                                │
│     T_surplus → a_max → t_lap (삼각형 속도 프로파일)               │
│      ↓                                                               │
│  ④ 최적 prop 선정                                                    │
│     10회 왕복 시간 최소화 (= 여유추력 최대화)                       │
│      ↓                                                               │
│  ⑤ 모터 선정 (load_motorList)                                        │
│     호버 + 전진비행 운용점 중 더 혹독한 조건 기준                   │
│     전진비행 소비전력 최소화                                         │
│      ↓                                                               │
│  ⑥ ESC 선정 (전류 여유 20%)                                          │
│      ↓                                                               │
│  ⑦ 질량 갱신                                                         │
│     mass_Total_ff = mass_Total                                       │
│                   + (motor_ff - motor_1안) × N                      │
│                   + (ESC_ff  - ESC_1안)   × N                       │
│                   + (prop_ff - prop_1안)  × N                       │
│      ↓                                                               │
│  |Δmass| < 1g → 수렴 종료                                           │
└──────────────────────────────────────────────────────────────────────┘
    ↓
mass_Total_ff 확정 → workspace 저장
설계 1안 vs 2안 비교 출력 + 여유추력 곡선 그래프
```

### 왕복 시간 추정 수식

$$T_{surplus} = N_{rotors} \cdot T_{max} - W$$

$$a_{max} = \frac{\sqrt{T_{total}^2 - W^2}}{m}$$

$$t_{lap} = 20 \times 2\sqrt{\frac{L}{a_{max}}} \quad (L = 2\,\text{m},\ 10\text{회 왕복})$$

> 항력 및 제어 지연을 무시한 이상적 조건의 하한 추정값입니다. 프롭 간 상대적 성능 비교에 활용합니다.

---

## 공력 해석 (`aero_analysis.m`)

항상 **1안(호버 최적 프롭)** 기준으로 수행합니다.

```
후보 프롭 전체 (propList_considered)
    ↓
① Disk Loading 계산
   DL = T / A  [Pa]  (A = π·(D/2)²)
    ↓
② Figure of Merit 계산
   FM = (T^1.5 / sqrt(2ρA)) / P_mech
    ↓
③ 비추력 vs RPM 곡선
   선정 프롭 강조 + 후보 전체 비교
    ↓
그래프 출력 (DL·FM·비추력 산점도 + 성능 곡선)
```

---

## 지면효과 해석 (`ground_effect_analysis.m`)

```
선정 프롭 (1안: propSpecification / 2안: prop_ff 자동 전환)
    ↓
① 유도속도 계산
   v_i = sqrt(T / (2·ρ·A))
    ↓
② 멀티로터 지면효과 모델 (computeQuadGroundEffect 로컬 함수)
   단일 로터 Cheeseman-Bennett + 인접/대각 로터 간섭 + fountain effect
    ↓
③ 고도별 추력 증가율 곡선
    ↓
④ 랜딩기어 높이 설계 기준
   지면효과 임계 고도 (1D, 1.5D, 2D 기준) 도출
    ↓
그래프 출력
```

---

## 암 후류 공력 손실 해석 (`arm_wake_analysis.m`)

```
선정 프롭 (1안 / 2안 자동 전환)
    ↓
① 후류 속도 계산
   v_wake = k · v_i  (k: 암 위치에 따른 계수)
    ↓
② 암 형상별 항력 계산
   D_arm = 0.5·ρ·v_wake²·Cd·A_proj
    ↓
③ 추가 소비전력 계산
   ΔP = D_arm · v_wake
    ↓
④ 허용 손실률 이내 최대 암 두께 도출
   암 단면 형상(원형/타원/NACA) 비교
    ↓
그래프 출력 (손실률 vs 암 두께/형상)
```

---

## 암 길이 해석 (`planar_analysis.m`)

```
선정 프롭 반경 d  (1안: propSpecification / 2안: prop_ff 자동 전환)
기체 질량 base_aum (1안: mass_Total / 2안: mass_Total_ff 자동 전환)
    ↓
[1단계] 암 길이 계산
   L/R spacing = 3.0 기준 (논문 최적값)
   arm_length = (D/2 · √2) / (L/R)
    ↓
[2단계] spacing별 발생 추력 비교
   논문 Figure 4 RPM별 추력 증가율 적용
   base_thrust = TWR × base_aum
    ↓
[3단계] 발생 추력 vs 필요 추력 (암 무게 증분 반영)
   카본 파이프 (OD16/ID14 mm) 단위 무게 계산
   total_aum = base_aum + n_arms × Δarm × weight_per_mm
   margin = thrust_gen - thrust_req
    ↓
최적 spacing 및 암 길이 도출
그래프 출력 (spacing별 추력 여유 비교)
```

---

## 모터 선정 정책 (`load_motorList.m`)

| 조건 | 처리 | 근거 |
|------|------|------|
| 판별식 `disc < 0` | **하드 탈락** | 물리적으로 해 없음 (전압 부족) |
| `I_req > I_max` | **하드 탈락** | 모터 정격 초과, 손상 위험 |
| `kV_margin < 0.7` | **하드 탈락** | 풀스로틀에서도 필요 RPM 70% 미달 |
| `mass > spec_mass` | **소프트 페널티** | 설계 추정값 초과, 조립 시 변동 가능 |
| `kV_margin 0.7~1.0` | **소프트 페널티** | 작동 가능하나 RPM 마진 부족 |

결과는 호버 소비전력 기반 점수 오름차순으로 정렬되어 반환됩니다.

---

## 주요 Workspace 변수

### main.m 출력

| 변수 | 설명 |
|------|------|
| `propList_considered` | 호버 후보 프롭 전체 (`consideredNo × 6` cell) |
| `operatingPoints` | 후보 프롭별 [hover/WOT/limit] 운용점 |
| `temp_propChosen_pos` | 1안 선정 프롭 인덱스 |
| `propSpecification` | 1안 선정 프롭 사양 `{name, diam_in, pitch_in}` |
| `motorChosen` | 1안 선정 모터 (`1×12` cell) |
| `best_esc` | 1안 선정 ESC |
| `mass_Total` | 1안 실측 기체 질량 [g] |
| `mass_Total_Est` | 추정값 기반 질량 [g] (프롭 선정 기준, 재계산 없음) |
| `time_hover` | 1안 호버 비행시간 벡터 [h] |

### forward_flight_analysis.m 출력 (2안)

| 변수 | 설명 |
|------|------|
| `prop_ff` | 2안 선정 프롭 (`1×6` cell) |
| `idx_ff_best` | 2안 선정 프롭 인덱스 (`propList_considered` 기준) |
| `V_max_best` | 최대 전진속도 [m/s] |
| `V_ff` | 운용 전진속도 (0.95 × V_max_best) [m/s] |
| `motorChosen_ff` | 2안 선정 모터 (`1×12` cell) |
| `best_esc_ff` | 2안 선정 ESC |
| `mass_Total_ff` | **2안 수렴 후 최종 기체 질량 [g]** ← downstream 전체 참조 |

---

## 적용 프레임: S500

| 항목 | 값 |
|------|----|
| 휠베이스 | 500 mm |
| 프레임 질량 | 405 g |
| 상면 투영면적 `Stop` | 0.059 m² |
| 정면 투영면적 `Sfront` | 0.021 m² |
| 항력계수 `Cd_frame` | 1.0 (일반 쿼드콥터 오픈 프레임 기준) |
| 권장 프롭 | 9–12 inch |
| 권장 배터리 | 4S 5000 mAh |
| 암 재질 (기준) | 카본 파이프 OD 16 mm / ID 14 mm |

---

## 참고 문헌

- Biczyski et al. (2020). *Multirotor Sizing Methodology with Flight Time Estimation.* Journal of Advanced Transportation.
- Pollet et al. (2021). *Design Optimization of Multirotor Drones in Forward Flight.* ICAS 2021.
- Leng et al. (2019). *An Analytical Model for Propeller Aerodynamic Efforts at High Incidence.*

---

## 요구 환경

- MATLAB R2017b 이상
- 추가 툴박스 불필요