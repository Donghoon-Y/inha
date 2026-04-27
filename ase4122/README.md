# Multirotor Sizing Methodology

멀티로터 드론의 **프로펠러 · 모터 · ESC · 배터리**를 자동으로 선정하고, 체공시간과 전진비행 성능을 추정하는 MATLAB 설계 도구입니다.

Biczyski et al. (2020) *"Multirotor Sizing Methodology with Flight Time Estimation"* 의 방법론을 기반으로 하며, 전진비행 해석(Pollet et al., ICAS 2021) 및 공력 해석 모듈이 추가되었습니다.

---

## 설계 방향

두 가지 설계안을 병렬로 도출합니다.

| 구분 | 목표 | 최적화 기준 |
|------|------|-------------|
| **설계 1안 (Endurance)** | 체공시간 최대화 | 호버링 비추력(g/W) 최대 |
| **설계 2안 (Maneuverability)** | 왕복비행 시간 최소화 | 여유추력(T_surplus) 최대 → 가속도 최대 |

---

## 실행 순서

```
1. main.m                   → 설계 1안: 호버 최적 prop / motor / ESC / 배터리 선정
2. forward_flight_analysis.m → 설계 2안: 전진비행 기동성 최적 prop / motor / ESC 선정
3. aero_analysis.m           → 선정 결과의 공력 해석 (Disk Loading, Figure of Merit)
```

> **주의**: `forward_flight_analysis.m`과 `aero_analysis.m`은 반드시 `main.m` 실행 후 동일 워크스페이스에서 실행해야 합니다.

---

## 파일 구조

```
.
├── main.m                        # 설계 1안 메인 스크립트
├── forward_flight_analysis.m     # 설계 2안: 전진비행 기동성 최적화
├── aero_analysis.m               # 공력 해석 (Disk Loading, FM, 지면효과)
│
├── src/
│   ├── load_propList.m           # APC 프롭 목록 로딩 및 필터링
│   ├── load_propPerf.m           # APC 정적 성능 데이터 파싱
│   ├── load_motorList.m          # 모터 필터링 및 점수 기반 정렬
│   ├── load_escList.m            # ESC 목록 로딩
│   ├── compare_hoverTime.m       # 설계안 간 체공시간 비교
│   └── validate_propPerf.m       # 프롭 성능 데이터 검증
│
├── plot/
│   ├── plot_propPerf.m           # 프롭 성능 곡선 + 배터리 시뮬레이션 그래프
│   └── plot_motorPerf.m          # 모터 성능 그래프
│
├── APC_Prop/
│   ├── PER2_STATIC-2.DAT         # APC 전 기종 정적 성능 통합 데이터
│   ├── WEBLIST.xlsx              # APC 프롭 전체 목록
│   ├── PROP-DATA-FILE_202602.xlsx # APC 치수/무게 데이터 (2026년 2월 기준)
│   └── PERFILES2/                # 기종별 개별 .dat 파일 (~300개)
│
├── UIUC_Prop/                    # UIUC 프롭 데이터베이스 (예비)
├── motor_list/Motor_list.xlsx    # 모터 DB (kV, Rm, I_max, 질량, I0)
└── ESC_list/ESC_data.xlsx        # ESC DB (정격전류, 질량)
```

---

## 주요 파라미터 설정 (`main.m`)

```matlab
RotorNo          = 4;        % 로터 수
OptimisationGoal = 'hover';  % 'hover' | 'max' | 'utilisation'
ThrustWeightRatio = 2;       % 추력-중량비 (2=최소, 3=수송, 5+=곡예)
PropDiameter_Min = 9;        % 프롭 직경 하한 [inch]
PropDiameter_Max = 12;       % 프롭 직경 상한 [inch]
AcceptedTypes    = {'MR' 'E' 'E-3'};  % 허용 프롭 시리즈

BattCellNo       = 4;        % 배터리 셀 수 (S)
BattCapacity     = 5000;     % 배터리 용량 [mAh]

mass_Frame       = 405;      % 프레임 질량 [g]  ← S500 기준
mass_Motor_Est   = 93;       % 모터 질량 추정값 [g]
mass_Payload     = 500;      % 페이로드 [g]
```

---

## 설계 1안 흐름 (`main.m`)

```
사용자 파라미터 입력
    ↓
① 프롭 후보 필터링
   직경 범위 + 허용 타입 조건
    ↓
② APC 정적 성능 데이터 로딩
   RPM vs Thrust / Power / Torque / Ct / Cp
    ↓
③ 운용점 계산 (보간)
   hover 운용점 / WOT 운용점 / RPM 한계 운용점
    ↓
④ 최적 프롭 선정
   비추력(g/W) 최대화
    ↓
⑤ 모터 선정
   전압·토크·전류 조건 + 호버 소비전력 최소화
    ↓
⑥ 배터리 시뮬레이션 (Peukert 방정식)
   호버 / WOT 비행시간 추정
    ↓
⑦ ESC 선정 (전류 여유 20%)
    ↓
결과 출력 + 그래프
```

---

## 설계 2안 흐름 (`forward_flight_analysis.m`)

설계 1안의 호버 후보 프롭(`propList_considered`)을 그대로 계승하여 전진비행 성능을 평가합니다.

```
설계 1안 호버 후보 프롭 (propList_considered)
    ↓
① 전진비행 힘 평형 (Pollet Eq. 3, 수치 반복)
   기체 경사각 α 수렴 계산
    ↓
② CT / CP 전진비행 보정 (Pollet Eq. 8-9)
   Leng et al. 비축 입사각 모델 적용
    ↓
③ 기동성 점수 계산 (왕복 시간 추정)
   T_surplus → a_max → t_lap (삼각형 속도 프로파일)
    ↓
④ 최적 프롭 선정
   10회 왕복 시간 최소화 (= 여유추력 최대화)
    ↓
⑤ 모터 선정
   하드 필터: disc<0, I_max 초과, kV 마진 < 0.7
   소프트 페널티: 질량 초과, kV 마진 0.7~1.0
    ↓
⑥ ESC 선정 (전류 여유 20%)
    ↓
설계 1안 vs 2안 비교 출력 + 여유추력 곡선 그래프
```

### 왕복 시간 추정 수식

$$T_{surplus} = N_{rotors} \cdot T_{max} - W$$

$$a_{max} = \frac{\sqrt{T_{total}^2 - W^2}}{m}$$

$$t_{lap} = 20 \times 2\sqrt{\frac{L}{a_{max}}} \quad (L = 2\,\text{m},\ 10\text{회 왕복})$$

> 항력 및 제어 지연을 무시한 이상적 조건의 하한 추정값입니다. 프롭 간 상대적 성능 비교에 활용합니다.

---

## 모터 선정 정책 (`load_motorList.m`)

| 조건 | 처리 | 근거 |
|------|------|------|
| 판별식 `disc < 0` | **하드 탈락** | 물리적으로 해 없음 |
| `I_req > I_max` | **하드 탈락** | 모터 정격 초과, 손상 위험 |
| `kV 마진 < 0.7` | **하드 탈락** | 풀스로틀에서도 필요 RPM 70% 미달 |
| `mass > spec_mass` | **소프트 페널티** | 설계 추정값, 실제 조립 시 변동 가능 |
| `kV 마진 0.7~1.0` | **소프트 페널티** | 작동 가능하나 마진 부족 |

결과는 호버 소비전력 기반 점수 오름차순으로 정렬되어 반환됩니다.

---

## 적용 프레임: S500

| 항목 | 값 |
|------|----|
| 휠베이스 | 500 mm |
| 프레임 질량 | 405 g |
| 상면 투영면적 `Stop` | 0.059 m² |
| 정면 투영면적 `Sfront` | 0.021 m² |
| 항력계수 `Cd` | 1.0 (일반 쿼드콥터 기준) |
| 권장 프롭 | 9–12 inch |
| 권장 배터리 | 4S 5000 mAh |

---

## 참고 문헌

- Biczyski et al. (2020). *Multirotor Sizing Methodology with Flight Time Estimation.* Journal of Advanced Transportation.
- Pollet et al. (2021). *Design Optimization of Multirotor Drones in Forward Flight.* ICAS 2021.
- Leng et al. (2019). *An Analytical Model for Propeller Aerodynamic Efforts at High Incidence.*

---

## 요구 환경

- MATLAB R2017b 이상
- 추가 툴박스 불필요
