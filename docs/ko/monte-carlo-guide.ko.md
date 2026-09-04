# 🎲 몬테카를로 시뮬레이션 사용 가이드

## 📋 개요

이 프로젝트는 **몬테카를로 시뮬레이션** 방식으로 AMR(Autonomous Mobile Robot) 시스템의 성능을 분석합니다.

### ✨ 주요 기능

1. ✅ **여러 번 시뮬레이션 반복 실행** (설정 가능한 횟수)
2. ✅ **각 반복의 결과 자동 수집**
3. ✅ **통계 분석** (최소/최대/평균/표준편차)
4. ✅ **CSV 로그 저장** (예제 형식: `[#1] avgLeadTime, 값`)
5. ✅ **자동 그래프 생성** (에러바, 표준편차 영역 포함)

---

## 🚀 사용 방법

### 1️⃣ 설정 파일 수정

`JSON/setup.json`에서 몬테카를로 반복 횟수 설정:

```json
{
    "fileName": "setup",
    "numVehicles": 5,
    "numJob": 3,
    "isTerminalOn": true,
    "isVisualizerOn": true,
    "monteCarlo": 10,  // ← 몬테카를로 반복 횟수
    "renderTime": 0.1
}
```

### 2️⃣ 시뮬레이션 실행

```bash
python main.py
```

### 3️⃣ 결과 확인

시뮬레이션이 완료되면 다음 위치에 결과가 저장됩니다:

```
Visualizations/
└── montecarlo_20251031_153045/
    └── analysis/
        ├── avg_lead_time_montecarlo.csv       # 각 반복의 Lead Time
        ├── avg_wait_time_montecarlo.csv       # 각 반복의 Wait Time
        ├── avg_throughput_montecarlo.csv      # 각 반복의 Throughput
        ├── total_jobs_montecarlo.csv          # 각 반복의 완료 Job 수
        ├── montecarlo_summary.txt             # 통계 요약
        ├── montecarlo_lead_time.png           # Lead Time 그래프
        ├── montecarlo_wait_time.png           # Wait Time 그래프
        ├── montecarlo_throughput.png          # Throughput 그래프
        └── montecarlo_all_metrics.png         # 전체 지표 비교
```

---

## 📊 생성되는 데이터

### CSV 파일 형식

**예제: `avg_lead_time_montecarlo.csv`**

```csv
[#1] avg_lead_time,65595.27272727272
[#2] avg_lead_time,70594.81818181818
[#3] avg_lead_time,70690.09090909091
[#4] avg_lead_time,68967.18181818182
[#5] avg_lead_time,67924.45454545454
...
```

### 통계 요약 파일

**예제: `montecarlo_summary.txt`**

```
======================================================================
Monte Carlo Simulation - Statistical Analysis Report
======================================================================

Total Iterations: 10
Analysis Timestamp: 20251031_153045

----------------------------------------------------------------------
Performance Metrics Statistics
----------------------------------------------------------------------

📊 avg_lead_time:
  - Minimum:       62670.45s
  - Maximum:       71365.00s
  - Average:       68122.34s
  - Std Deviation: 2345.67s
  - Range:         8694.55s
  - CV (Std/Mean): 3.44%

📊 avg_wait_time:
  - Minimum:       1234.56s
  - Maximum:       1567.89s
  - Average:       1401.23s
  - Std Deviation: 98.76s
  - Range:         333.33s
  - CV (Std/Mean): 7.05%

...
```

---

## 📈 그래프 설명

### 1. Lead Time 통계 그래프

![Lead Time Example](docs/example_lead_time.png)

- 🔵 **각 반복의 결과**: 파란색 점으로 표시
- 🟢 **평균선**: 녹색 점선
- 🔴/🔵 **최소/최대선**: 빨간색/파란색 점선
- 🟦 **표준편차 영역**: 파란색 반투명 영역 (평균 ± 1σ)

### 2. Wait Time 통계 그래프

Wait Time의 변동성을 시각화하여 시스템 안정성을 평가할 수 있습니다.

### 3. Throughput 통계 그래프

시간당 처리되는 Job 수의 변동성을 확인할 수 있습니다.

### 4. 전체 지표 비교 그래프

모든 지표를 한 화면에서 비교하여 시스템 성능을 종합적으로 파악할 수 있습니다.

---

## 🔧 코드 구조

### 주요 파일

```
(20251031)PyDEVS/
├── main.py                                          # 메인 실행 파일
├── modeling/
│   ├── AMRSimModel.py                              # 시뮬레이션 모델
│   └── experiment/
│       ├── MonteCarloAnalyzer.py                   # 몬테카를로 분석기 ⭐
│       ├── experimental_frame.py                   # 실험 프레임
│       └── atomic/
│           └── Data_collector.py                   # 데이터 수집기 (수정됨)
└── JSON/
    └── setup.json                                  # 설정 파일
```

### 핵심 클래스

#### 1. `MonteCarloAnalyzer` (새로 생성)

```python
from modeling.experiment.MonteCarloAnalyzer import MonteCarloAnalyzer

# 초기화
mc_analyzer = MonteCarloAnalyzer(num_iterations=10)

# 각 반복 결과 추가
mc_analyzer.add_iteration_result(iteration=1, results_dict={
    'avg_lead_time': 65595.27,
    'avg_wait_time': 1234.56,
    'total_jobs': 22,
    'avg_throughput': 0.0033
})

# 통합 분석 수행
mc_analyzer.analyze_and_save()
```

#### 2. `Data_collector` (수정됨)

```python
# 현재 반복의 결과 반환
iteration_results = data_collector.get_iteration_results()
# Returns:
# {
#     'avg_lead_time': 65595.27,
#     'avg_wait_time': 1234.56,
#     'total_jobs': 22,
#     'avg_throughput': 0.0033,
#     'sim_time': 9999.0
# }
```

#### 3. `AMRSimModel` (수정됨)

```python
# 시뮬레이션 모델에서 결과 가져오기
objModels = AMRSimModel(objConfiguration)
# ... 시뮬레이션 실행 ...
results = objModels.get_iteration_results()
```

---

## 🎯 예제 실행 흐름

```python
# 1. 몬테카를로 분석기 초기화
mc_analyzer = MonteCarloAnalyzer(num_iterations=10)

# 2. 10번 반복
for iteration in range(1, 11):
    # 시뮬레이션 실행
    objModels = AMRSimModel(objConfiguration)
    engine = SimulationEngine()
    engine.setOutmostModel(objModels)
    engine.run(maxTime=9999)
    
    # 결과 수집
    results = objModels.get_iteration_results()
    mc_analyzer.add_iteration_result(iteration, results)
    
    print(f"Iteration {iteration} 완료!")
    print(f"  - Lead Time: {results['avg_lead_time']:.2f}s")
    print(f"  - Wait Time: {results['avg_wait_time']:.2f}s")

# 3. 통합 분석
mc_analyzer.analyze_and_save()
```

---

## 📌 주요 지표 설명

| 지표 | 설명 |
|------|------|
| **avg_lead_time** | 각 Job의 시작부터 완료까지 평균 시간 |
| **avg_wait_time** | 각 Job이 대기한 평균 시간 |
| **total_jobs** | 완료된 총 Job 수 |
| **avg_throughput** | 초당 처리되는 Job 수 (jobs/s) |
| **sim_time** | 시뮬레이션 실행 시간 |

---

## 🔍 분석 방법

### 1. 결과의 일관성 확인

- **표준편차(σ)가 작을수록** → 시스템이 안정적
- **CV (변동계수) < 10%** → 결과 신뢰도 높음

### 2. 최소/최대 범위 확인

- **범위가 넓을수록** → 시스템 변동성 큼
- **최악의 경우(Max)** 고려 필요

### 3. 평균 비교

- 여러 설정 조건에서 몬테카를로 시뮬레이션 실행
- **평균값**을 기준으로 성능 비교

---

## ⚙️ 고급 설정

### 반복 횟수 선택 가이드

| 반복 횟수 | 추천 상황 |
|----------|----------|
| 5-10회 | 빠른 테스트, 초기 성능 확인 |
| 20-30회 | 일반적인 분석 |
| 50-100회 | 정밀한 통계 분석 필요 시 |

### 저장 디렉토리 변경

```python
mc_analyzer = MonteCarloAnalyzer(
    num_iterations=10,
    base_save_dir='CustomResults'  # ← 커스텀 경로
)
```

---

## 🐛 문제 해결

### Q1: "No results to analyze!" 에러

**원인**: 시뮬레이션 결과가 수집되지 않음

**해결**:
1. `setup.json`에서 `numJob > 0` 확인
2. `Data_collector`에 `globalVar`가 올바르게 전달되었는지 확인

### Q2: 그래프가 생성되지 않음

**원인**: matplotlib 관련 문제

**해결**:
```bash
pip install matplotlib numpy
```

### Q3: CSV 파일이 비어있음

**원인**: Job 데이터가 수집되지 않음

**해결**:
- `maxTime`을 충분히 크게 설정 (예: 9999)
- 시뮬레이션이 정상적으로 완료되는지 확인

---

## 📚 참고 자료

- OHT 시뮬레이션 예제 코드 기반
- 몬테카를로 시뮬레이션 방법론 적용
- DEVS (Discrete Event System Specification) 기반 구현

---

## 📞 문의

문제가 발생하거나 추가 기능이 필요한 경우:
1. `log.txt` 파일 확인
2. 콘솔 출력 메시지 확인
3. 설정 파일 검토

---

**버전**: 1.0.0  
**최종 업데이트**: 2025-10-31

