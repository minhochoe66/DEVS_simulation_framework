# 📁 몬테카를로 시뮬레이션 폴더 구조 가이드

## ✅ 수정 완료! 이제 올바른 폴더 구조로 저장됩니다

---

## 📊 **새로운 폴더 구조**

```
Visualizations/
└── 20251031_154707/                    # 시뮬레이션 시작 시간 (모든 반복 공유)
    ├── iteration_1/                    # 몬테카를로 1번째 반복
    │   ├── Agent/
    │   │   ├── VEHICLE0000000001.csv   # 1번째 반복의 차량 1 궤적
    │   │   ├── VEHICLE0000000002.csv   # 1번째 반복의 차량 2 궤적
    │   │   ├── VEHICLE0000000003.csv
    │   │   ├── VEHICLE0000000004.csv
    │   │   └── VEHICLE0000000005.csv
    │   └── analysis/                   # 1번째 반복의 개별 Job 분석
    │       ├── job_summary.csv
    │       ├── lead_time.png
    │       ├── wait_time.png
    │       ├── job_throughput.png
    │       └── summary_report.txt
    │
    ├── iteration_2/                    # 몬테카를로 2번째 반복
    │   ├── Agent/
    │   │   ├── VEHICLE0000000001.csv   # 2번째 반복의 차량 1 궤적
    │   │   ├── VEHICLE0000000002.csv   # 2번째 반복의 차량 2 궤적
    │   │   ├── VEHICLE0000000003.csv
    │   │   ├── VEHICLE0000000004.csv
    │   │   └── VEHICLE0000000005.csv
    │   └── analysis/                   # 2번째 반복의 개별 Job 분석
    │       ├── job_summary.csv
    │       ├── lead_time.png
    │       ├── wait_time.png
    │       ├── job_throughput.png
    │       └── summary_report.txt
    │
    ├── iteration_3/                    # 몬테카를로 3번째 반복
    │   ├── Agent/
    │   │   └── ...
    │   └── analysis/
    │       └── ...
    │
    └── analysis/                       # 몬테카를로 통합 분석 결과 ⭐
        ├── avg_lead_time_montecarlo.csv
        ├── avg_wait_time_montecarlo.csv
        ├── avg_throughput_montecarlo.csv
        ├── total_jobs_montecarlo.csv
        ├── sim_time_montecarlo.csv
        ├── global_planner_time_montecarlo.csv      # ⭐ 알고리즘 통계
        ├── global_planner_calls_montecarlo.csv     # ⭐ 알고리즘 통계
        ├── local_planner_time_montecarlo.csv       # ⭐ 알고리즘 통계
        ├── local_planner_calls_montecarlo.csv      # ⭐ 알고리즘 통계
        ├── montecarlo_lead_time.png
        ├── montecarlo_wait_time.png
        ├── montecarlo_throughput.png
        ├── montecarlo_algorithm_performance.png    # ⭐ 알고리즘 성능 그래프
        ├── montecarlo_all_metrics.png
        └── montecarlo_summary.txt                  # ⭐ 알고리즘 통계 포함
```

---

## 🔧 **수정된 파일 목록**

### 1️⃣ `modeling/experiment/atomic/Data_collector.py`
- ✅ `_MinimalPoseBuffer`에 `iteration_num` 추가
- ✅ 각 반복마다 `iteration_X` 폴더 생성
- ✅ `analysis` 폴더는 전역 base_dir 아래 생성 (iteration 폴더 아님)
- ✅ 각 반복마다 새로운 CSV 파일 생성 (이어쓰기 방지)

### 2️⃣ `modeling/experiment/experimental_frame.py`
- ✅ `ExperimentalFrame`에 `iteration_num` 파라미터 추가
- ✅ `Data_collector`에 `iteration_num` 전달

### 3️⃣ `modeling/AMRSimModel.py`
- ✅ `AMRSimModel`에 `iteration_num` 파라미터 추가
- ✅ `ExperimentalFrame`에 `iteration_num` 전달

### 4️⃣ `modeling/experiment/MonteCarloAnalyzer.py`
- ✅ 타임스탬프 동기화 (Data_collector와 같은 폴더 사용)
- ✅ `analysis` 폴더 경로 수정 (`montecarlo_` 접두사 제거)

### 5️⃣ `main.py`
- ✅ `AMRSimModel` 생성 시 `iteration_num` 전달

---

## 🎯 **작동 원리**

### **타임스탬프 공유**
```python
# 첫 번째 iteration에서 타임스탬프 생성
_MinimalPoseBuffer._global_timestamp = "20251031_154707"
_MinimalPoseBuffer._global_base_dir = "Visualizations/20251031_154707"

# 모든 iteration에서 같은 타임스탬프 사용
# → 같은 폴더 아래 iteration_1, iteration_2, ... 생성
```

### **각 반복마다 독립적인 데이터**
```python
# iteration 1
iteration_num=1 → base_dir = "Visualizations/20251031_154707/iteration_1"
                  agent_dir = "Visualizations/20251031_154707/iteration_1/Agent"

# iteration 2
iteration_num=2 → base_dir = "Visualizations/20251031_154707/iteration_2"
                  agent_dir = "Visualizations/20251031_154707/iteration_2/Agent"
```

### **통합 분석은 전역 폴더**
```python
# analysis 폴더는 iteration 폴더가 아닌 전역 base_dir 아래 생성
analysis_dir = "Visualizations/20251031_154707/analysis"
```

---

## 📝 **CSV 파일 형식**

### **Vehicle 궤적 파일 예시**
`iteration_1/Agent/VEHICLE0000000001.csv`:
```csv
x,y,yaw,linear_velocity,angular_velocity
50.0,50.0,0.0,1.5,0.0
51.5,50.0,0.0,1.5,0.0
53.0,50.0,0.0,1.5,0.0
...
```

### **몬테카를로 분석 파일 예시**
`analysis/avg_lead_time_montecarlo.csv`:
```csv
[#1] avg_lead_time,65595.27272727272
[#2] avg_lead_time,70594.81818181818
[#3] avg_lead_time,70690.09090909091
...
```

---

## 🚀 **실행 방법**

```bash
python main.py
```

**예상 출력:**
```
======================================================================
🎲 Monte Carlo Simulation Started (3 iterations)
======================================================================

──────────────────────────────────────────────────────────────────────
▶ Iteration 1/3 - 시뮬레이션 시작
──────────────────────────────────────────────────────────────────────
...
[Data_collector] Vehicle data saved to: Visualizations/20251031_154707/iteration_1/Agent/
✓ Iteration 1/3 완료

──────────────────────────────────────────────────────────────────────
▶ Iteration 2/3 - 시뮬레이션 시작
──────────────────────────────────────────────────────────────────────
...
[Data_collector] Vehicle data saved to: Visualizations/20251031_154707/iteration_2/Agent/
✓ Iteration 2/3 완료

──────────────────────────────────────────────────────────────────────
▶ Iteration 3/3 - 시뮬레이션 시작
──────────────────────────────────────────────────────────────────────
...
[Data_collector] Vehicle data saved to: Visualizations/20251031_154707/iteration_3/Agent/
✓ Iteration 3/3 완료

======================================================================
📊 Monte Carlo 통합 분석 시작...
======================================================================
[MonteCarloAnalyzer] Analysis complete! Results saved to: Visualizations/20251031_154707/analysis
```

---

## ✅ **문제 해결**

### ❌ **이전 문제들**

1. ❌ **Vehicle CSV가 이어서 저장됨**
   - 원인: 같은 폴더에 계속 append 모드로 저장
   - 해결: 각 iteration마다 `iteration_X` 폴더 생성

2. ❌ **폴더가 두 개 생성됨**
   - 원인: Data_collector와 MonteCarloAnalyzer가 다른 타임스탬프 사용
   - 해결: 타임스탬프 동기화

3. ❌ **데이터 초기화 안됨**
   - 원인: 클래스 변수로 data 공유
   - 해결: 각 iteration마다 새로운 _MinimalPoseBuffer 인스턴스 생성

### ✅ **이제 모두 해결!**

---

## 📌 **주요 변경 사항 요약**

| 변경 사항 | 이전 | 이후 |
|----------|------|------|
| 폴더 구조 | `20251031_154707/Agent/` (모든 반복 혼재) | `20251031_154707/iteration_X/Agent/` (반복별 분리) |
| 타임스탬프 | Data_collector와 MonteCarloAnalyzer가 각각 생성 | 동기화하여 같은 폴더 사용 |
| CSV 초기화 | 이어쓰기됨 | 각 반복마다 새 파일 |
| 개별 분석 | 덮어쓰기됨 | `iteration_X/analysis/`에 각각 저장 |
| 통합 분석 | `montecarlo_20251031_154707/analysis/` | `20251031_154707/analysis/` |

---

## 📂 **두 가지 Analysis 폴더**

### 1️⃣ **개별 Analysis** (`iteration_X/analysis/`)
- 각 몬테카를로 반복의 상세 Job 분석
- Lead Time, Wait Time, Throughput 그래프
- Job별 상세 데이터 CSV
- **용도**: 특정 반복의 상세 분석, 이상값 확인

### 2️⃣ **통합 Analysis** (`analysis/`)
- 모든 반복을 종합한 통계 분석
- 최소/최대/평균/표준편차 계산
- 몬테카를로 시뮬레이션 결과 시각화
- **용도**: 전체 시스템 성능 평가, 신뢰도 분석

---

## 🎉 **완료!**

이제 몬테카를로 시뮬레이션을 실행하면 깔끔하게 정리된 폴더 구조로 결과가 저장됩니다!

각 반복의 Vehicle 궤적 데이터와 통합 분석 결과를 쉽게 찾을 수 있습니다.

