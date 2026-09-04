from SimulationEngine.SimulationEngine import SimulationEngine
from Environment.EnvironmentLoader import EnvironmentLoader
from modeling.AMRSimModel import AMRSimModel
from modeling.experiment.MonteCarloAnalyzer import MonteCarloAnalyzer
import time
from datetime import datetime


# 설정 파일 로드
strPath = "JSON/"
lstFileNames = ["setup", "map", "processInfo", "vehicleInfo"]
envLoader = EnvironmentLoader(strPath, lstFileNames)
objConfiguration = envLoader.getConfiguration()

# 실행 모드 설정
monteCarlo = objConfiguration.getConfiguration("monteCarlo")
vehicle_change_mode = objConfiguration.getConfiguration("Vehiclechange")
max_vehicles = objConfiguration.getConfiguration("numVehicles")

# 모든 반복이 공유하는 타임스탬프
shared_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

print(f"\n{'='*70}")
if vehicle_change_mode:
    print(f"🚗 Vehicle Change Mode - Monte Carlo Simulation")
    print(f"   차량 수 범위: 1 ~ {max_vehicles}대")
    print(f"   각 차량 수당 반복: {monteCarlo} iterations")
else:
    print(f"🎲 Monte Carlo Simulation Started ({monteCarlo} iterations)")
print(f"{'='*70}\n")

# 몬테카를로 분석기
mc_analyzer = MonteCarloAnalyzer(
    num_iterations=monteCarlo,
    shared_timestamp=shared_timestamp,
    vehicle_change_mode=vehicle_change_mode
)

# 전체 실행 시작 시각
total_start_time = time.time()
total_sim_time = 0.0

if vehicle_change_mode:
    # ==================== Vehicle Change Mode ====================
    vehicle_counts = list(range(1, max_vehicles + 1))

    for vehicle_count in vehicle_counts:
        scenario_label = f"vehicle_num_{vehicle_count}"

        print(f"\n{'='*70}")
        print(f"🚗 차량 수: {vehicle_count}대 ({vehicle_count}/{max_vehicles})")
        print(f"{'='*70}\n")

        # 시나리오 등록
        mc_analyzer.register_scenario(scenario_label, vehicle_count)

        # 이번 시나리오의 차량 수
        objConfiguration.addConfiguration("numVehicles", vehicle_count)

        for iteration in range(1, monteCarlo + 1):
            print(f"\n{'─'*70}")
            print(f"▶ 차량 {vehicle_count}대 | Iteration {iteration}/{monteCarlo}")
            print(f"{'─'*70}")

            start_time = time.time()

            # 시뮬레이션 모델 생성
            objModels = AMRSimModel(
                objConfiguration,
                iteration_num=iteration,
                scenario_label=scenario_label
            )

            # 엔진 실행
            engine = SimulationEngine()
            engine.setOutmostModel(objModels)
            engine.run(
                maxTime=9999,
                ta=-1,
                logFileName='log.txt',
                logGeneral=False,
                logActivateState=False,
                logActivateMessage=False,
                logActivateTA=False,
                logStructure=False,
            )

            elapsed_time = time.time() - start_time

            # 결과 수집
            iteration_results = objModels.get_iteration_results()
            sim_time = iteration_results.get('sim_time', engine.getTime())
            total_sim_time += sim_time

            # 실행 시간 정보 추가
            time_ratio = elapsed_time / sim_time if sim_time > 0 else 0
            iteration_results['real_time'] = elapsed_time
            iteration_results['time_ratio'] = time_ratio
            iteration_results['vehicle_count'] = vehicle_count

            # 결과 저장
            mc_analyzer.add_iteration_result(
                iteration, iteration_results, scenario_label=scenario_label)

            print(
                f"\n✓ 차량 {vehicle_count}대 | Iteration {iteration}/{monteCarlo} 완료")
            print(f"  - Simulation Time: {sim_time:.2f}s")
            print(f"  - Real Time: {elapsed_time:.2f}s")
            print(f"  - Lead Time: {iteration_results['avg_lead_time']:.2f}s")
            print(f"  - Wait Time: {iteration_results['avg_wait_time']:.2f}s")

else:
    # ==================== 단일 차량 수 Monte Carlo Mode ====================
    for iteration in range(1, monteCarlo + 1):
        print(f"\n{'─'*70}")
        print(f"▶ Iteration {iteration}/{monteCarlo} - 시뮬레이션 시작")
        print(f"{'─'*70}")

        start_time = time.time()

        # 시뮬레이션 모델 생성
        objModels = AMRSimModel(objConfiguration, iteration_num=iteration)

        # 엔진 실행
        engine = SimulationEngine()
        engine.setOutmostModel(objModels)
        engine.run(
            maxTime=9999,
            ta=-1,
            logFileName='log.txt',
            logGeneral=False,
            logActivateState=False,
            logActivateMessage=False,
            logActivateTA=False,
            logStructure=False,
        )

        elapsed_time = time.time() - start_time

        # 결과 수집
        iteration_results = objModels.get_iteration_results()
        sim_time = iteration_results.get('sim_time', engine.getTime())
        total_sim_time += sim_time

        # 실행 시간 정보 추가
        time_ratio = elapsed_time / sim_time if sim_time > 0 else 0
        iteration_results['real_time'] = elapsed_time
        iteration_results['time_ratio'] = time_ratio

        # 결과 저장
        mc_analyzer.add_iteration_result(iteration, iteration_results)

        print(f"\n✓ Iteration {iteration}/{monteCarlo} 완료")
        print(f"{'─'*70}")
        print(f"⏱️  시간 측정:")
        print(f"  - Simulation Time: {sim_time:.2f}s (시뮬레이션 내부 시간)")
        print(f"  - Real Time: {elapsed_time:.2f}s (실제 실행 시간)")
        print(f"  - 비율 (Real/Sim): {time_ratio:.3f}x", end="")
        if time_ratio < 1.0:
            print(f" → 실시간보다 {1/time_ratio:.1f}배 빠르게 실행 ⚡")
        else:
            print(f" → 실시간보다 {time_ratio:.1f}배 느리게 실행")
        print(f"\n📊 성능 지표:")
        print(f"  - 평균 Lead Time: {iteration_results['avg_lead_time']:.2f}s")
        print(f"  - 평균 Wait Time: {iteration_results['avg_wait_time']:.2f}s")
        print(f"  - 완료된 Job 수: {iteration_results['total_jobs']}")
        print(
            f"  - Throughput: {iteration_results['avg_throughput']:.4f} jobs/s")
        print(f"\n🛠️  알고리즘 성능:")
        print(
            f"  - Global Planner: {iteration_results['global_planner_time']:.3f}s ({iteration_results['global_planner_calls']} calls)")
        print(
            f"  - Local Planner: {iteration_results['local_planner_time']:.3f}s ({iteration_results['local_planner_calls']} calls)")

# 반복 전체에 대한 통합 분석
print(f"\n{'='*70}")
print(f"📊 Monte Carlo 통합 분석 시작...")
print(f"{'='*70}\n")

mc_analyzer.analyze_and_save()

# 알고리즘 성능 통계
if not vehicle_change_mode:
    print("\n🎯 Final Algorithm Statistics:")
    objModels.globalVar.print_algorithm_statistics()

# 전체 실행 시간 집계
total_elapsed_time = time.time() - total_start_time
total_time_ratio = total_elapsed_time / \
    total_sim_time if total_sim_time > 0 else 0

print(f"\n{'='*70}")
print(f"✅ All Simulations Complete!")
print(f"{'='*70}")
print(f"\n⏱️  총 시뮬레이션 시간:")

if vehicle_change_mode:
    total_iterations = monteCarlo * len(vehicle_counts)
    print(f"  - 차량 수 범위: 1 ~ {max_vehicles}대")
    print(
        f"  - 총 시뮬레이션 횟수: {total_iterations} ({monteCarlo} iter × {len(vehicle_counts)} vehicles)")
else:
    print(
        f"  - Total Simulation Time: {total_sim_time:.2f}s ({monteCarlo} iterations)")

print(f"  - Total Real Time: {total_elapsed_time:.2f}s")
print(f"  - 평균 비율 (Real/Sim): {total_time_ratio:.3f}x", end="")
if total_time_ratio < 1.0:
    print(f" → 실시간보다 {1/total_time_ratio:.1f}배 빠르게 실행 ⚡")
else:
    print(f" → 실시간보다 {total_time_ratio:.1f}배 느리게 실행")

if vehicle_change_mode:
    avg_time_per_scenario = total_elapsed_time / len(vehicle_counts)
    print(f"  - 차량 수당 평균 Real Time: {avg_time_per_scenario:.2f}s")
else:
    print(f"  - Iteration당 평균 Real Time: {total_elapsed_time/monteCarlo:.2f}s")

print(f"{'='*70}\n")
