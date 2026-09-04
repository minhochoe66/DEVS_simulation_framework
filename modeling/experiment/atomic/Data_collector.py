from SimulationEngine.ClassicDEVS.DEVSAtomicModel import DEVSAtomicModel
import os
import csv
import json
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np


class _MinimalPoseBuffer:
    """AMR 포즈를 버퍼링하여 CSV로 저장하는 최소 구현."""

    _global_timestamp = None
    _global_base_dir = None  # 실행 전체의 기본 디렉터리

    def __init__(self, iteration_num=None, scenario_label=None):
        # 타임스탬프는 한 번만 정하고 모든 반복이 공유한다
        if _MinimalPoseBuffer._global_timestamp is None:
            _MinimalPoseBuffer._global_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.timestamp = _MinimalPoseBuffer._global_timestamp
        self.iteration_num = iteration_num  # 몬테카를로 반복 번호
        self.scenario_label = scenario_label  # 시나리오 레이블 (차량 수 변경 모드)
        self.base_dir = None
        self.agent_dir = None
        self.data = {}  # vehicle_id -> last tuple

    def add_pose(self, pose):
        # 필요한 필드: pose.strID, pose.x, pose.y, pose.yaw, pose.lin_vel, pose.ang_vel
        vehicle_name = getattr(pose, 'strID', 'Vehicle')
        self.data[vehicle_name] = (
            getattr(pose, 'x', 0.0),
            getattr(pose, 'y', 0.0),
            getattr(pose, 'yaw', 0.0),
            getattr(pose, 'lin_vel', 0.0),
            getattr(pose, 'ang_vel', 0.0),
        )

    def _ensure_dirs(self, base_directory, globalVar=None):
        if self.base_dir is None:
            # 타임스탬프 디렉터리
            if _MinimalPoseBuffer._global_base_dir is None:
                _MinimalPoseBuffer._global_base_dir = os.path.join(
                    base_directory, self.timestamp)

            # 경로는 <timestamp>/<scenario>/iteration_X 또는 <timestamp>/iteration_X
            if self.scenario_label:
                # Vehicle change mode: <timestamp>/<scenario_label>/iteration_X
                scenario_dir = os.path.join(_MinimalPoseBuffer._global_base_dir, self.scenario_label)
                if self.iteration_num is not None:
                    self.base_dir = os.path.join(scenario_dir, f'iteration_{self.iteration_num}')
                else:
                    self.base_dir = scenario_dir
            else:
                # 단일 차량 수 모드
                if self.iteration_num is not None:
                    self.base_dir = os.path.join(
                        _MinimalPoseBuffer._global_base_dir, f'iteration_{self.iteration_num}')
                else:
                    self.base_dir = _MinimalPoseBuffer._global_base_dir

            self.agent_dir = os.path.join(self.base_dir, 'Agent')
            os.makedirs(self.agent_dir, exist_ok=True)

            # 이 반복이 실제로 쓴 레이아웃을 함께 남긴다
            if globalVar:
                self._save_map_config(globalVar)

    def _save_map_config(self, globalVar):
        """필터링된 장비와 대기 구역을 map.json으로 저장한다."""
        try:
            equipmentInfo = globalVar.getEquipmentInfo()

            # Equipment 객체를 dict로 변환
            equipment_list = []
            for eq_id, eq in equipmentInfo.items():
                eq_dict = {
                    "equipmentID": eq.strEquipmentID,
                    "processType": eq.strType,
                    "stageID": eq.strStageID,
                    "processTime": eq.dblProcessTime,
                    "exchangeTime": eq.dblExchangeTime,
                    "performance": eq.strPerformance,
                    "inputPort": eq.inputPort,
                    "outputPort": eq.outputPort,
                    "workPosition": eq.workPosition
                }
                equipment_list.append(eq_dict)

            # WaitingArea 객체를 dict로 변환
            waiting_area_list = []
            waitingAreaInfo = globalVar.getWaitingAreaInfo()
            if waitingAreaInfo:
                for area_id, area in waitingAreaInfo.items():
                    area_dict = {
                        "areaID": area.strAreaID,
                        "position": area.position,
                        "boundingBox": area.boundingBox
                    }
                    waiting_area_list.append(area_dict)

            map_data = {
                "fileName": "map",
                "equipmentInfo": equipment_list,
                "WatingareaInfo": waiting_area_list
            }

            # 타임스탬프 폴더에 저장
            map_file_path = os.path.join(self.base_dir, 'map.json')
            with open(map_file_path, 'w', encoding='utf-8') as f:
                json.dump(map_data, f, indent=4, ensure_ascii=False)

            print(
                f"📋 Saved map config: {map_file_path} ({len(equipment_list)} equipment, {len(waiting_area_list)} waiting areas)")
        except Exception as e:
            print(f"⚠️ Failed to save map config: {e}")

    def flush_latest_to_csv(self, base_directory, globalVar=None):
        if not self.data:
            return
        self._ensure_dirs(base_directory, globalVar)
        for vehicle_name, pose in self.data.items():
            # 파일명은 VEHICLE 접두어만 남기고 _maneuver 같은 접미어는 뗀다
            vehicle_file_id = vehicle_name.split('_')[0]
            file_path = os.path.join(self.agent_dir, f"{vehicle_file_id}.csv")
            is_new = not os.path.exists(file_path)
            with open(file_path, 'a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=[
                                        'x', 'y', 'yaw', 'linear_velocity', 'angular_velocity'])
                if is_new:
                    writer.writeheader()
                writer.writerow({
                    'x': pose[0],
                    'y': pose[1],
                    'yaw': pose[2],
                    'linear_velocity': pose[3],
                    'angular_velocity': pose[4],
                })


class Data_collector(DEVSAtomicModel):
    """AMR 시뮬레이션 데이터를 수집하고 분석하는 원자 모델.

    주행 중에는 차량 포즈를 모아 CSV로 남기고, 시뮬레이션이 끝나면
    작업 성능을 분석하여 그래프와 리포트를 만든다.

    입력 포트
        MyManeuverState_I:    자기 차량 포즈
        OtherManeuverState_I: 다른 차량 포즈 (선택)
        Complete_I:           저장을 마무리하는 트리거

    저장 위치
        Visualizations/<timestamp>/.../Agent/VEHICLE*.csv   차량 궤적
        Visualizations/<timestamp>/.../analysis/            작업 성능 분석
    """

    def __init__(self, ID, objConfiguration=None, globalVar=None, base_save_dir=None, iteration_num=None, scenario_label=None):
        super().__init__(ID)

        self.objConfiguration = objConfiguration
        self.globalVar = globalVar
        self.iteration_num = iteration_num  # 몬테카를로 반복 번호
        self.scenario_label = scenario_label  # 시나리오 레이블
        self.addStateVariable('state', 'INIT')

        # 입력 포트
        self.addInputPort('MyManeuverState_I')
        self.addInputPort('OtherManeuverState_I')
        self.addInputPort('Complete_I')

        # 포즈 버퍼
        self._pose_buffer = _MinimalPoseBuffer(iteration_num=iteration_num, scenario_label=scenario_label)
        # 저장 경로 우선순위: 생성자 인자 > 설정 객체 > 기본값
        if isinstance(base_save_dir, str) and base_save_dir.strip():
            self._base_save_dir = base_save_dir
        elif hasattr(self.objConfiguration, 'getConfiguration'):
            try:
                self._base_save_dir = self.objConfiguration.getConfiguration(
                    'pose_save_dir') or 'Visualizations'
            except Exception:
                self._base_save_dir = 'Visualizations'
        else:
            self._base_save_dir = 'Visualizations'

        # 분석 그래프 색상
        self.colorList = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    def funcExternalTransition(self, strPort, objEvent):
        if strPort == 'MyManeuverState_I' or strPort == 'OtherManeuverState_I':
            self._pose_buffer.add_pose(objEvent)
            self.setStateValue('state', 'SAVE')
        elif strPort == 'Complete_I':
            self.setStateValue('state', 'DONE')
        return True

    def funcInternalTransition(self):
        state = self.getStateValue('state')
        if state == 'SAVE':
            # 즉시 저장하고 대기로 돌아간다
            self._pose_buffer.flush_latest_to_csv(
                self._base_save_dir, self.globalVar)
            self.setStateValue('state', 'INIT')
        elif state == 'DONE':
            # 마지막 저장을 마치고 종료를 기다린다
            self._pose_buffer.flush_latest_to_csv(
                self._base_save_dir, self.globalVar)

            # 작업 성능 분석
            if self.globalVar:
                self._analyze_job_performance()

            self.setStateValue('state', 'INIT')
        return True

    def funcOutput(self):
        return False

    def funcTimeAdvance(self):
        state = self.getStateValue('state')
        if state == 'INIT':
            return float('inf')
        # SAVE와 DONE은 내부 전이로 곧바로 저장한다
        return 0

    def funcSelect(self):
        pass

    def _analyze_job_performance(self):
        """작업 성능을 분석하고 그래프와 리포트를 만든다."""
        sim_time = self.getTime()

        # 분석 디렉터리를 만든다
        # 몬테카를로 실행이면 반복 폴더 아래에 둔다
        # 단일 실행이면 base_dir 아래에 둔다
        if self.iteration_num is not None:
            analysis_dir = os.path.join(self._pose_buffer.base_dir, 'analysis')
        else:
            if _MinimalPoseBuffer._global_base_dir:
                analysis_dir = os.path.join(
                    _MinimalPoseBuffer._global_base_dir, 'analysis')
            else:
                analysis_dir = os.path.join(
                    self._pose_buffer.base_dir, 'analysis')
        os.makedirs(analysis_dir, exist_ok=True)

        # 작업 데이터 수집
        job_data = self._collect_job_data()

        if not job_data:
            print(f"[{sim_time}][Data_collector] No job data available for analysis")
            return

        # CSV 저장
        self._save_job_csv(job_data, analysis_dir)

        # 그래프
        self._plot_lead_time(job_data, analysis_dir)
        self._plot_wait_time(job_data, analysis_dir)
        self._plot_throughput(job_data, sim_time, analysis_dir)

        # 요약 리포트
        self._save_summary_report(job_data, sim_time, analysis_dir)

        print(
            f"[{sim_time}][Data_collector] Job analysis complete! Results saved to {analysis_dir}")

    def get_iteration_results(self):
        """이번 반복의 성능 지표를 반환한다.

        리드 타임, 대기 시간, 처리량, 완료 작업 수, 시뮬레이션 시간과
        전역 및 지역 플래너의 계산 시간, 호출 횟수, 평균을 담은 dict다.
        """
        sim_time = self.getTime()
        job_data = self._collect_job_data()

        # 플래너 계산 시간 통계
        global_time = self.globalVar.GlobalPlanner_algorithm_time if self.globalVar else 0.0
        global_calls = self.globalVar.GlobalPlanner_call_count if self.globalVar else 0
        global_avg = global_time / global_calls if global_calls > 0 else 0.0

        local_time = self.globalVar.LocalPlanner_algorithm_time if self.globalVar else 0.0
        local_calls = self.globalVar.LocalPlanner_call_count if self.globalVar else 0
        local_avg = local_time / local_calls if local_calls > 0 else 0.0

        if not job_data:
            return {
                'avg_lead_time': 0.0,
                'avg_wait_time': 0.0,
                'total_jobs': 0,
                'avg_throughput': 0.0,
                'sim_time': sim_time,
                'global_planner_time': global_time,
                'global_planner_calls': global_calls,
                'global_planner_avg': global_avg,
                'local_planner_time': local_time,
                'local_planner_calls': local_calls,
                'local_planner_avg': local_avg
            }

        # 평균 리드 타임
        lead_times = [d['lead_time'] for d in job_data]
        avg_lead_time = np.mean(lead_times) if lead_times else 0.0

        # 평균 대기 시간
        wait_times = [d['total_wait_time'] for d in job_data]
        avg_wait_time = np.mean(wait_times) if wait_times else 0.0

        # 처리량
        avg_throughput = len(job_data) / sim_time if sim_time > 0 else 0.0

        return {
            'avg_lead_time': avg_lead_time,
            'avg_wait_time': avg_wait_time,
            'total_jobs': len(job_data),
            'avg_throughput': avg_throughput,
            'sim_time': sim_time,
            'global_planner_time': global_time,
            'global_planner_calls': global_calls,
            'global_planner_avg': global_avg,
            'local_planner_time': local_time,
            'local_planner_calls': local_calls,
            'local_planner_avg': local_avg
        }

    def _collect_job_data(self):
        """완료된 작업의 리드 타임과 대기 시간을 모은다."""
        job_data = []
        target_jobs = self.globalVar.getTargetJobs()

        for job_id, job_info in target_jobs.items():
            if not job_info.dictStartTime or not job_info.dictOutTime:
                continue

            start_times = list(job_info.dictStartTime.values())
            out_times = list(job_info.dictOutTime.values())

            if not start_times or not out_times:
                continue

            start_time = start_times[0]
            complete_time = out_times[-1]
            lead_time = complete_time - start_time

            # 대기 시간
            total_wait_time = 0
            for eqp_id in job_info.dictDoneTime.keys():
                if eqp_id in job_info.dictOutTime:
                    wait = job_info.dictOutTime[eqp_id] - \
                        job_info.dictDoneTime[eqp_id]
                    total_wait_time += wait

            job_data.append({
                'job_id': job_id,
                'start_time': start_time,
                'complete_time': complete_time,
                'lead_time': lead_time,
                'total_wait_time': total_wait_time,
                'num_processes': len(job_info.dictStartTime)
            })

        return job_data

    def _save_job_csv(self, job_data, analysis_dir):
        """작업별 지표를 CSV로 저장한다."""
        csv_path = os.path.join(analysis_dir, 'job_summary.csv')

        with open(csv_path, 'w', newline='') as csvfile:
            fieldnames = ['job_id', 'start_time', 'complete_time', 'lead_time',
                          'total_wait_time', 'num_processes']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(job_data)

    def _plot_lead_time(self, job_data, analysis_dir):
        """리드 타임 그래프."""
        job_ids = [int(d['job_id']) for d in job_data]
        lead_times = [d['lead_time'] for d in job_data]

        plt.figure(figsize=(12, 6))
        plt.bar(job_ids, lead_times, color=self.colorList[0], alpha=0.7)
        plt.xlabel('Job ID')
        plt.ylabel('Lead Time (s)')
        plt.title(f'Job Lead Time Analysis (Avg: {np.mean(lead_times):.2f}s)')
        plt.grid(axis='y', alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'lead_time.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_wait_time(self, job_data, analysis_dir):
        """대기 시간 그래프."""
        job_ids = [int(d['job_id']) for d in job_data]
        wait_times = [d['total_wait_time'] for d in job_data]

        plt.figure(figsize=(12, 6))
        plt.bar(job_ids, wait_times, color=self.colorList[1], alpha=0.7)
        plt.xlabel('Job ID')
        plt.ylabel('Total Wait Time (s)')
        plt.title(f'Job Wait Time Analysis (Avg: {np.mean(wait_times):.2f}s)')
        plt.grid(axis='y', alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'wait_time.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_throughput(self, job_data, sim_time, analysis_dir):
        """작업 완료 시각 분포와 처리량 그래프."""
        complete_times = sorted([d['complete_time'] for d in job_data])
        job_count = list(range(1, len(complete_times) + 1))

        plt.figure(figsize=(12, 6))
        plt.plot(complete_times, job_count, color=self.colorList[2],
                 linewidth=2, marker='o', markersize=4)
        plt.xlabel('Simulation Time (s)')
        plt.ylabel('Cumulative Jobs Completed')
        plt.title(f'Job Completion Timeline (Total: {len(job_data)} jobs)')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'job_throughput.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _save_summary_report(self, job_data, sim_time, analysis_dir):
        """요약 리포트를 저장한다."""
        report_path = os.path.join(analysis_dir, 'summary_report.txt')

        with open(report_path, 'w') as f:
            f.write("=" * 60 + "\n")
            f.write("AMR Simulation - Performance Analysis Report\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"Simulation Time: {sim_time:.2f}s\n")
            f.write(f"Analysis Timestamp: {self._pose_buffer.timestamp}\n\n")

            # Job Statistics
            f.write("-" * 60 + "\n")
            f.write("Job Performance Metrics\n")
            f.write("-" * 60 + "\n")
            f.write(f"Total Jobs Completed: {len(job_data)}\n\n")

            # 리드 타임 통계
            lead_times = [d['lead_time'] for d in job_data]
            f.write("Lead Time Statistics:\n")
            f.write(f"  - Average: {np.mean(lead_times):.2f}s\n")
            f.write(f"  - Min: {np.min(lead_times):.2f}s\n")
            f.write(f"  - Max: {np.max(lead_times):.2f}s\n")
            f.write(f"  - Std Dev: {np.std(lead_times):.2f}s\n\n")

            # 대기 시간 통계
            wait_times = [d['total_wait_time'] for d in job_data]
            f.write("Wait Time Statistics:\n")
            f.write(f"  - Average: {np.mean(wait_times):.2f}s\n")
            f.write(f"  - Min: {np.min(wait_times):.2f}s\n")
            f.write(f"  - Max: {np.max(wait_times):.2f}s\n")
            f.write(f"  - Std Dev: {np.std(wait_times):.2f}s\n\n")

            # 처리량
            if sim_time > 0:
                throughput = len(job_data) / sim_time * 3600  # jobs/hour
                f.write(f"Throughput: {throughput:.2f} jobs/hour\n")
                avg_cycle_time = sim_time / \
                    len(job_data) if len(job_data) > 0 else 0
                f.write(f"Average Cycle Time: {avg_cycle_time:.2f}s\n\n")

            # 공정 수 통계
            num_processes = [d['num_processes'] for d in job_data]
            f.write("Process Statistics:\n")
            f.write(
                f"  - Average Processes per Job: {np.mean(num_processes):.1f}\n")
            f.write(f"  - Min Processes: {np.min(num_processes)}\n")
            f.write(f"  - Max Processes: {np.max(num_processes)}\n\n")

            f.write("=" * 60 + "\n")
            f.write("Vehicle trajectory data: Agent/Vehicle*.csv\n")
            f.write("=" * 60 + "\n")
