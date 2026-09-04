"""Aggregation of Monte Carlo replication results.

Computes the minimum, maximum, mean and standard deviation of each metric
across replications, and writes the per-scenario CSVs and error-bar figures.
Handles both a single scenario and a sweep over fleet sizes.
"""

import os
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from datetime import datetime


def _configure_matplotlib_font():
    """Select a font that can render Korean labels on this platform."""
    try:
        font_list = [f.name for f in fm.fontManager.ttflist]
        
        # per-platform preference order
        if sys.platform.startswith('win'):
            korean_fonts = ['Malgun Gothic', 'NanumGothic', 'NanumBarunGothic', 'Gulim', 'Dotum', 'Batang']
        elif sys.platform == 'darwin':  # macOS
            korean_fonts = ['AppleGothic', 'NanumGothic', 'NanumBarunGothic']
        else:  # Linux
            korean_fonts = ['NanumGothic', 'NanumBarunGothic', 'UnDotum']
        
        korean_font = None
        for font_name in korean_fonts:
            if font_name in font_list:
                korean_font = font_name
                break
        
        if korean_font:
            plt.rcParams['font.family'] = korean_font
            plt.rcParams['axes.unicode_minus'] = False
            print(f"[MonteCarloAnalyzer] 한글 폰트 설정 완료: {korean_font}")
        else:
            plt.rcParams['font.family'] = 'DejaVu Sans'
            plt.rcParams['axes.unicode_minus'] = False
            print("[MonteCarloAnalyzer] ⚠️ 한글 폰트를 찾을 수 없습니다. 기본 폰트를 사용합니다.")
    except Exception as e:
        print(f"[MonteCarloAnalyzer] ⚠️ 폰트 설정 중 오류 발생: {e}")
        plt.rcParams['font.family'] = 'DejaVu Sans'
        plt.rcParams['axes.unicode_minus'] = False


# initialise the font
_configure_matplotlib_font()


class MonteCarloAnalyzer:
    """Aggregates Monte Carlo results, for a single scenario or a sweep."""

    # a class-level timestamp, kept in step with Data_collector
    _shared_timestamp = None

    def __init__(self, num_iterations, base_save_dir='Visualizations', shared_timestamp=None, vehicle_change_mode=False):
        """shared_timestamp makes this write into the same folder as Data_collector."""
        self.num_iterations = num_iterations
        self.base_save_dir = base_save_dir
        self.vehicle_change_mode = vehicle_change_mode

        # synchronise the timestamp
        if shared_timestamp:
            self.timestamp = shared_timestamp
        elif MonteCarloAnalyzer._shared_timestamp:
            self.timestamp = MonteCarloAnalyzer._shared_timestamp
        else:
            self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            MonteCarloAnalyzer._shared_timestamp = self.timestamp

        # result store
        if vehicle_change_mode:
            # sweep: {scenario_label: {iter_num: {metric: value}}}
            self.iteration_results = {}
            self.scenario_labels = {}  # {scenario_label: vehicle_count}
        else:
            # single scenario: {iter_num: {metric: value}}
            self.iteration_results = {}

        # colour palette
        self.colorList = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    def register_scenario(self, scenario_label, vehicle_count):
        """Register a scenario. Used only in fleet-size sweep mode.

        scenario_label looks like "vehicle_num_3".
        """
        if self.vehicle_change_mode:
            if scenario_label not in self.iteration_results:
                self.iteration_results[scenario_label] = {}
                self.scenario_labels[scenario_label] = vehicle_count
                print(f"[MonteCarloAnalyzer] 시나리오 등록: {scenario_label} (차량 {vehicle_count}대)")

    def add_iteration_result(self, iter_num, results_dict, scenario_label=None):
        """Store the result of one replication.

        results_dict maps metric names to values. In fleet-size sweep mode,
        scenario_label is required.
        """
        if self.vehicle_change_mode:
            if scenario_label is None:
                raise ValueError("vehicle_change_mode에서는 scenario_label이 필요합니다.")
            if scenario_label not in self.iteration_results:
                self.iteration_results[scenario_label] = {}
            self.iteration_results[scenario_label][iter_num] = results_dict
            vehicle_count = self.scenario_labels.get(scenario_label, "?")
            print(f"[MonteCarloAnalyzer] {scenario_label} | Iteration #{iter_num}/{self.num_iterations} 저장 완료")
        else:
            self.iteration_results[iter_num] = results_dict
            print(f"[MonteCarloAnalyzer] Iteration #{iter_num}/{self.num_iterations} 저장 완료")

    def analyze_and_save(self):
        """Aggregate once every replication has finished."""
        if not self.iteration_results:
            print("[MonteCarloAnalyzer] 분석할 결과가 없습니다!")
            return

        # create the analysis directory
        analysis_dir = os.path.join(self.base_save_dir, self.timestamp, 'analysis')
        os.makedirs(analysis_dir, exist_ok=True)

        if self.vehicle_change_mode:
            self._analyze_vehicle_change_mode(analysis_dir)
        else:
            self._analyze_single_mode(analysis_dir)

        print(f"\n[MonteCarloAnalyzer] ✅ 분석 완료! 결과 저장 위치: {analysis_dir}\n")

    def _analyze_single_mode(self, analysis_dir):
        """Analyse a single scenario."""
        print(f"\n{'='*70}")
        print(f"📊 Monte Carlo 분석 ({len(self.iteration_results)} iterations)")
        print(f"{'='*70}\n")

        # statistics
        statistics = self._calculate_statistics_single()
        
        # CSV
        self._save_iteration_logs_single(analysis_dir)
        
        # summary report
        self._save_statistics_summary_single(statistics, analysis_dir)
        
        # error-bar figure showing mean and standard deviation
        self._plot_summary_errorbar(statistics, analysis_dir)

    def _analyze_vehicle_change_mode(self, analysis_dir):
        """Analyse the sweep over fleet sizes."""
        print(f"\n{'='*70}")
        print(f"🚗 Vehicle Change Mode 분석")
        print(f"{'='*70}\n")

        # per-scenario statistics
        scenario_statistics = self._calculate_statistics_multi_scenario()
        
        # CSV
        self._save_scenario_logs(analysis_dir, scenario_statistics)
        
        # summary report
        self._save_scenario_summary(scenario_statistics, analysis_dir)
        
        # error-bar figure against fleet size
        self._plot_vehicle_comparison(scenario_statistics, analysis_dir)

    def _calculate_statistics_single(self):
        """Compute the metric statistics for a single scenario."""
        statistics = {}
        if not self.iteration_results:
            return statistics

        first_iter = next(iter(self.iteration_results.values()))
        metrics = first_iter.keys()

        for metric_name in metrics:
            values = []
            for iter_num in sorted(self.iteration_results.keys()):
                value = self.iteration_results[iter_num].get(metric_name)
                if value is not None and isinstance(value, (int, float)):
                    values.append(value)

            if values:
                statistics[metric_name] = {
                    'values': values,
                    'min': np.min(values),
                    'max': np.max(values),
                    'mean': np.mean(values),
                    'std': np.std(values)
                }

        return statistics

    def _calculate_statistics_multi_scenario(self):
        """Compute the metric statistics for each fleet size."""
        scenario_statistics = {}
        
        for scenario_label in sorted(self.iteration_results.keys()):
            iter_results = self.iteration_results[scenario_label]
            if not iter_results:
                continue

            first_iter = next(iter(iter_results.values()))
            metrics = first_iter.keys()

            scenario_statistics[scenario_label] = {}
            for metric_name in metrics:
                values = []
                for iter_num in sorted(iter_results.keys()):
                    value = iter_results[iter_num].get(metric_name)
                    if value is not None and isinstance(value, (int, float)):
                        values.append(value)

                if values:
                    scenario_statistics[scenario_label][metric_name] = {
                        'values': values,
                        'mean': np.mean(values),
                        'std': np.std(values),
                        'min': np.min(values),
                        'max': np.max(values)
                    }

        return scenario_statistics

    def _save_iteration_logs_single(self, analysis_dir):
        """Write the single-scenario results to CSV."""
        if not self.iteration_results:
            return

        first_iter = next(iter(self.iteration_results.values()))
        metrics = first_iter.keys()

        precision_map = {
            'avg_lead_time': 2, 'avg_wait_time': 2, 'avg_throughput': 6,
            'sim_time': 2, 'real_time': 2, 'time_ratio': 4,
            'global_planner_time': 3, 'local_planner_time': 3,
            'global_planner_avg': 3, 'local_planner_avg': 3,
            'total_jobs': 0, 'global_planner_calls': 0, 'local_planner_calls': 0,
        }

        for metric_name in metrics:
            csv_path = os.path.join(analysis_dir, f'{metric_name}_montecarlo.csv')
            with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for iter_num in sorted(self.iteration_results.keys()):
                    value = self.iteration_results[iter_num].get(metric_name, 'N/A')
                    if value != 'N/A' and isinstance(value, (int, float)):
                        precision = precision_map.get(metric_name, 4)
                        value = int(value) if precision == 0 else round(value, precision)
                    writer.writerow([f'[#{iter_num}] {metric_name}', value])

    def _save_scenario_logs(self, analysis_dir, scenario_statistics):
        """Write the fleet-size sweep results to CSV."""
        if not scenario_statistics:
            return

        # one file per metric
        first_scenario = next(iter(scenario_statistics.values()))
        metrics = first_scenario.keys()

        for metric_name in metrics:
            csv_path = os.path.join(analysis_dir, f'{metric_name}_by_vehicle.csv')
            with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Vehicle_Count', 'Mean', 'Std', 'Min', 'Max'])
                
                for scenario_label in sorted(scenario_statistics.keys()):
                    vehicle_count = self.scenario_labels.get(scenario_label, 0)
                    stats = scenario_statistics[scenario_label].get(metric_name)
                    if stats:
                        writer.writerow([
                            vehicle_count,
                            round(stats['mean'], 4),
                            round(stats['std'], 4),
                            round(stats['min'], 4),
                            round(stats['max'], 4)
                        ])

    def _save_iteration_logs(self, analysis_dir):
        """Write one row per replication, formatted as "[#1] avgLeadTime, 65595.27"."""
        # a separate CSV per metric
        if not self.iteration_results:
            return

        # take the metric list from the first replication
        first_iter = next(iter(self.iteration_results.values()))
        metrics = first_iter.keys()

        # decimal places per metric
        precision_map = {
            'avg_lead_time': 2,
            'avg_wait_time': 2,
            'avg_throughput': 6,  # throughput is small, so it needs more digits
            'sim_time': 2,
            'real_time': 2,
            'time_ratio': 4,
            'global_planner_time': 3,
            'local_planner_time': 3,
            'global_planner_avg': 3,
            'local_planner_avg': 3,
            'total_jobs': 0,
            'global_planner_calls': 0,
            'local_planner_calls': 0,
        }

        for metric_name in metrics:
            csv_path = os.path.join(
                analysis_dir, f'{metric_name}_montecarlo.csv')

            with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)

                # one row per replication
                for iter_num in sorted(self.iteration_results.keys()):
                    value = self.iteration_results[iter_num].get(
                        metric_name, 'N/A')
                    
                    # format numeric values
                    if value != 'N/A' and isinstance(value, (int, float)):
                        precision = precision_map.get(metric_name, 4)
                        if precision == 0:
                            value = int(value)
                        else:
                            value = round(value, precision)
                    
                    writer.writerow([f'[#{iter_num}] {metric_name}', value])

    def _calculate_statistics(self):
        """Compute the minimum, maximum, mean and standard deviation of each metric.

        Returns {metric: {'values': [...], 'min': ..., 'max': ..., 'mean': ..., 'std': ...}}.
        """
        statistics = {}

        if not self.iteration_results:
            return statistics

        # take the metric list from the first replication
        first_iter = next(iter(self.iteration_results.values()))
        metrics = first_iter.keys()

        for metric_name in metrics:
            # gather that metric across every replication
            values = []
            for iter_num in sorted(self.iteration_results.keys()):
                value = self.iteration_results[iter_num].get(metric_name)
                if value is not None and isinstance(value, (int, float)):
                    values.append(value)

            if values:
                statistics[metric_name] = {
                    'values': values,
                    'min': np.min(values),
                    'max': np.max(values),
                    'mean': np.mean(values),
                    'std': np.std(values)
                }

        return statistics

    def _save_statistics_summary(self, statistics, analysis_dir):
        """Write the statistical summary report."""
        report_path = os.path.join(analysis_dir, 'montecarlo_summary.txt')

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("Monte Carlo Simulation - Statistical Analysis Report\n")
            f.write("=" * 70 + "\n\n")

            f.write(f"Total Iterations: {len(self.iteration_results)}\n")
            f.write(f"Analysis Timestamp: {self.timestamp}\n\n")

            # Job Performance Metrics
            f.write("-" * 70 + "\n")
            f.write("📦 Job Performance Metrics Statistics\n")
            f.write("-" * 70 + "\n\n")

            job_metrics = ['avg_lead_time', 'avg_wait_time',
                           'total_jobs', 'avg_throughput', 'sim_time']
            for metric_name in job_metrics:
                if metric_name in statistics:
                    stats = statistics[metric_name]
                    f.write(f"📊 {metric_name}:\n")
                    f.write(f"  - Minimum:       {stats['min']:.4f}\n")
                    f.write(f"  - Maximum:       {stats['max']:.4f}\n")
                    f.write(f"  - Average:       {stats['mean']:.4f}\n")
                    f.write(f"  - Std Deviation: {stats['std']:.4f}\n")
                    f.write(
                        f"  - Range:         {stats['max'] - stats['min']:.4f}\n")
                    if stats['mean'] != 0:
                        f.write(
                            f"  - CV (Std/Mean): {(stats['std'] / stats['mean'] * 100):.2f}%\n")
                    f.write("\n")

            # Time Performance Metrics
            f.write("-" * 70 + "\n")
            f.write("⏱️  Time Performance Statistics\n")
            f.write("-" * 70 + "\n\n")

            time_metrics = ['sim_time', 'real_time', 'time_ratio']
            for metric_name in time_metrics:
                if metric_name in statistics:
                    stats = statistics[metric_name]
                    display_name = {
                        'sim_time': 'Simulation Time (시뮬레이션 내부 시간)',
                        'real_time': 'Real Time (실제 실행 시간)',
                        'time_ratio': 'Time Ratio (Real/Sim)'
                    }
                    f.write(
                        f"📊 {display_name.get(metric_name, metric_name)}:\n")
                    f.write(f"  - Minimum:       {stats['min']:.4f}\n")
                    f.write(f"  - Maximum:       {stats['max']:.4f}\n")
                    f.write(f"  - Average:       {stats['mean']:.4f}\n")
                    f.write(f"  - Std Deviation: {stats['std']:.4f}\n")

                    if metric_name == 'time_ratio':
                        if stats['mean'] < 1.0:
                            f.write(
                                f"  - 성능:          실시간보다 {1/stats['mean']:.1f}배 빠름 ⚡\n")
                        else:
                            f.write(
                                f"  - 성능:          실시간보다 {stats['mean']:.1f}배 느림\n")
                    f.write("\n")

            # Algorithm Performance Metrics
            f.write("-" * 70 + "\n")
            f.write("🔧 Algorithm Performance Statistics\n")
            f.write("-" * 70 + "\n\n")

            # Global Planner
            if 'global_planner_time' in statistics:
                f.write("🌐 Global Planner:\n")
                time_stats = statistics['global_planner_time']
                f.write(f"  Total Time:\n")
                f.write(
                    f"    - Average:       {time_stats['mean']:.3f} seconds\n")
                f.write(
                    f"    - Std Deviation: {time_stats['std']:.3f} seconds\n")
                f.write(
                    f"    - Min / Max:     {time_stats['min']:.3f} / {time_stats['max']:.3f} seconds\n\n")

                if 'global_planner_calls' in statistics:
                    call_stats = statistics['global_planner_calls']
                    f.write(f"  Call Count:\n")
                    f.write(
                        f"    - Average:       {call_stats['mean']:.1f} calls\n")
                    f.write(
                        f"    - Std Deviation: {call_stats['std']:.1f} calls\n")
                    f.write(
                        f"    - Min / Max:     {int(call_stats['min'])} / {int(call_stats['max'])} calls\n\n")

            # Local Planner
            if 'local_planner_time' in statistics:
                f.write("📍 Local Planner (DWA):\n")
                time_stats = statistics['local_planner_time']
                f.write(f"  Total Time:\n")
                f.write(
                    f"    - Average:       {time_stats['mean']:.3f} seconds\n")
                f.write(
                    f"    - Std Deviation: {time_stats['std']:.3f} seconds\n")
                f.write(
                    f"    - Min / Max:     {time_stats['min']:.3f} / {time_stats['max']:.3f} seconds\n\n")

                if 'local_planner_calls' in statistics:
                    call_stats = statistics['local_planner_calls']
                    f.write(f"  Call Count:\n")
                    f.write(
                        f"    - Average:       {call_stats['mean']:.1f} calls\n")
                    f.write(
                        f"    - Std Deviation: {call_stats['std']:.1f} calls\n")
                    f.write(
                        f"    - Min / Max:     {int(call_stats['min'])} / {int(call_stats['max'])} calls\n\n")

            f.write("=" * 70 + "\n")

    def _plot_lead_time_statistics(self, statistics, analysis_dir):
        """Lead-time figure."""
        if 'avg_lead_time' not in statistics:
            return

        stats = statistics['avg_lead_time']
        iterations = list(range(1, len(stats['values']) + 1))

        plt.figure(figsize=(14, 8))

        # individual replications
        plt.plot(iterations, stats['values'], 'o-',
                 color=self.colorList[0], alpha=0.6, linewidth=1.5,
                 markersize=6, label='각 반복 결과')

        # mean
        plt.axhline(y=stats['mean'], color='green', linestyle='--',
                    linewidth=2, label=f"평균: {stats['mean']:.2f}s")

        # minimum and maximum
        plt.axhline(y=stats['min'], color='blue', linestyle=':',
                    linewidth=1.5, label=f"최소: {stats['min']:.2f}s")
        plt.axhline(y=stats['max'], color='red', linestyle=':',
                    linewidth=1.5, label=f"최대: {stats['max']:.2f}s")

        # standard-deviation band
        plt.fill_between(iterations,
                         stats['mean'] - stats['std'],
                         stats['mean'] + stats['std'],
                         alpha=0.2, color=self.colorList[0],
                         label=f'±1 표준편차 ({stats["std"]:.2f}s)')

        plt.xlabel('반복 횟수 (Iteration)', fontsize=12)
        plt.ylabel('평균 Lead Time (초)', fontsize=12)
        plt.title(f'몬테카를로 시뮬레이션 - Lead Time 통계\n({len(iterations)} 반복)',
                  fontsize=14, fontweight='bold')
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'montecarlo_lead_time.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_wait_time_statistics(self, statistics, analysis_dir):
        """Wait-time figure."""
        if 'avg_wait_time' not in statistics:
            return

        stats = statistics['avg_wait_time']
        iterations = list(range(1, len(stats['values']) + 1))

        plt.figure(figsize=(14, 8))

        plt.plot(iterations, stats['values'], 's-',
                 color=self.colorList[1], alpha=0.6, linewidth=1.5,
                 markersize=6, label='각 반복 결과')

        plt.axhline(y=stats['mean'], color='green', linestyle='--',
                    linewidth=2, label=f"평균: {stats['mean']:.2f}s")

        plt.axhline(y=stats['min'], color='blue', linestyle=':',
                    linewidth=1.5, label=f"최소: {stats['min']:.2f}s")
        plt.axhline(y=stats['max'], color='red', linestyle=':',
                    linewidth=1.5, label=f"최대: {stats['max']:.2f}s")

        plt.fill_between(iterations,
                         stats['mean'] - stats['std'],
                         stats['mean'] + stats['std'],
                         alpha=0.2, color=self.colorList[1],
                         label=f'±1 표준편차 ({stats["std"]:.2f}s)')

        plt.xlabel('반복 횟수 (Iteration)', fontsize=12)
        plt.ylabel('평균 Wait Time (초)', fontsize=12)
        plt.title(f'몬테카를로 시뮬레이션 - Wait Time 통계\n({len(iterations)} 반복)',
                  fontsize=14, fontweight='bold')
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'montecarlo_wait_time.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_throughput_statistics(self, statistics, analysis_dir):
        """Throughput figure."""
        if 'avg_throughput' not in statistics:
            return

        stats = statistics['avg_throughput']
        iterations = list(range(1, len(stats['values']) + 1))

        plt.figure(figsize=(14, 8))

        plt.plot(iterations, stats['values'], '^-',
                 color=self.colorList[2], alpha=0.6, linewidth=1.5,
                 markersize=6, label='각 반복 결과')

        plt.axhline(y=stats['mean'], color='green', linestyle='--',
                    linewidth=2, label=f"평균: {stats['mean']:.4f} jobs/s")

        plt.fill_between(iterations,
                         stats['mean'] - stats['std'],
                         stats['mean'] + stats['std'],
                         alpha=0.2, color=self.colorList[2],
                         label=f'±1 표준편차 ({stats["std"]:.4f})')

        plt.xlabel('반복 횟수 (Iteration)', fontsize=12)
        plt.ylabel('Throughput (jobs/s)', fontsize=12)
        plt.title(f'몬테카를로 시뮬레이션 - Throughput 통계\n({len(iterations)} 반복)',
                  fontsize=14, fontweight='bold')
        plt.legend(loc='best', fontsize=10)
        plt.grid(True, alpha=0.3)
        plt.tight_layout()

        save_path = os.path.join(analysis_dir, 'montecarlo_throughput.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_algorithm_performance(self, statistics, analysis_dir):
        """Planner computation-time figure."""
        # check that both planner statistics are present
        has_global = 'global_planner_time' in statistics
        has_local = 'local_planner_time' in statistics

        if not has_global and not has_local:
            return

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('알고리즘 성능 통계 (몬테카를로 시뮬레이션)',
                     fontsize=16, fontweight='bold')

        # Global Planner Time
        if has_global:
            ax = axes[0, 0]
            stats = statistics['global_planner_time']
            iterations = list(range(1, len(stats['values']) + 1))

            ax.plot(iterations, stats['values'], 'o-',
                    color=self.colorList[3], alpha=0.6, linewidth=1.5, markersize=5)
            ax.axhline(y=stats['mean'], color='green',
                       linestyle='--', linewidth=2)
            ax.fill_between(iterations,
                            stats['mean'] - stats['std'],
                            stats['mean'] + stats['std'],
                            alpha=0.2, color=self.colorList[3])

            ax.set_xlabel('Iteration', fontsize=10)
            ax.set_ylabel('Time (s)', fontsize=10)
            ax.set_title(
                f'Global Planner Total Time\n평균: {stats["mean"]:.3f}s ± {stats["std"]:.3f}s', fontsize=11)
            ax.grid(True, alpha=0.3)

        # Local Planner Time
        if has_local:
            ax = axes[0, 1]
            stats = statistics['local_planner_time']
            iterations = list(range(1, len(stats['values']) + 1))

            ax.plot(iterations, stats['values'], 's-',
                    color=self.colorList[4], alpha=0.6, linewidth=1.5, markersize=5)
            ax.axhline(y=stats['mean'], color='green',
                       linestyle='--', linewidth=2)
            ax.fill_between(iterations,
                            stats['mean'] - stats['std'],
                            stats['mean'] + stats['std'],
                            alpha=0.2, color=self.colorList[4])

            ax.set_xlabel('Iteration', fontsize=10)
            ax.set_ylabel('Time (s)', fontsize=10)
            ax.set_title(
                f'Local Planner Total Time\n평균: {stats["mean"]:.3f}s ± {stats["std"]:.3f}s', fontsize=11)
            ax.grid(True, alpha=0.3)

        # Global Planner Call Count
        if 'global_planner_calls' in statistics:
            ax = axes[1, 0]
            stats = statistics['global_planner_calls']
            iterations = list(range(1, len(stats['values']) + 1))

            ax.bar(iterations, stats['values'],
                   color=self.colorList[3], alpha=0.6)
            ax.axhline(y=stats['mean'], color='red', linestyle='--',
                       linewidth=2, label=f'평균: {stats["mean"]:.1f}')

            ax.set_xlabel('Iteration', fontsize=10)
            ax.set_ylabel('Call Count', fontsize=10)
            ax.set_title(
                f'Global Planner Call Count\n평균: {stats["mean"]:.1f} calls', fontsize=11)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3, axis='y')

        # Local Planner Call Count
        if 'local_planner_calls' in statistics:
            ax = axes[1, 1]
            stats = statistics['local_planner_calls']
            iterations = list(range(1, len(stats['values']) + 1))

            ax.bar(iterations, stats['values'],
                   color=self.colorList[4], alpha=0.6)
            ax.axhline(y=stats['mean'], color='red', linestyle='--',
                       linewidth=2, label=f'평균: {stats["mean"]:.1f}')

            ax.set_xlabel('Iteration', fontsize=10)
            ax.set_ylabel('Call Count', fontsize=10)
            ax.set_title(
                f'Local Planner Call Count\n평균: {stats["mean"]:.1f} calls', fontsize=11)
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        save_path = os.path.join(
            analysis_dir, 'montecarlo_algorithm_performance.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_time_performance(self, statistics, analysis_dir):
        """Compare simulated time against wall-clock time."""
        has_sim_time = 'sim_time' in statistics
        has_real_time = 'real_time' in statistics
        has_time_ratio = 'time_ratio' in statistics

        if not (has_sim_time and has_real_time):
            return

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('시뮬레이션 시간 성능 분석 (몬테카를로)',
                     fontsize=16, fontweight='bold')

        # 1. Simulation Time
        ax = axes[0, 0]
        stats = statistics['sim_time']
        iterations = list(range(1, len(stats['values']) + 1))

        ax.plot(iterations, stats['values'], 'o-',
                color=self.colorList[5], alpha=0.6, linewidth=1.5, markersize=6)
        ax.axhline(y=stats['mean'], color='green',
                   linestyle='--', linewidth=2, label=f"평균: {stats['mean']:.2f}s")
        ax.fill_between(iterations,
                        stats['mean'] - stats['std'],
                        stats['mean'] + stats['std'],
                        alpha=0.2, color=self.colorList[5])

        ax.set_xlabel('Iteration', fontsize=11)
        ax.set_ylabel('Simulation Time (s)', fontsize=11)
        ax.set_title(f'시뮬레이션 내부 시간\n평균: {stats["mean"]:.2f}s ± {stats["std"]:.2f}s',
                     fontsize=12, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        # 2. Real Time
        ax = axes[0, 1]
        stats = statistics['real_time']

        ax.plot(iterations, stats['values'], 's-',
                color=self.colorList[6], alpha=0.6, linewidth=1.5, markersize=6)
        ax.axhline(y=stats['mean'], color='green',
                   linestyle='--', linewidth=2, label=f"평균: {stats['mean']:.2f}s")
        ax.fill_between(iterations,
                        stats['mean'] - stats['std'],
                        stats['mean'] + stats['std'],
                        alpha=0.2, color=self.colorList[6])

        ax.set_xlabel('Iteration', fontsize=11)
        ax.set_ylabel('Real Time (s)', fontsize=11)
        ax.set_title(f'실제 실행 시간\n평균: {stats["mean"]:.2f}s ± {stats["std"]:.2f}s',
                     fontsize=12, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        # 3. Time Ratio
        if has_time_ratio:
            ax = axes[1, 0]
            stats = statistics['time_ratio']

            ax.plot(iterations, stats['values'], '^-',
                    color=self.colorList[7], alpha=0.6, linewidth=1.5, markersize=6)
            ax.axhline(y=stats['mean'], color='green',
                       linestyle='--', linewidth=2, label=f"평균: {stats['mean']:.3f}x")
            ax.axhline(y=1.0, color='red', linestyle=':',
                       linewidth=1.5, label='실시간 (1.0x)')
            ax.fill_between(iterations,
                            stats['mean'] - stats['std'],
                            stats['mean'] + stats['std'],
                            alpha=0.2, color=self.colorList[7])

            ax.set_xlabel('Iteration', fontsize=11)
            ax.set_ylabel('Time Ratio (Real/Sim)', fontsize=11)

            # performance summary line
            if stats['mean'] < 1.0:
                performance_msg = f"실시간보다 {1/stats['mean']:.1f}배 빠름 ⚡"
            else:
                performance_msg = f"실시간보다 {stats['mean']:.1f}배 느림"

            ax.set_title(f'시간 비율 (Real/Sim)\n평균: {stats["mean"]:.3f}x - {performance_msg}',
                         fontsize=12, fontweight='bold')
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)

        # 4. simulated time against wall-clock time, as a scatter
        ax = axes[1, 1]
        sim_values = statistics['sim_time']['values']
        real_values = statistics['real_time']['values']

        ax.scatter(sim_values, real_values,
                   s=100, alpha=0.6, color=self.colorList[8], edgecolors='black', linewidth=1)

        # y = x marks real time
        max_val = max(max(sim_values), max(real_values))
        ax.plot([0, max_val], [0, max_val], 'r--', linewidth=2,
                alpha=0.5, label='실시간 기준 (Real=Sim)')

        # trend line
        z = np.polyfit(sim_values, real_values, 1)
        p = np.poly1d(z)
        ax.plot(sim_values, p(sim_values), 'b-', linewidth=2, alpha=0.7,
                label=f'추세선 (y={z[0]:.2f}x+{z[1]:.2f})')

        ax.set_xlabel('Simulation Time (s)', fontsize=11)
        ax.set_ylabel('Real Time (s)', fontsize=11)
        ax.set_title('시뮬레이션 시간 vs 실제 시간',
                     fontsize=12, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = os.path.join(
            analysis_dir, 'montecarlo_time_performance.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _plot_all_metrics_comparison(self, statistics, analysis_dir):
        """Render every metric on one sheet."""
        num_metrics = len(statistics)
        if num_metrics == 0:
            return

        fig, axes = plt.subplots(num_metrics, 1, figsize=(14, 5 * num_metrics))

        if num_metrics == 1:
            axes = [axes]

        for idx, (metric_name, stats) in enumerate(statistics.items()):
            ax = axes[idx]
            iterations = list(range(1, len(stats['values']) + 1))

            # box-plot styling
            ax.plot(iterations, stats['values'], 'o-',
                    color=self.colorList[idx % len(self.colorList)],
                    alpha=0.6, linewidth=1.5, markersize=5)

            ax.axhline(y=stats['mean'], color='green', linestyle='--',
                       linewidth=2, alpha=0.7)
            ax.axhline(y=stats['min'], color='blue', linestyle=':',
                       linewidth=1, alpha=0.5)
            ax.axhline(y=stats['max'], color='red', linestyle=':',
                       linewidth=1, alpha=0.5)

            ax.fill_between(iterations,
                            stats['mean'] - stats['std'],
                            stats['mean'] + stats['std'],
                            alpha=0.15, color=self.colorList[idx % len(self.colorList)])

            ax.set_xlabel('Iteration', fontsize=10)
            ax.set_ylabel(metric_name, fontsize=10)
            ax.set_title(f'{metric_name} (평균: {stats["mean"]:.2f}, 표준편차: {stats["std"]:.2f})',
                         fontsize=11)
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = os.path.join(analysis_dir, 'montecarlo_all_metrics.png')
        plt.savefig(save_path, dpi=150)
        plt.close()

    def _save_statistics_summary_single(self, statistics, analysis_dir):
        """Write the single-scenario summary."""
        report_path = os.path.join(analysis_dir, 'montecarlo_summary.txt')
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("Monte Carlo 시뮬레이션 통계 분석 리포트\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"총 반복 횟수: {len(self.iteration_results)}\n")
            f.write(f"분석 시각: {self.timestamp}\n\n")
            
            # headline metrics
            f.write("-" * 70 + "\n")
            f.write("📊 주요 성능 지표\n")
            f.write("-" * 70 + "\n\n")
            
            key_metrics = ['avg_lead_time', 'avg_wait_time', 'avg_throughput', 'sim_time']
            for metric in key_metrics:
                if metric in statistics:
                    s = statistics[metric]
                    f.write(f"{metric}:\n")
                    f.write(f"  평균: {s['mean']:.4f} ± {s['std']:.4f}\n")
                    f.write(f"  범위: [{s['min']:.4f}, {s['max']:.4f}]\n\n")
            
            f.write("=" * 70 + "\n")

    def _save_scenario_summary(self, scenario_statistics, analysis_dir):
        """Write the fleet-size sweep summary."""
        report_path = os.path.join(analysis_dir, 'vehicle_comparison_summary.txt')
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("🚗 Vehicle Change Mode 분석 리포트\n")
            f.write("=" * 70 + "\n\n")
            f.write(f"분석 시각: {self.timestamp}\n")
            f.write(f"차량 수 범위: {min(self.scenario_labels.values())} ~ {max(self.scenario_labels.values())}대\n")
            f.write(f"각 시나리오당 반복 횟수: {self.num_iterations}\n\n")
            
            # table of headline metrics against fleet size
            f.write("-" * 70 + "\n")
            f.write("📊 차량 수별 성능 비교\n")
            f.write("-" * 70 + "\n\n")
            
            key_metrics = ['avg_lead_time', 'avg_wait_time', 'avg_throughput', 'sim_time']
            
            for metric in key_metrics:
                f.write(f"\n{metric}:\n")
                f.write(f"{'차량 수':<10} {'평균':<15} {'표준편차':<15}\n")
                f.write("-" * 45 + "\n")
                
                for scenario_label in sorted(scenario_statistics.keys()):
                    vehicle_count = self.scenario_labels[scenario_label]
                    if metric in scenario_statistics[scenario_label]:
                        s = scenario_statistics[scenario_label][metric]
                        f.write(f"{vehicle_count:<10} {s['mean']:<15.4f} {s['std']:<15.4f}\n")
                f.write("\n")
            
            f.write("=" * 70 + "\n")

    def _plot_summary_errorbar(self, statistics, analysis_dir):
        """Single-scenario error-bar figure."""
        key_metrics = {
            'avg_lead_time': ('평균 Lead Time', '초'),
            'avg_wait_time': ('평균 Wait Time', '초'),
            'avg_throughput': ('Throughput', 'jobs/s'),
            'sim_time': ('시뮬레이션 시간', '초')
        }
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Monte Carlo 시뮬레이션 통계 요약', fontsize=16, fontweight='bold')
        axes = axes.flatten()
        
        for idx, (metric, (title, unit)) in enumerate(key_metrics.items()):
            if metric in statistics:
                ax = axes[idx]
                s = statistics[metric]
                
                ax.errorbar([0], [s['mean']], yerr=[s['std']], 
                           fmt='o', markersize=12, capsize=10, capthick=2,
                           color=self.colorList[idx], ecolor='black', linewidth=2)
                
                ax.axhline(y=s['min'], color='blue', linestyle=':', alpha=0.5, label=f"최소: {s['min']:.2f}")
                ax.axhline(y=s['max'], color='red', linestyle=':', alpha=0.5, label=f"최대: {s['max']:.2f}")
                
                ax.set_xlim(-0.5, 0.5)
                ax.set_xticks([0])
                ax.set_xticklabels([f'{len(statistics[metric]["values"])} iterations'])
                ax.set_ylabel(f'{title} ({unit})', fontsize=11)
                ax.set_title(f'{title}\n평균: {s["mean"]:.2f} ± {s["std"]:.2f} {unit}', 
                            fontsize=12, fontweight='bold')
                ax.legend(loc='best', fontsize=9)
                ax.grid(True, axis='y', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(analysis_dir, 'montecarlo_summary.png'), dpi=150)
        plt.close()

    def _plot_vehicle_comparison(self, scenario_statistics, analysis_dir):
        """Error-bar figure comparing fleet sizes."""
        key_metrics = {
            'avg_lead_time': ('평균 Lead Time', '초'),
            'avg_wait_time': ('평균 Wait Time', '초'),
            'avg_throughput': ('Throughput', 'jobs/s'),
            'sim_time': ('시뮬레이션 시간', '초'),
            'global_planner_time': ('Global Planner 시간', '초'),
            'local_planner_time': ('Local Planner 시간', '초')
        }
        
        fig, axes = plt.subplots(3, 2, figsize=(16, 14))
        fig.suptitle('🚗 차량 수별 성능 비교 분석', fontsize=16, fontweight='bold')
        axes = axes.flatten()
        
        for idx, (metric, (title, unit)) in enumerate(key_metrics.items()):
            ax = axes[idx]
            
            vehicle_counts = []
            means = []
            stds = []
            
            for scenario_label in sorted(scenario_statistics.keys()):
                vehicle_count = self.scenario_labels[scenario_label]
                if metric in scenario_statistics[scenario_label]:
                    s = scenario_statistics[scenario_label][metric]
                    vehicle_counts.append(vehicle_count)
                    means.append(s['mean'])
                    stds.append(s['std'])
            
            if vehicle_counts:
                ax.errorbar(vehicle_counts, means, yerr=stds,
                           fmt='o-', markersize=8, capsize=8, capthick=2,
                           color=self.colorList[idx % len(self.colorList)],
                           ecolor='black', linewidth=2, alpha=0.7)
                
                ax.set_xlabel('차량 수', fontsize=11)
                ax.set_ylabel(f'{title} ({unit})', fontsize=11)
                ax.set_title(f'{title}', fontsize=12, fontweight='bold')
                ax.grid(True, alpha=0.3)
                ax.set_xticks(vehicle_counts)
        
        plt.tight_layout()
        plt.savefig(os.path.join(analysis_dir, 'vehicle_comparison.png'), dpi=150)
        plt.close()
        
        # per-metric detail figures
        for metric, (title, unit) in key_metrics.items():
            vehicle_counts = []
            means = []
            stds = []
            mins = []
            maxs = []
            
            for scenario_label in sorted(scenario_statistics.keys()):
                vehicle_count = self.scenario_labels[scenario_label]
                if metric in scenario_statistics[scenario_label]:
                    s = scenario_statistics[scenario_label][metric]
                    vehicle_counts.append(vehicle_count)
                    means.append(s['mean'])
                    stds.append(s['std'])
                    mins.append(s['min'])
                    maxs.append(s['max'])
            
            if vehicle_counts:
                plt.figure(figsize=(12, 7))
                plt.errorbar(vehicle_counts, means, yerr=stds,
                            fmt='o-', markersize=10, capsize=10, capthick=2.5,
                            color=self.colorList[0], ecolor='black', 
                            linewidth=2.5, alpha=0.8, label='평균 ± 표준편차')
                
                # minimum-to-maximum band
                plt.fill_between(vehicle_counts, mins, maxs, alpha=0.2, 
                               color=self.colorList[1], label='최소-최대 범위')
                
                plt.xlabel('차량 수', fontsize=13)
                plt.ylabel(f'{title} ({unit})', fontsize=13)
                plt.title(f'{title} - 차량 수별 비교\n(각 {self.num_iterations} 반복 평균)', 
                         fontsize=14, fontweight='bold')
                plt.legend(loc='best', fontsize=11)
                plt.grid(True, alpha=0.3)
                plt.xticks(vehicle_counts)
                plt.tight_layout()
                
                safe_metric_name = metric.replace('_', '-')
                plt.savefig(os.path.join(analysis_dir, f'{safe_metric_name}_by_vehicle.png'), dpi=150)
                plt.close()
