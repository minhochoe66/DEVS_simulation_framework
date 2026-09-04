import os
import sys
import argparse
import json
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

# Windows 콘솔에서 UTF-8 출력이 되도록 맞춘다
if sys.platform == 'win32':
    try:
        import io
        sys.stdout = io.TextIOWrapper(
            sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(
            sys.stderr.buffer, encoding='utf-8', errors='replace')
    except Exception:
        pass

# Tk 대화형 백엔드. 화면 없이 저장만 하려면 이 줄을 지우고 Agg를 쓴다
matplotlib.use('TkAgg')


def find_latest_timestamp_folder(base_dir: str) -> str:
    if not os.path.exists(base_dir):
        return ''
    entries = [d for d in os.listdir(base_dir)
               if os.path.isdir(os.path.join(base_dir, d))]
    if not entries:
        return ''
    # 이름순이 아니라 생성 시각 기준으로 최신 폴더를 고른다
    entries = sorted(entries, key=lambda d: os.path.getmtime(
        os.path.join(base_dir, d)))
    return entries[-1]


def load_map_json(map_path: str = 'JSON/map.json') -> dict:
    """map.json에서 장비와 대기 구역 정보를 읽는다."""
    if not os.path.exists(map_path):
        print(f'⚠️  Map file not found: {map_path}')
        return None
    try:
        with open(map_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        num_equipment = len(data.get("equipmentInfo", []))
        num_waiting_areas = len(data.get("WatingareaInfo", []))
        print(
            f'✅ Loaded map with {num_equipment} equipment(s) and {num_waiting_areas} waiting area(s)')
        return data
    except Exception as e:
        print(f'❌ Failed to load map: {e}')
        return None


def load_agent_csvs(agent_dir: str) -> dict:
    """Agent 폴더의 차량 CSV를 읽어 {vehicle_name: DataFrame}으로 반환한다."""
    data = {}
    if not agent_dir or not os.path.exists(agent_dir):
        return data
    for fname in sorted(os.listdir(agent_dir)):
        if fname.lower().endswith('.csv'):
            fpath = os.path.join(agent_dir, fname)
            try:
                df = pd.read_csv(fpath)
                # 필수 컬럼 확인
                if {'x', 'y'}.issubset(df.columns):
                    # 시간축을 만든다. 샘플 간격은 0.1 s로 가정한다
                    if 'timestamp' not in df.columns:
                        df['timestamp'] = np.arange(len(df)) * 0.1
                    data[fname.replace('.csv', '')] = df
            except Exception as e:
                print(f"Failed to load {fpath}: {e}")
    return data


def create_rectangle_points(x, y, length=1.0, width=1.0, yaw=0.0):
    local = np.array([[-length/2, -width/2], [length/2, -width/2],
                      [length/2, width/2], [-length/2, width/2]])
    rot = np.array([[np.cos(yaw), -np.sin(yaw)],
                    [np.sin(yaw), np.cos(yaw)]])
    return np.dot(local, rot.T) + np.array([x, y])


def draw_equipment(ax, map_data):
    """장비를 배경에 그린다."""
    if not map_data:
        return

    equipment_info = map_data.get('equipmentInfo', [])
    for eq in equipment_info:
        eq_id = eq.get('equipmentID', '')
        process_type = eq.get('processType', '')

        # 입력 포트
        if 'inputPort' in eq and eq['inputPort']:
            inp = eq['inputPort']
            pos = inp.get('position', {})
            bbox = inp.get('boundingBox', {})
            x, y = pos.get('x', 0), pos.get('y', 0)
            w, h = bbox.get('width', 5), bbox.get('height', 5)
            rect = Rectangle((x - w/2, y - h/2), w, h,
                             facecolor='#90EE90', edgecolor='#228B22',
                             alpha=0.4, linewidth=1.5, label='IN' if eq_id == equipment_info[0]['equipmentID'] else '')
            ax.add_patch(rect)
            ax.text(x, y, 'IN', ha='center', va='center',
                    fontsize=7, color='green', weight='bold')

        # 작업 위치
        if 'workPosition' in eq and eq['workPosition']:
            work = eq['workPosition']
            pos = work.get('position', {})
            bbox = work.get('boundingBox', {})
            x, y = pos.get('x', 0), pos.get('y', 0)
            w, h = bbox.get('width', 6), bbox.get('height', 6)
            rect = Rectangle((x - w/2, y - h/2), w, h,
                             facecolor='#FFD700', edgecolor='#B8860B',
                             alpha=0.3, linewidth=1.5, label='WORK' if eq_id == equipment_info[0]['equipmentID'] else '')
            ax.add_patch(rect)
            ax.text(x, y - h/2 - 1.5, eq_id, ha='center', va='top',
                    fontsize=8, color='black', weight='bold')

        # 출력 포트
        if 'outputPort' in eq and eq['outputPort']:
            out = eq['outputPort']
            pos = out.get('position', {})
            bbox = out.get('boundingBox', {})
            x, y = pos.get('x', 0), pos.get('y', 0)
            w, h = bbox.get('width', 5), bbox.get('height', 5)
            rect = Rectangle((x - w/2, y - h/2), w, h,
                             facecolor='#FFB6C1', edgecolor='#DC143C',
                             alpha=0.4, linewidth=1.5, label='OUT' if eq_id == equipment_info[0]['equipmentID'] else '')
            ax.add_patch(rect)
            ax.text(x, y, 'OUT', ha='center', va='center',
                    fontsize=7, color='red', weight='bold')


def draw_waiting_areas(ax, map_data):
    """대기 구역을 배경에 그린다."""
    if not map_data:
        return

    waiting_areas = map_data.get('WatingareaInfo', [])
    for area in waiting_areas:
        area_id = area.get('areaID', '')
        pos = area.get('position', {})
        bbox = area.get('boundingBox', {})

        x, y = pos.get('x', 0), pos.get('y', 0)
        w, h = bbox.get('width', 5), bbox.get('height', 5)

        # 대기 구역은 보라색 점선으로 표시한다
        rect = Rectangle((x - w/2, y - h/2), w, h,
                         facecolor='#E6E6FA', edgecolor='#9370DB',
                         alpha=0.3, linewidth=2, linestyle='--',
                         label='Waiting Area' if area_id == waiting_areas[0]['areaID'] else '')
        ax.add_patch(rect)

        # 'W' 표시와 구역 ID
        ax.text(x, y, 'W', ha='center', va='center',
                fontsize=8, color='purple', weight='bold')
        ax.text(x, y + h/2 + 1.0, area_id, ha='center', va='bottom',
                fontsize=6, color='purple', style='italic')


def animate_agents(agents: dict, map_data: dict = None, title: str = ''):
    if not agents:
        print('No agent CSV files found.')
        return

    # 모든 차량의 timestamp를 합쳐 정렬하고 샘플링하여 전역 시간축을 만든다
    all_times = []
    for df in agents.values():
        all_times.extend(df['timestamp'].tolist())
    if not all_times:
        print('No timestamps in CSVs.')
        return
    times = np.array(sorted(set(all_times)))
    # 프레임은 1000개로 제한한다
    if len(times) > 1000:
        idx = np.linspace(0, len(times)-1, 1000).astype(int)
        times = times[idx]

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.set_facecolor('#F5F5F5')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title(title or 'AMR Trajectory with Map')

    # 맵에서 표시 범위를 계산한다
    xmin, xmax, ymin, ymax = 0.0, 300.0, 0.0, 100.0
    if map_data:
        equipment_info = map_data.get('equipmentInfo', [])
        for eq in equipment_info:
            for port_key in ['inputPort', 'outputPort', 'workPosition']:
                if port_key in eq and eq[port_key]:
                    pos = eq[port_key].get('position', {})
                    bbox = eq[port_key].get('boundingBox', {})
                    x, y = pos.get('x', 0), pos.get('y', 0)
                    w, h = bbox.get('width', 5), bbox.get('height', 5)
                    xmin = min(xmin, x - w/2)
                    xmax = max(xmax, x + w/2)
                    ymin = min(ymin, y - h/2)
                    ymax = max(ymax, y + h/2)

        # 대기 구역도 범위에 넣는다
        waiting_areas = map_data.get('WatingareaInfo', [])
        for area in waiting_areas:
            pos = area.get('position', {})
            bbox = area.get('boundingBox', {})
            x, y = pos.get('x', 0), pos.get('y', 0)
            w, h = bbox.get('width', 5), bbox.get('height', 5)
            xmin = min(xmin, x - w/2)
            xmax = max(xmax, x + w/2)
            ymin = min(ymin, y - h/2)
            ymax = max(ymax, y + h/2)

    # 차량 궤적도 범위에 넣는다
    for df in agents.values():
        xmin = min(xmin, float(np.nanmin(df['x'])))
        xmax = max(xmax, float(np.nanmax(df['x'])))
        ymin = min(ymin, float(np.nanmin(df['y'])))
        ymax = max(ymax, float(np.nanmax(df['y'])))

    margin = 5.0
    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)

    # 장비 (배경)
    draw_equipment(ax, map_data)

    # 대기 구역 (배경)
    draw_waiting_areas(ax, map_data)

    # 차량마다 궤적 선과 현재 위치 사각형을 만든다
    colors = ['#ff7f0e', '#1f77b4', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    traj_lines = {}
    rect_patches = {}
    arrows = {}

    for i, (name, df) in enumerate(agents.items()):
        c = colors[i % len(colors)]
        line, = ax.plot([], [], '-', color=c, alpha=0.8,
                        linewidth=2, label=name)
        traj_lines[name] = line
        # 초기 사각형
        rect = plt.Polygon(create_rectangle_points(0, 0, 1.0, 1.0, 0.0),
                           closed=True, facecolor=c, edgecolor='black', alpha=0.7, linewidth=1.5)
        ax.add_patch(rect)
        rect_patches[name] = rect
        # 방향 화살표. 처음에는 None이고 갱신할 때 만든다
        arrows[name] = None

    # 범례
    ax.legend(loc='upper right')

    # 프레임 갱신
    def update(frame_idx):
        t = times[frame_idx]
        ax.set_title(f"AMR Trajectory (t={t:.1f}s)")
        artists = []
        for name, df in agents.items():
            # 현재 시각까지의 궤적
            df_t = df[df['timestamp'] <= t]
            if df_t.empty:
                continue

            # 꼬리 효과를 위해 최근 20개만 그린다
            if len(df_t) > 20:
                df_tail = df_t.tail(20)
            else:
                df_tail = df_t

            traj_lines[name].set_data(df_tail['x'].values, df_tail['y'].values)
            artists.append(traj_lines[name])

            # 현재 자세
            row = df_t.iloc[-1]
            x = float(row['x'])
            y = float(row['y'])
            yaw = float(row['yaw']) if 'yaw' in df.columns else 0.0
            rect_patches[name].set_xy(
                create_rectangle_points(x, y, 1.0, 1.0, yaw))
            artists.append(rect_patches[name])

            # 방향 화살표 갱신
            if arrows[name] is not None and arrows[name] in ax.patches:
                try:
                    arrows[name].remove()
                except Exception:
                    pass
            dx = 0.8 * np.cos(yaw)
            dy = 0.8 * np.sin(yaw)
            arrows[name] = ax.arrow(x, y, dx, dy, head_width=0.25, head_length=0.2,
                                    fc='black', ec='black', alpha=0.8)
            artists.append(arrows[name])
        return artists

    ani = animation.FuncAnimation(fig, update, frames=len(
        times), interval=30, blit=False, repeat=True)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize AMR CSV trajectories (no obstacles)')

    # 경로는 이 스크립트 위치를 기준으로 잡는다
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_base = script_dir  # 이 파일이 Visualizations/ 안에 있다

    parser.add_argument('--base', type=str, default=default_base,
                        help='Base directory containing timestamp folders')
    parser.add_argument('--foldername', type=str, default='',
                        help='Timestamp folder name (e.g., 20251014_152258). If empty, latest will be auto-selected')
    args = parser.parse_args()

    base_dir = args.base
    print(f'📂 Base directory: {base_dir}')

    # 폴더를 지정하지 않으면 최신 폴더를 쓴다
    if args.foldername:
        ts = args.foldername
        print(f'📌 Manually specified folder: {ts}')
    else:
        ts = find_latest_timestamp_folder(base_dir)
        if ts:
            print(f'🔍 Auto-detected latest folder: {ts}')

    if not ts:
        print('❌ No timestamp folder found under', base_dir)
        print('💡 Make sure you have run the simulation first to generate CSV files.')
        return

    ts_folder_path = os.path.join(base_dir, ts)

    # Agent 폴더는 iteration_1/Agent이거나 Agent다
    agent_dir_iteration = os.path.join(ts_folder_path, 'iteration_1', 'Agent')
    agent_dir_direct = os.path.join(ts_folder_path, 'Agent')

    if os.path.exists(agent_dir_iteration):
        agent_dir = agent_dir_iteration
        print(f'📊 Using timestamp folder: {ts}')
        print(f'📁 Agent data folder: {agent_dir} (iteration_1)')
    elif os.path.exists(agent_dir_direct):
        agent_dir = agent_dir_direct
        print(f'📊 Using timestamp folder: {ts}')
        print(f'📁 Agent data folder: {agent_dir}')
    else:
        print(f'❌ No Agent folder found in {ts_folder_path}')
        return

    agents = load_agent_csvs(agent_dir)
    if not agents:
        print('❌ No vehicle CSV files found!')
        return

    # Agent 폴더와 같은 위치의 map.json을 먼저 쓴다
    if os.path.exists(agent_dir_iteration):
        # iteration_1 아래에서 Agent를 찾은 경우
        local_map_path = os.path.join(
            ts_folder_path, 'iteration_1', 'map.json')
    else:
        # Agent 폴더를 바로 찾은 경우
        local_map_path = os.path.join(ts_folder_path, 'map.json')

    if os.path.exists(local_map_path):
        print(f'📍 Using local map config: {local_map_path}')
        map_data = load_map_json(local_map_path)
    else:
        print(f'⚠️ No local map.json found, using project root map.json')
        # 결과 폴더에 map.json이 없으면 프로젝트의 JSON/map.json을 쓴다
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        map_path = os.path.join(project_root, 'JSON', 'map.json')
        map_data = load_map_json(map_path)

    print(f'✅ Loaded {len(agents)} vehicle(s): {", ".join(agents.keys())}')
    print('🎬 Starting animation...')
    animate_agents(agents, map_data, title=f'AMR Trajectory - {ts}')


if __name__ == '__main__':
    main()
