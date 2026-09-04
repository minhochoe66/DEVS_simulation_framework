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

# make UTF-8 output work on the Windows console
if sys.platform == 'win32':
    try:
        import io
        sys.stdout = io.TextIOWrapper(
            sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(
            sys.stderr.buffer, encoding='utf-8', errors='replace')
    except Exception:
        pass

# interactive Tk backend; for headless saving, drop this line and use Agg
matplotlib.use('TkAgg')


def find_latest_timestamp_folder(base_dir: str) -> str:
    if not os.path.exists(base_dir):
        return ''
    entries = [d for d in os.listdir(base_dir)
               if os.path.isdir(os.path.join(base_dir, d))]
    if not entries:
        return ''
    # pick the newest folder by modification time, not by name
    entries = sorted(entries, key=lambda d: os.path.getmtime(
        os.path.join(base_dir, d)))
    return entries[-1]


def load_map_json(map_path: str = 'JSON/map.json') -> dict:
    """Read the equipment and waiting areas from map.json."""
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
    """Read the vehicle CSVs in the Agent folder into {vehicle_name: DataFrame}."""
    data = {}
    if not agent_dir or not os.path.exists(agent_dir):
        return data
    for fname in sorted(os.listdir(agent_dir)):
        if fname.lower().endswith('.csv'):
            fpath = os.path.join(agent_dir, fname)
            try:
                df = pd.read_csv(fpath)
                # check the required columns
                if {'x', 'y'}.issubset(df.columns):
                    # build a time axis, assuming a 0.1 s sample interval
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
    """Draw the equipment as the background."""
    if not map_data:
        return

    equipment_info = map_data.get('equipmentInfo', [])
    for eq in equipment_info:
        eq_id = eq.get('equipmentID', '')
        process_type = eq.get('processType', '')

        # input port
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

        # work position
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

        # output port
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
    """Draw the waiting areas as the background."""
    if not map_data:
        return

    waiting_areas = map_data.get('WatingareaInfo', [])
    for area in waiting_areas:
        area_id = area.get('areaID', '')
        pos = area.get('position', {})
        bbox = area.get('boundingBox', {})

        x, y = pos.get('x', 0), pos.get('y', 0)
        w, h = bbox.get('width', 5), bbox.get('height', 5)

        # waiting areas are drawn as purple dashed outlines
        rect = Rectangle((x - w/2, y - h/2), w, h,
                         facecolor='#E6E6FA', edgecolor='#9370DB',
                         alpha=0.3, linewidth=2, linestyle='--',
                         label='Waiting Area' if area_id == waiting_areas[0]['areaID'] else '')
        ax.add_patch(rect)

        # the 'W' marker and the area ID
        ax.text(x, y, 'W', ha='center', va='center',
                fontsize=8, color='purple', weight='bold')
        ax.text(x, y + h/2 + 1.0, area_id, ha='center', va='bottom',
                fontsize=6, color='purple', style='italic')


def animate_agents(agents: dict, map_data: dict = None, title: str = ''):
    if not agents:
        print('No agent CSV files found.')
        return

    # merge every vehicle's timestamps, sort and sample them into one global time axis
    all_times = []
    for df in agents.values():
        all_times.extend(df['timestamp'].tolist())
    if not all_times:
        print('No timestamps in CSVs.')
        return
    times = np.array(sorted(set(all_times)))
    # cap the animation at 1000 frames
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

    # derive the view bounds from the map
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

        # include the waiting areas in the bounds
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

    # include the vehicle trajectories too
    for df in agents.values():
        xmin = min(xmin, float(np.nanmin(df['x'])))
        xmax = max(xmax, float(np.nanmax(df['x'])))
        ymin = min(ymin, float(np.nanmin(df['y'])))
        ymax = max(ymax, float(np.nanmax(df['y'])))

    margin = 5.0
    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)

    # equipment (background)
    draw_equipment(ax, map_data)

    # waiting areas (background)
    draw_waiting_areas(ax, map_data)

    # one trajectory line and one position marker per vehicle
    colors = ['#ff7f0e', '#1f77b4', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
    traj_lines = {}
    rect_patches = {}
    arrows = {}

    for i, (name, df) in enumerate(agents.items()):
        c = colors[i % len(colors)]
        line, = ax.plot([], [], '-', color=c, alpha=0.8,
                        linewidth=2, label=name)
        traj_lines[name] = line
        # initial marker
        rect = plt.Polygon(create_rectangle_points(0, 0, 1.0, 1.0, 0.0),
                           closed=True, facecolor=c, edgecolor='black', alpha=0.7, linewidth=1.5)
        ax.add_patch(rect)
        rect_patches[name] = rect
        # heading arrow: None at first, created on the first update
        arrows[name] = None

    # legend
    ax.legend(loc='upper right')

    # frame update
    def update(frame_idx):
        t = times[frame_idx]
        ax.set_title(f"AMR Trajectory (t={t:.1f}s)")
        artists = []
        for name, df in agents.items():
            # the trajectory up to the current time
            df_t = df[df['timestamp'] <= t]
            if df_t.empty:
                continue

            # draw only the last 20 points, giving a tail
            if len(df_t) > 20:
                df_tail = df_t.tail(20)
            else:
                df_tail = df_t

            traj_lines[name].set_data(df_tail['x'].values, df_tail['y'].values)
            artists.append(traj_lines[name])

            # current pose
            row = df_t.iloc[-1]
            x = float(row['x'])
            y = float(row['y'])
            yaw = float(row['yaw']) if 'yaw' in df.columns else 0.0
            rect_patches[name].set_xy(
                create_rectangle_points(x, y, 1.0, 1.0, yaw))
            artists.append(rect_patches[name])

            # update the heading arrow
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

    # resolve paths relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_base = script_dir  # this file lives inside Visualizations/

    parser.add_argument('--base', type=str, default=default_base,
                        help='Base directory containing timestamp folders')
    parser.add_argument('--foldername', type=str, default='',
                        help='Timestamp folder name (e.g., 20251014_152258). If empty, latest will be auto-selected')
    args = parser.parse_args()

    base_dir = args.base
    print(f'📂 Base directory: {base_dir}')

    # with no folder given, use the newest one
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

    # the Agent folder is either iteration_1/Agent or Agent
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

    # prefer the map.json sitting next to the Agent folder
    if os.path.exists(agent_dir_iteration):
        # the Agent folder was found under iteration_1
        local_map_path = os.path.join(
            ts_folder_path, 'iteration_1', 'map.json')
    else:
        # the Agent folder was found directly
        local_map_path = os.path.join(ts_folder_path, 'map.json')

    if os.path.exists(local_map_path):
        print(f'📍 Using local map config: {local_map_path}')
        map_data = load_map_json(local_map_path)
    else:
        print(f'⚠️ No local map.json found, using project root map.json')
        # with no map.json in the run folder, fall back to the project's JSON/map.json
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        map_path = os.path.join(project_root, 'JSON', 'map.json')
        map_data = load_map_json(map_path)

    print(f'✅ Loaded {len(agents)} vehicle(s): {", ".join(agents.keys())}')
    print('🎬 Starting animation...')
    animate_agents(agents, map_data, title=f'AMR Trajectory - {ts}')


if __name__ == '__main__':
    main()
