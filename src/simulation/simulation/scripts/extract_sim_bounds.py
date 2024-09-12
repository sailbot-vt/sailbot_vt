import sys
sys.path.append('..')  # noqa
sys.path.append('.')  # noqa

import tqdm
import pickle
import click
import threading
import os
import os.path as osp
import numpy as np
import gymnasium as gym
from collections import defaultdict
from gymnasium.wrappers.time_limit import TimeLimit
from gymnasium.wrappers.record_video import RecordVideo

from sailboat_gym import CV2DRenderer, env_by_name

current_dir = osp.dirname(osp.abspath(__file__))
pkl_dir = osp.join(current_dir, '..', 'pkl')


global_theta_wind = 0
global_wind_velocity = None


def generate_wind(_):
    theta_wind_rad = np.deg2rad(global_theta_wind)
    return np.array([np.cos(theta_wind_rad), np.sin(theta_wind_rad)])*global_wind_velocity


def create_env(env_name, i):
    assert env_name in env_by_name.keys(), f'Unknown env name: {env_name}'
    env = gym.make(env_name,
                   renderer=CV2DRenderer(),
                   wind_generator_fn=generate_wind,
                   container_tag='mss1',
                   name=f'{i}',
                   keep_sim_alive=False)
    env = TimeLimit(env, max_episode_steps=env.NB_STEPS_PER_SECONDS*10) # 10 seconds
    # env = RecordVideo(env, video_folder='./output/videos/')
    return env


def get_vmc(obs):
    v = obs['dt_p_boat'][0:2]
    d = np.array([1, 0])  # x-axis
    return np.dot(v, d)


def deep_convert_to_dict(d):
    d = dict(d)
    for k, v in d.items():
        if isinstance(v, defaultdict):
            d[k] = deep_convert_to_dict(v)
    return d


def save_bounds(env_name: str, bounds):
    bounds = deep_convert_to_dict(bounds)
    if not osp.exists(pkl_dir):
        os.makedirs(pkl_dir, exist_ok=True)
    file_path = osp.join(
        pkl_dir,
        f'{env_name}_bounds_v_wind_{global_wind_velocity}.pkl')
    with open(file_path, 'wb') as f:
        pickle.dump(bounds, f)
    print(f'Saved bounds to file: {file_path}')


def run_simulation(env, bounds, theta_wind, theta_sail):
    global global_theta_wind
    global_theta_wind = theta_wind

    def ctrl(_):
        return {'theta_rudder': np.array(0), 'theta_sail': np.array(np.deg2rad(theta_sail))}

    obs, info = env.reset(seed=0)
    while True:
        obs, reward, terminated, truncated, info = env.step(ctrl(obs))

        vmc = get_vmc(obs)
        v_min, v_max = bounds[theta_wind][theta_sail]['vmc']
        bounds[theta_wind][theta_sail]['vmc'] = (
            min(v_min, vmc), max(v_max, vmc))

        for k, v in obs.items():
            if k in ['p_boat']:
                for d in range(v.shape[0]):
                    v_min, v_max = bounds[theta_wind][theta_sail][f'{k}_{d}']
                    bounds[theta_wind][theta_sail][f'{k}_{d}'] = (
                        min(v_min, v[d]), max(v_max, v[d]))
            else:
                v = np.linalg.norm(v)
                v_min, v_max = bounds[theta_wind][theta_sail][k]
                bounds[theta_wind][theta_sail][k] = (
                    min(v_min, v), max(v_max, v))

        if terminated or truncated:
            break
    env.close()


@click.command()
@click.option('--env-name', default=list(env_by_name.keys())[0], help='Env name', type=click.Choice(list(env_by_name.keys()), case_sensitive=False))
@click.option('--wind-velocity', default=1, help='Wind velocity', type=int)
def extract_sim_stats(env_name, wind_velocity):
    global global_wind_velocity
    global_wind_velocity = int(wind_velocity)

    envs = [create_env(env_name, i) for i in range(5)]

    bounds_by_wind_by_sail_by_var = defaultdict(
        lambda: defaultdict(lambda: defaultdict(lambda: (np.inf, -np.inf))))

    for theta_wind in tqdm.trange(0, 360, 5, desc='wind angle'):
        for theta_sail in tqdm.trange(-90, 90+1, 5, desc='sail angle', leave=False):
            threads = [
                threading.Thread(target=run_simulation,
                                 args=(env, bounds_by_wind_by_sail_by_var, theta_wind, theta_sail)) for env in envs]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
        save_bounds(env_name, bounds_by_wind_by_sail_by_var)


if __name__ == '__main__':
    extract_sim_stats()
