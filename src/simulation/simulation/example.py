import numpy as np
import tqdm
import gymnasium as gym
from gymnasium.wrappers.time_limit import TimeLimit
from gymnasium.wrappers.record_video import RecordVideo

from sailboat_gym import CV2DRenderer, Observation, get_best_sail


theta_wind = np.deg2rad(90)


def generate_wind(_):
    wind_speed = 2
    return np.array([np.cos(theta_wind), np.sin(theta_wind)]) * wind_speed


env = gym.make('SailboatLSAEnv-v0',
               renderer=CV2DRenderer(),
               wind_generator_fn=generate_wind,
               video_speed=20,
               map_scale=.25,
               keep_sim_alive=True)

EPISODE_LENGTH = env.NB_STEPS_PER_SECONDS * 60 * 3  # 10 minutes

env = TimeLimit(env, max_episode_steps=EPISODE_LENGTH)
env = RecordVideo(env, video_folder='./output/videos/')

sail_angle = get_best_sail('SailboatLSAEnv-v0', theta_wind)
wanted_heading = np.deg2rad(-30)

print(
    f'Best sail angle: {np.rad2deg(sail_angle)} for wind angle: {np.rad2deg(theta_wind)}')


def ctrl(obs: Observation):
    rudder_angle = obs['theta_boat'][2] - wanted_heading
    return {'theta_rudder': np.array(rudder_angle), 'theta_sail': sail_angle}


truncated = False
obs, info = env.reset(seed=10)
env.render()
for t in tqdm.trange(EPISODE_LENGTH, desc='Running simulation'):
    obs, reward, terminated, truncated, info = env.step(ctrl(obs))
    if t % 100 == 0:
        wanted_heading += np.deg2rad(10)
        print(f'New wanted heading: {np.rad2deg(wanted_heading)}')
    if terminated or truncated:
        break
    env.render()
env.close()
