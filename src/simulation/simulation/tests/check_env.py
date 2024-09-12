import gymnasium as gym
from typing import Any, Tuple, Callable, Type, TypedDict
from sailboat_gym import SailboatEnv


def check_env_implementation(env: Type[SailboatEnv]):
    assert issubclass(
        env, SailboatEnv), f'env must be a subclass of {SailboatEnv.__name__}'
    assert hasattr(env, 'NB_STEPS_PER_SECONDS'), \
        f'env must have a {SailboatEnv.__name__}.NB_STEPS_PER_SECONDS static attribute'
    env_instance = env()
    assert hasattr(env_instance, 'render_mode'), \
        f'env must have a {SailboatEnv.__name__}.render_mode attribute'
    assert hasattr(env_instance, 'metadata'), \
        f'env must have a {SailboatEnv.__name__}.metadata attribute'
    assert isinstance(env_instance.metadata, dict), \
        f'env.metadata must be a dict'
    assert 'render_modes' in env_instance.metadata, \
        f'env.metadata must have a "render_modes" key'
    assert isinstance(env_instance.metadata['render_modes'], list), \
        f'env.metadata["render_modes"] must be a list'
    assert 'render_fps' in env_instance.metadata, \
        f'env.metadata must have a "render_fps" key'
    assert isinstance(env_instance.metadata['render_fps'], float), \
        f'env.metadata["render_fps"] must be a float'
    assert hasattr(env_instance, 'action_space'), \
        f'env must have a {SailboatEnv.__name__}.action_space attribute'
    assert hasattr(env_instance, 'observation_space'), \
        f'env must have a {SailboatEnv.__name__}.observation_space attribute'
    assert isinstance(env_instance.action_space, gym.Space), \
        f'env.action_space must be a gym.Space'
    assert isinstance(env_instance.observation_space, gym.Space), \
        f'env.observation_space must be a gym.Space'
    env_instance.close()
