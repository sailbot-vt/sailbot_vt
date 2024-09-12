from sailboat_gym import env_by_name, is_debugging

from .check_env import check_env_implementation


def check_all():
    try:
        print('-- Checking all environments --')
        for name, env in env_by_name.items():
            print(f'\tChecking [{name}]...')
            check_env_implementation(env)
            print(f'\t[{name}] OK\n')
    except Exception as e:
        if is_debugging():
            raise e
        else:
            print(f'Error: {repr(e)}')
            print('\nℹ️  Run with DEBUG=1 to have traceback')
            exit(1)
