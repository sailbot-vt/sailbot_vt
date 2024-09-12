#!/bin/zsh

# Test
python3 run_tests.py
if [ $? -ne 0 ]; then
    echo "❌ Tests failed"
    exit 1
fi

# Remove unnecessary files
rm -rf build dist sailboat_gym.egg-info

# Install dependencies
pip install build

# Build the distribution files
python3 -m build

# Upload the distribution files to PyPI
twine upload dist/*
