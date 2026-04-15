# ppo
Using Python 3.7.17 environment
gym                  0.15.7
opencv-python        4.13.0.92
protobuf             3.20.3
pytest               7.4.4
pytest-forked        1.6.0
scipy                1.7.3
tensorflow           1.14.0
tensorflow-estimator 1.14.0

检验
```
cd /home/user/drone-scheduling/backend-marl/ppo
export TMPDIR=/tmp TEMP=/tmp TMP=/tmp
uv run python -m baselines.run --alg=ppo2 --env=CartPole-v0 --log_interval=1
```