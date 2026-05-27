# 中期
## qmix
drone=3 t_env 1200
12 qmix unique_task train "results\models\qmix__2026-04-24_19-46-57"
16 qmix unique_task test

17 qmix no_unique_task train "results\models\qmix__2026-05-10_21-37-26"

# 期末
## qmix
drone =10 train episode_max_steps = 2000
41 qmix_unique_1 backend_wx\pymarl-master\results\models\qmix__2026-05-27_21-29-57
42 qmix_unique_0 backend_wx\pymarl-master\results\models\qmix__2026-05-27_21-30-32

drone =10 test episode_max_steps = 3000
46 qmix_unique_1
44 qmix_unique_0

drone=3 train
31 qmix_unique_1 backend_wx\pymarl-master\results\models\qmix__2026-05-27_17-13-55
25 qmix_unique_0 "backend_wx\pymarl-master\results\models\qmix__2026-05-27_16-22-24"

drone=3 test
36 qmix_unique_1
37 qmix_unique_0

# 贪心
drone = 3 test
completion=1.0000, on_time=0.9000, avg_delay=9.8500, avg_wait=18.8000, avg_delivery=166.7000, avg_gen_to_done=185.5000, avg_gen_interval=55.5789, max_delivery=326.0000, steps=1280

drone = 10 test
Mean Metrics
completion_rate:        0.9222
on_time_rate:           0.9699
avg_delay:              1.1005
avg_wait_time_to_load:  0.0000
avg_delivery_time:      146.6489
avg_gen_to_done:        146.6489
avg_generation_time:    28.0169
total_energy_consumed:   89417.1596
total_generated (mean):   60.0