drone=3 t_env 1200
12 qmix unique_task train "results\models\qmix__2026-04-24_19-46-57"
16 qmix unique_task test

17 qmix no_unique_task train "results\models\qmix__2026-05-10_21-37-26"

drone=10 task_total=50
23 qmix_unique_1_scale_2 train "backend_wx\pymarl-master\results\models\qmix__2026-05-27_02-11-23" drone = 10
25 qmix_unique_0_scale_2 train "backend_wx\pymarl-master\results\models\qmix__2026-05-27_16-22-24" drone = 3
26 qmix_unique_0_scale_2 train "backend_wx\pymarl-master\results\models\qmix__2026-05-27_16-25-51" drone = 10
